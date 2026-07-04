//==============================================================================
//
//  Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
//  All Rights Reserved.
//  Confidential and Proprietary - Qualcomm Technologies, Inc.
//
//==============================================================================

#include "qualla/detail/threadpool.hpp"
#include <chrono>

#if defined(__APPLE__)
static bool __thread_affinity(uint64_t mask) { return true; }

#elif defined(_WIN32)
#include <stdint.h>
#include <windows.h>

static bool __thread_affinity(uint64_t mask) {
  HANDLE h    = GetCurrentThread();
  DWORD_PTR m = mask;

  m = SetThreadAffinityMask(h, m);

  return m != 0;
}

#else

#if defined(__ANDROID__)
#include <sched.h>

int32_t libc_setaffinity(pid_t pid, size_t cpusetsize, const cpu_set_t *mask) {
    int32_t err = sched_setaffinity(pid, cpusetsize, mask);
    return err < 0 ? errno : err;
}

int32_t libc_getaffinity(pid_t pid, size_t cpusetsize, cpu_set_t *mask) {
    int32_t err = sched_getaffinity(pid, cpusetsize, mask);
    return err < 0 ? errno : err;
}

#define SET_AFFINITY(a, b) libc_setaffinity(0, a, b)
#define GET_AFFINITY(a, b) libc_getaffinity(0, a, b)

#elif defined(__QNXNTO__)
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/neutrino.h>
#include <sys/procfs.h>
#include <sys/syspage.h>

#include <string>
#include <cstring>
#include <mutex>

// Hack for platform with 32 CPUs or less
struct CpuSet {
  unsigned mask;
};

typedef struct CpuSet cpu_set_t;

static inline void qnx_cpu_zero(cpu_set_t *cpuset) {
  if (RMSK_SIZE(_syspage_ptr->num_cpu) == 1) {
    cpuset->mask = 0U;
  } else {
    assert(RMSK_SIZE(_syspage_ptr->num_cpu) == 1);
  }
}

static inline bool qnx_cpu_equal(cpu_set_t *cpuset1, cpu_set_t *cpuset2) {
  return (cpuset1->mask == cpuset2->mask);
}

#define CPU_ISSET(a, b) RMSK_ISSET(a, b.mask)
#define CPU_SET(a, b)   RMSK_SET(a, b.mask)
#define CPU_ZERO(a)     qnx_cpu_zero(a)
#define CPU_EQUAL(a, b) qnx_cpu_equal(a, b)

static int qnxSgetThreadAffinity(bool isSet, cpu_set_t *cpuSet) {
  int ret = -1;
  procfs_threadctl tctl;
  std::string procStr = "/proc/" + std::to_string(getpid()) + "/as";

  if (cpuSet == nullptr) {
    return -EINVAL;
  }

  tctl.tid = static_cast<_Uint32t>(gettid());
  tctl.cmd = _NTO_TCTL_RUNMASK_GET_AND_SET_INHERIT;
  memset(tctl.data, 0, sizeof(tctl.data));

  struct _thread_runmask *tr = reinterpret_cast<struct _thread_runmask *>(&tctl.data);
  tr->size                 = RMSK_SIZE(_syspage_ptr->num_cpu);

  unsigned *runmask     = reinterpret_cast<unsigned *>(reinterpret_cast<uint8_t *>(tr) + sizeof(tr->size));
  unsigned *inheritMask = reinterpret_cast<unsigned *>(
      reinterpret_cast<uint8_t *>(runmask) + static_cast<long unsigned int>(tr->size) * sizeof(unsigned));

  if (isSet) {
    *runmask     = cpuSet->mask;
    *inheritMask = cpuSet->mask;
  } else {
    cpuSet->mask = 0U;
  }

  // Exclusive access to the proc fs with multi threads
  {
    static std::mutex access_mtx;
    std::lock_guard<std::mutex> lock(access_mtx);
    int fd = open(procStr.c_str(), O_RDWR);
    if (-1 == fd) {
      return errno;
    }
    ret = devctl(fd, DCMD_PROC_THREADCTL, &tctl, sizeof(tctl), NULL);
    close(fd);
  }
  if (!isSet && !ret) {
    // for get affinity case
    cpuSet->mask = *runmask;
  }
  return ret;
}

static inline int qnx_set_affinity(size_t size, const cpu_set_t *cpuset) {
  (void)size;
  return qnxSgetThreadAffinity(true, const_cast<cpu_set_t *>(cpuset));
}

static inline int qnx_get_affinity(size_t size, cpu_set_t *cpuset) {
  (void)size;
  return qnxSgetThreadAffinity(false, cpuset);
}

#define SET_AFFINITY(a, b) qnx_set_affinity(a, b)
#define GET_AFFINITY(a, b) qnx_get_affinity(a, b)

#else // POSIX
#include <pthread.h>
#define SET_AFFINITY(a, b) pthread_setaffinity_np(pthread_self(), a, b)
#define GET_AFFINITY(a, b) pthread_getaffinity_np(pthread_self(), a, b)
#endif

static bool __thread_affinity(uint64_t mask) {
  cpu_set_t cpuset;
  int32_t err;

  CPU_ZERO(&cpuset);

  for (uint32_t i = 0; i < 64; i++) {
    if ((1ULL << static_cast<uint32_t>(i)) & mask) {
      CPU_SET(i, &cpuset);
    }
  }

  err = SET_AFFINITY(sizeof(cpuset), &cpuset);

  if (err != 0) {
    fprintf(stderr,
            "warn: failed to set affinity mask 0x%llx (err %d)\n",
            static_cast<unsigned long long>(mask),
            static_cast<int>(err));
  }

  return err == 0;
}

#endif

#ifdef _MSC_VER
static inline void __cpu_relax(void) { YieldProcessor(); }
#else
#if defined(__aarch64__) || defined(__arm__)
static inline void __cpu_relax(void) { __asm__ volatile("yield" ::: "memory"); }
#else
static inline void __cpu_relax(void) { __asm__ volatile("rep; nop" ::: "memory"); }
#endif
#endif

namespace qualla {

void ThreadPool::stop() {
  _queue_mutex.lock();
  _terminate = true;
  _queue_mutex.unlock();
  _mutex_condition.notify_all();

  for (auto& t : _threads) {
    t.join();
  }
  _threads.clear();
}

void ThreadPool::start(unsigned int n_threads, uint64_t cpumask, bool polling) {
  _enable_polling = polling;
  _n_threads      = n_threads ? n_threads : std::thread::hardware_concurrency();
  _cpumask        = cpumask;
  _poll           = false;  // always start non-polling (enqueue will enable as needed)
  for (uint32_t i = 0; i < _n_threads; ++i) {
    _threads.emplace_back(std::thread(&ThreadPool::loop, this, i));
  }
}

void ThreadPool::suspend() {
  std::unique_lock<std::mutex> lock(_queue_mutex);
  _poll = false;
}

const std::vector<std::thread::id> ThreadPool::getThreadIds() const {
  std::vector<std::thread::id> threadIds;
  for (auto& thread : _threads) {
    threadIds.push_back(thread.get_id());
  }
  return threadIds;
}

void ThreadPool::loop(uint32_t) {
  if (_cpumask) {
    __thread_affinity(_cpumask);
  }

  std::unique_lock<std::mutex> lock{_queue_mutex, std::defer_lock};
  auto _last_work = std::chrono::steady_clock::now();  // adaptive: spin window then block

  while (!_terminate) {
    lock.lock();

    if (!_jobs.empty()) {
      // Dispatch front job
      auto j = _jobs.front();
      _jobs.pop();
      lock.unlock();
      j();
      _last_work = std::chrono::steady_clock::now();
    } else {
      // No jobs. Wait
      if (_poll && (std::chrono::steady_clock::now() - _last_work) < std::chrono::milliseconds(100)) {
        lock.unlock();
        __cpu_relax();
      } else {
        _mutex_condition.wait(lock);
        lock.unlock();
        _last_work = std::chrono::steady_clock::now();
      }
    }
  }
}

}  // namespace qualla
