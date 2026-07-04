//==============================================================================
//
//  Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
//  All Rights Reserved.
//  Confidential and Proprietary - Qualcomm Technologies, Inc.
//
//==============================================================================

#ifndef QUALLA_DETAIL_THREADPOOL_HPP
#define QUALLA_DETAIL_THREADPOOL_HPP

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace qualla {

// Generic thread pool
class ThreadPool {
 public:
  size_t size() const { return _n_threads; }

  // Check for queued up jobs
  bool busy() {
    std::unique_lock<std::mutex> lock(_queue_mutex);
    return !_jobs.empty();
  }

  // Enque single job
  void enqueue(const std::function<void()>& job) {
    _queue_mutex.lock();
    _poll = _enable_polling;
    _jobs.push(job);
    _queue_mutex.unlock();
    _mutex_condition.notify_one();
  }

  // Enque multiple jobs
  // Avoids extra latency due the race to lock mutex between enque() and threadLoop()
  void enqueue(const std::vector<std::function<void()>>& job_list) {
    _queue_mutex.lock();
    _poll = _enable_polling;
    for (auto& j : job_list) _jobs.push(j);
    _queue_mutex.unlock();
    _mutex_condition.notify_all();
  }

  // Start worker threads
  void start(unsigned int n_threads, uint64_t cpumask = 0, bool polling = false);

  // Stop worker threads
  void stop();

  // Susped worker threads (stop polling)
  void suspend();

  const std::vector<std::thread::id> getThreadIds() const;

 private:
  size_t _n_threads{0};
  volatile bool _terminate{false};             // Tells threads to stop looking for jobs
  volatile bool _poll{false};                  // Tells threads to poll or not
  uint64_t _cpumask{0};                        // Bind worker threads to select cpus
  bool _enable_polling{false};                 // Use polling to wait for jobs
  std::mutex _queue_mutex{};                   // Prevents data races to the job queue
  std::condition_variable _mutex_condition{};  // Allows threads to wait on new jobs or termination
  std::vector<std::thread> _threads;
  std::queue<std::function<void()>> _jobs;

  void loop(uint32_t ti);
};

}  // namespace qualla

#endif  // QUALLA_DETAIL_THREADPOOL_HPP
