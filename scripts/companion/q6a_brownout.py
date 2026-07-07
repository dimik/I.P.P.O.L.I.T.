#!/usr/bin/env python3
"""q6a_brownout.py — companion brownout guard.

The Q6A is powered from the robot's 14.8 V battery (via a 12 V buck), so if the battery dies the companion
gets an UNCLEAN power cut — which risks the ext4 root and a cDSP wedge (we've seen how badly the Q6A handles
unclean shutdowns). This daemon watches the robot battery (Valetudo REST over the USB link) and, while
DISCHARGING:
  - at WARN%  : logs + speaks a warning, and (optionally) sends the robot home to dock so it recharges;
  - at CRIT%  : does a CLEAN `systemctl poweroff` of the Q6A before the cut.
Only acts while discharging (never during charging), with one-shot latches so it doesn't spam.

Non-ROS (urllib + ssh + systemctl); runs as q6a-brownout.service. Config via env (see below).
"""
import json, os, subprocess, time, urllib.request

HOST = os.environ.get('Q6A_ROBOT_HOST', 'http://192.168.10.1')
ROBOT_SSH = os.environ.get('Q6A_ROBOT_SSH', 'robot-usb')
WARN = int(os.environ.get('Q6A_BROWNOUT_WARN', '25'))
CRIT = int(os.environ.get('Q6A_BROWNOUT_CRIT', '12'))
POLL = int(os.environ.get('Q6A_BROWNOUT_POLL', '60'))          # seconds between checks
SEND_HOME = os.environ.get('Q6A_BROWNOUT_HOME', '0') != '0'    # at WARN, send robot to dock (off: AVA already auto-docks when low)
POWEROFF = os.environ.get('Q6A_BROWNOUT_POWEROFF', '1') != '0'  # at CRIT, clean-poweroff the Q6A
SPEAK = os.environ.get('Q6A_BROWNOUT_SPEAK', '1') != '0'
DRY_RUN = os.environ.get('Q6A_BROWNOUT_DRYRUN', '0') != '0'    # log actions but don't execute (for testing)


def log(m):
    print(f'[brownout] {m}', flush=True)


def battery():
    """(level%, flag) from Valetudo; flag in {charging, discharging, charged, none}. (None,None) on error."""
    try:
        r = urllib.request.urlopen(f'{HOST}/api/v2/robot/state', timeout=8)
        for x in json.load(r)['attributes']:
            if x.get('__class') == 'BatteryStateAttribute':
                return int(x['level']), x.get('flag', 'none')
    except Exception as e:
        log(f'battery read failed: {e}')
    return None, None


def charging():
    """True=charging, False=discharging, None=unknown. Valetudo's battery flag is BROKEN on the D10S Pro
    (stuck 'none'), so use AVA's authoritative charge_state, which a boot-hook poller keeps in the robot's
    /tmp/charge_state (values seen: 'not charge' = discharging)."""
    try:
        r = subprocess.run(['ssh', '-o', 'ConnectTimeout=5', '-o', 'StrictHostKeyChecking=no', ROBOT_SSH,
                            'cat /tmp/charge_state'], capture_output=True, text=True, timeout=10)
        s = r.stdout.strip().lower()
        if not s:
            return None
        return ('charg' in s) and ('not' not in s)
    except Exception as e:
        log(f'charge_state read failed: {e}')
        return None


def speak(text):
    if not SPEAK:
        return
    try:
        subprocess.run(['ssh', '-o', 'ConnectTimeout=5', '-o', 'StrictHostKeyChecking=no', ROBOT_SSH,
                        'chroot /data/chroot python3 /opt/speak.py'],
                       input=text.encode(), timeout=40, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        log(f'speak failed: {e}')


def send_home():
    try:
        req = urllib.request.Request(f'{HOST}/api/v2/robot/capabilities/BasicControlCapability',
                                     data=b'{"command":"home"}', method='PUT',
                                     headers={'Content-Type': 'application/json'})
        urllib.request.urlopen(req, timeout=8)
        log('sent robot home to dock')
    except Exception as e:
        log(f'send_home failed: {e}')


def poweroff():
    log('CLEAN POWEROFF of the Q6A now')
    if not DRY_RUN:
        subprocess.run(['sudo', 'systemctl', 'poweroff'])


def main():
    log(f'up: WARN={WARN}% CRIT={CRIT}% poll={POLL}s home={SEND_HOME} poweroff={POWEROFF} '
        f'speak={SPEAK} dry_run={DRY_RUN} host={HOST}')
    warned = homed = False
    while True:
        level, _ = battery()
        chg = charging()                        # True / False / None (AVA charge_state; Valetudo flag broken)
        if level is not None and chg is not None:
            if chg:
                warned = homed = False          # reset latches while charging
            else:                               # discharging
                if level <= CRIT:
                    log(f'CRIT: battery {level}% (discharging) <= {CRIT}%')
                    speak(f'Battery critical at {level} percent, shutting down the companion')
                    if POWEROFF:
                        poweroff()
                elif level <= WARN and not warned:
                    warned = True
                    log(f'WARN: battery {level}% (discharging) <= {WARN}%')
                    speak(f'Battery low at {level} percent, returning to dock')
                    if SEND_HOME and not homed:
                        homed = True
                        send_home()
        time.sleep(POLL)


if __name__ == '__main__':
    main()
