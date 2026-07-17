#!/usr/bin/env python3
"""E6 batch pick driver: round-robin 7 reachable objects x N trials.

Publishes /grasp_target then /gantry_reach_executor/pick 'target', waits for
the CSV row count to grow (pick finished) or a per-pick timeout, and scrapes
the executor's own log for the terminal SUCCESS/FAILED line so failures are
categorized by the executor's own reasoning (no invented categories).
"""
import re
import subprocess
import sys
import time

CSV = '/tmp/e6.csv'
LOG = '/tmp/e6_pick_stack.log'
OBJECTS = ['obj_0', 'obj_1', 'obj_3', 'obj_4', 'obj_5', 'obj_6', 'obj_7']
TRIALS_PER_OBJ = 5
# A pick can retry across up to 4 candidates (IK/plan/exec each ~10-60s), so
# a single pick has been observed to take up to ~160s end-to-end (see obj_1
# round-1: 1st candidate exec-aborted, 2nd candidate succeeded at 161s total).
# Soft deadline just warns; HARD_CAP is when we truly give up and move on.
PICK_TIMEOUT = 240.0
HARD_CAP = 420.0

TRIAL_LOG = sys.argv[1] if len(sys.argv) > 1 else '/tmp/e6_trials.csv'
# Skip trials already recorded in an existing TRIAL_LOG (round,obj) so a
# resumed run doesn't redo (or double-fire onto a still-busy executor) work
# from a previous partial run.
SKIP = set()
try:
    with open(TRIAL_LOG) as f:
        next(f)
        for line in f:
            parts = line.split(',', 2)
            if len(parts) >= 2:
                SKIP.add((int(parts[0]), parts[1]))
except (FileNotFoundError, StopIteration):
    pass
_FRESH_LOG = not SKIP


def csv_lines():
    try:
        with open(CSV) as f:
            return sum(1 for _ in f)
    except FileNotFoundError:
        return 0


def log_size():
    try:
        with open(LOG, 'rb') as f:
            f.seek(0, 2)
            return f.tell()
    except FileNotFoundError:
        return 0


def pub(topic, msg_type, data):
    subprocess.run(
        ['ros2', 'topic', 'pub', '-1', topic, msg_type, f"{{data: '{data}'}}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)


def scrape_result(off0):
    with open(LOG, 'rb') as f:
        f.seek(off0)
        chunk = f.read().decode(errors='replace')
    m = re.search(r'>>> (\S+): SUCCESS -- (.+)', chunk)
    if m:
        return 'SUCCESS', m.group(2).strip()
    m = re.search(r'>>> (\S+): FAILED -- (.+?)\.', chunk)
    if m:
        reason = m.group(2)
        if 'UNREACHABLE' in reason or 'no IK' in reason:
            cat = 'IK-31'
        elif 'execution aborted' in reason:
            cat = 'exec-abort'
        elif 'no collision-free plan' in reason or 'blocked' in reason:
            cat = 'plan-fail'
        else:
            cat = 'other-fail'
        return cat, reason.strip()
    return 'TIMEOUT-no-log-match', chunk[-400:]


def main():
    if _FRESH_LOG:
        with open(TRIAL_LOG, 'w') as tf:
            tf.write('round,obj,t_start,csv_rows_before,csv_rows_after,result,detail\n')
    else:
        print(f'resuming: skipping {len(SKIP)} already-recorded trials', flush=True)

    n = 0
    for rnd in range(1, TRIALS_PER_OBJ + 1):
        for obj in OBJECTS:
            n += 1
            if (rnd, obj) in SKIP:
                print(f'[{n}] round {rnd} obj {obj} -- already recorded, skip', flush=True)
                continue
            rows_before = csv_lines()
            off0 = log_size()
            t0 = time.time()
            print(f'[{n}] round {rnd} obj {obj} -- firing pick', flush=True)
            pub('/grasp_target', 'std_msgs/String', obj)
            time.sleep(1.5)
            pub('gantry_reach_executor/pick', 'std_msgs/String', 'target')

            deadline = time.time() + PICK_TIMEOUT
            hard_deadline = time.time() + HARD_CAP
            warned = False
            while time.time() < hard_deadline:
                if csv_lines() > rows_before:
                    break
                if time.time() > deadline and not warned:
                    print(f'    ... still waiting past {PICK_TIMEOUT:.0f}s '
                          f'(candidate retry in progress?)', flush=True)
                    warned = True
                time.sleep(1.0)
            rows_after = csv_lines()
            result, detail = scrape_result(off0)
            dt = time.time() - t0
            print(f'    -> {result} ({dt:.1f}s) {detail[:120]}', flush=True)
            with open(TRIAL_LOG, 'a') as tf:
                tf.write(f'{rnd},{obj},{t0:.1f},{rows_before},{rows_after},'
                         f'{result},"{detail[:200]}"\n')
            # brief settle before the next trigger
            time.sleep(2.0)

    print('DONE', flush=True)


if __name__ == '__main__':
    main()
