"""G31 A5 driver: screen R35 seeds as g31_sched.py writes them -- rotation-using seeds first, the rest last."""
import json, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
SEEDS = [1, 2, 13, 16, 17, 18, 20, 21, 22, 23, 26, 29, 33, 35, 36, 40, 42]
done, later = set(), []
def cand():
    try:
        return {r['seed']: r for r in json.load(open(os.path.join(HERE, 'g31_candidates.json')))}
    except Exception:
        return {}
def screen(s):
    print(f'== screen seed {s} {time.strftime("%T")}', flush=True)
    subprocess.run([sys.executable, '-u', 'g31_screen.py', '--seeds', str(s), '--variant', 'R35'], cwd=HERE)
    done.add(s)
while len(done) < len(SEEDS):
    c = cand()
    sched_done = not any('g31_sched.py' in l for l in os.popen('ps -eo cmd').read().splitlines() if 'python3' in l)
    todo = [s for s in SEEDS if s in c and s not in done and s not in later]
    for s in todo:
        if sum(c[s]['R35']['rot_moves'].values()) > 0:
            screen(s)
        else:
            later.append(s)
            print(f'== seed {s} tanpa rotasi -> akhir', flush=True)
    if sched_done and all(s in c for s in SEEDS) and all(s in done or s in later for s in SEEDS):
        for s in later:
            if s not in done:
                screen(s)
    if not todo:
        time.sleep(30)
print('== driver selesai', time.strftime('%T'), flush=True)
