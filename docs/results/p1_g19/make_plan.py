"""docs/p1_g19_hw.md A4 assignment, mechanically: task 0 = LA[0]; odd trials (->B) = LB[0..4];
even trials (->A) = LA[1..5]; a list that is too short is CYCLED in order. No new candidates.
Writes g19_plan.txt ("i rail a1xyz a2xyz") for batch.sh."""
import json, os
D = os.path.dirname(os.path.abspath(__file__))
LA = [r for r in json.load(open(f'{D}/g19_screen_0.550.json')) if r['ok']]
LB = [r for r in json.load(open(f'{D}/g19_screen_0.950.json')) if r['ok']]
print(f'LA (rel 0.550) lolos {len(LA)}: {[r["pair"] for r in LA]}')
print(f'LB (rel 0.950) lolos {len(LB)}: {[r["pair"] for r in LB]}')
assert LA and LB, 'daftar kosong -- A4 tidak dapat dipenuhi, BERHENTI'
rows = [(0, 0.550, LA[0])]
for i in range(1, 11):
    rows.append((i, 0.950, LB[(i // 2) % len(LB)]) if i % 2 else (i, 0.550, LA[(i // 2) % len(LA)]))
fmt = lambda p: ','.join(f'{v:.3f}' for v in p)
with open(f'{D}/g19_plan.txt', 'w') as f:
    for i, rail, r in rows:
        f.write(f'{i} {rail:.3f} {fmt(r["a1"])} {fmt(r["a2"])}\n')
        print(f'{i:2d} rel {rail:.3f}  B2#{r["pair"]:<2d} a1 {fmt(r["a1"])}  a2 {fmt(r["a2"])}')
