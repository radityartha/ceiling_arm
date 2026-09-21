"""g20 A4, mechanical: LQ = passing quartets in original order; trial i = LQ[(i-1) mod |LQ|], i = 1..10."""
import json, sys
res = json.load(open(sys.argv[1]))
LQ = [r for r in res if r['ok']]
if not LQ:
    sys.exit('|LQ| = 0 -- langkah 5 TIDAK dijalankan (A4)')
for i in range(1, 11):
    r = LQ[(i - 1) % len(LQ)]
    print(i, *(','.join(f'{v:.3f}' for v in t) for t in r['targets']), f"# kuartet {r['quad']}", file=sys.stderr)
    print(i, *(','.join(f'{v:.3f}' for v in t) for t in r['targets']))
