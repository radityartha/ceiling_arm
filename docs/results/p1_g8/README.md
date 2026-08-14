# G8 raw results — 63 minutes of exact-solver compute

Backing data for [`docs/p1_g8_sched2.md §B`](../../p1_g8_sched2.md). Committed
because the JSON is cheap to store and expensive to recompute: `g8_part1.json`
alone is **27.2 min** of `sched.solve_exact`, and the `lb_subset` fields in
`g8_part2.json` are another **36.0 min**.

| file | instances | what it holds |
|---|---|---|
| `g8_part1.json` | 140 | Part I — exact makespan + 5 schedulers + K4 verdict per instance (the K1 gap table) |
| `g8_part2.json` | 50 | Part II — 5 schedulers + `LB_analytic` + `LB_subset` (the bracket table) |
| `g8_holdout.json` | 60 | seeds 10–14, never seen while anything was designed — the honest test of `pose-tour+wide` |
| `g8_ablate.json` | 40 | D7 stage ablation: `full` / `nn-only` / `cover-order` / `no-refine` / `neither` |

## Reproducing the tables

```bash
cd ros2_ws/src/reachability_gng
cp ../../../docs/results/p1_g8/g8_*.json /tmp/          # or pass --part1/--part2
python3 test/eval_sched_heur.py report
```

To recompute from nothing (needs `/tmp/cap_g{1,2}_rail160.npz`, see
`ros2_ws/src/reachability_gng/data/README.md`), delete the JSON and re-run
`part1` / `part2` / `part2lb` / `holdout` / `ablate`. Every subcommand is
**incremental** — it skips instances already present in the file, so an
interrupted run resumes instead of restarting.

## Headline, so nobody has to re-derive it

`pose-tour`: mean gap **2.45%** (passes the locked ≤ 5%), max gap **24.31%**
(fails the locked ≤ 10%) → **K1 NOT met**, reported as failed rather than
having the threshold moved. Exactly optimal on 72/140. 1250 schedules passed
the K4 replay with 0 violations.
