"""G0' power check: three mutants of the new sched.py paths must each FAIL the gate."""
import sys
import numpy as np
import g0_gate as G
S = G.sched
orig_cost, orig_tt = S.GantryDP.cost, S.traverse_time

def m1(self):                       # ignores t_fold_first (retract never skipped)
    return self.h[:, self.r0]
def m2(dl, dr, t_fold=0.0, rail_cmd=0.0):   # rail_cmd ignored
    return orig_tt(dl, dr, t_fold, 0.0)
res = {}
S.GantryDP.cost = property(m1); res['cost ignores T0'] = G.main(); S.GantryDP.cost = orig_cost
S.traverse_time = m2; res['rail_cmd ignored'] = G.main(); S.traverse_time = orig_tt
orig_dt = S._dur_table
def m3(inst, g):                    # serial off: parallel slots even when serial_arms
    from dataclasses import replace
    return orig_dt(replace(inst, serial_arms=False), g)
S._dur_table = m3; res['serial ignored'] = G.main(); S._dur_table = orig_dt
print({k: ('LOLOS (BURUK)' if v else 'TERTANGKAP') for k, v in res.items()})
sys.exit(0 if not any(res.values()) else 1)
