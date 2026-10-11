"""G54 (docs/p1_g54_parallel.md A1): cross-gantry RESERVATIONS for the parallel runner.

Invariant: at every instant every joint of gantry g (its two arms, rail, rotation) is inside res_g = the list of joint
states g may pass through until res_g is next rewritten. A motion of g may start only after its whole sweep has been
screened CLEAR (CrossGantryChecker, margin 50 mm, MARGIN = TOLAK) against EVERY state of res_other, and res_g has been
rewritten to cover that sweep -- both under one exclusive flock, so two screens never interleave. When a motion ends,
res_g shrinks to the measured end state. Traverses are not reserved here: the runner runs them EXCLUSIVELY (pose_to_g
fails on any arm motion, A0).

    dir/lock           flock target
    dir/res_g1.json    {"t": wall, "who": str, "states": [{joint: rad|m}, ...]}
"""
import fcntl
import json
import math
import os
import time

GANTRY_JOINTS = {g: [f't{g}_linear_joint', f't{g}_rotation_joint'] +
                 [f't{g}_a{a}_joint_{i}' for a in (1, 2) for i in range(1, 7)] for g in (1, 2)}


def own(g, state):
    """The gantry-g part of a full joint dict (strict: every name must be there)."""
    return {k: float(state[k]) for k in GANTRY_JOINTS[g]}


class Lock:
    def __init__(self, d):
        os.makedirs(d, exist_ok=True)
        self.d = d
        self.f = None

    def __enter__(self):
        self.f = open(os.path.join(self.d, 'lock'), 'a')
        fcntl.flock(self.f, fcntl.LOCK_EX)
        return self

    def __exit__(self, *a):
        fcntl.flock(self.f, fcntl.LOCK_UN)
        self.f.close()

    def read(self, g):
        p = os.path.join(self.d, f'res_g{g}.json')
        if not os.path.exists(p):
            return None                                     # never written = unknown = REFUSE (caller)
        return json.load(open(p))['states']

    def write(self, g, states, who):
        p = os.path.join(self.d, f'res_g{g}.json')
        tmp = p + '.tmp'
        json.dump(dict(t=time.time(), who=who, states=[own(g, s) for s in states]), open(tmp, 'w'))
        os.replace(tmp, p)


def thin(states, tol=1e-4):
    """Drop exact repeats (every joint within `tol` rad/m of the previous kept one): a held gantry repeats. Keeps first
    and last. NOT a coarsening -- the screened set stays the sampled sweep."""
    out = []
    for s in states:
        if out and all(abs(s[k] - out[-1][k]) <= tol for k in s):
            continue
        out.append(s)
    if states and out[-1] is not states[-1]:
        out.append(states[-1])
    return out


def screen(cross, names, pts, base, other_states, g_other):
    """Moving `names` along `pts` (everything else = base) vs EVERY other-gantry state. (verdict, d_min, pair, k_other).
    CLEAR only if every screen is CLEAR; MARGIN/COLLIDE = refusal (MARGIN = TOLAK)."""
    worst = ('CLEAR', math.inf, None, -1)
    for k, s in enumerate(thin([own(g_other, x) for x in other_states])):
        held = dict(base)
        held.update(s)
        v, d, pair, _ = cross.screen_trajectory(names, pts, held)
        if d < worst[1]:
            worst = (v, d, pair, k)
        if v != 'CLEAR':
            return v, d, pair, k
    return worst


# ------------------------------------------------------------- worker process (G54 B2: keep the runner's GIL free)
_CROSS = None


def _init_worker():
    global _CROSS
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        from interarm_collision import CrossGantryChecker
        _CROSS = CrossGantryChecker()


def screen_job(args):
    """screen() in a spawned worker (initializer _init_worker). args = (names, pts, base, other_states, g_other)."""
    v, d, pair, k = screen(_CROSS, *args)
    return v, float(d), pair, k
