#!/usr/bin/env python3
"""Fast surrogate of the gantry workcell for task-allocation / coalition policies.

Why a surrogate: training or even A/B-ing an allocation policy against MoveIt +
Isaac costs seconds to minutes per pick. This env answers the same question --
"which arm takes which handle, with which gantry pose, and what does it cost?" --
from the pre-computed FK maps in data/maps, in microseconds and with no ROS.

IT DELIBERATELY EXPOSES THE EXECUTOR'S INTERFACE. An action is

    (arm, item, handle, gantry_pose)

which is exactly what gantry_reach_executor._do_pick resolves, and the cost is
the SAME energy J the executor scores candidates with -- same weights, same
references (w_gantry_lin 2.0, w_gantry_rot 12.0, w_arm 20.0, w_dist 3.0,
w_hold 1.0, w_manip 1.0). So a policy trained here emits actions the real
executor can run, and J here is comparable to J there. Two deliberate
deviations, both documented at their use sites: the gantry rotation difference
is WRAPPED to +-pi (the executor never wraps, which would price a
-179 -> +179 deg move as 358 deg), and gantry travel is charged ONCE PER GANTRY
MOVE rather than once per arm, so a 4-arm coalition sharing one gantry move is
not billed for it four times. That second rule is applied identically to every
policy, so it cannot favour coalitions.

THE MECHANISM THAT MAKES COALITIONS NECESSARY IS PAYLOAD, NOT A RULE. Each
Gen3 Lite carries 0.5 kg, so an item of mass m needs ceil(m / 0.5) arms. A
1.8 kg block needs 4. A single-arm allocator does not "decline" to form a
coalition -- it physically cannot lift the block, and burns energy failing. That
is why greedy-energy fails here, and it is a property of the hardware rather
than of the baseline's action set.

Reachability comes from the same gantry-local oracle as comanip_map.py, so
feasibility here and in Fig. 1 cannot disagree.

    python3 scripts/comanip_env.py            # self-check + scene summary
"""

from __future__ import annotations

import os

import numpy as np
from scipy.spatial import cKDTree

from comanip_map import (ARM_GANTRY, GANTRY_ARMS, GANTRY_BASE, MOUNT_LOCAL,
                         Z_OFFSET, arm_clearance, handle_corners, to_local)

ARMS = ['arm1', 'arm2', 'arm3', 'arm4']
PAYLOAD_PER_ARM = 0.5              # kg, Gen3 Lite spec
RAIL_LIMITS = (0.0, 2.0)           # URDF t{N}_linear_joint

# J weights/references, copied from gantry_reach_executor's declared defaults
# (and identical in reach_fusion). Keep in sync -- this IS the reward.
W = dict(gantry_lin=2.0, gantry_rot=12.0, arm=20.0, dist=3.0, hold=1.0, manip=1.0)
REF = dict(gantry_lin=0.95, gantry_rot=0.70, arm=6.0, dist=1.36, hold=2.90,
           manip=0.145)


def wrap(a):
    """Angle difference wrapped to [-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class ArmOracle:
    """One arm's reachability + energy, from its FK dataset and GNG stats.

    Reach is tested in the GANTRY-LOCAL frame, where the arm's reachable set is
    independent of where its gantry sits -- the same construction comanip_map.py
    uses, so the two cannot disagree about what is reachable.
    """

    def __init__(self, maps_dir, name, reach_radius=0.05):
        self.name, self.gantry = name, ARM_GANTRY[name]
        d = np.load(os.path.join(maps_dir, f'{name}_dataset.npz'))
        p = d['pose'][:, :3].astype(np.float64)
        self.q = d['q'].astype(np.float64)          # (N, 8) = [rail, theta, j1..j6]
        self.manip = d['manip'].astype(np.float64)
        self.reach_radius = reach_radius

        base = GANTRY_BASE[self.gantry]
        v = p - base - Z_OFFSET
        v[:, 0] -= self.q[:, 0]                     # undo the rail offset
        ang = -(np.pi / 2.0 + self.q[:, 1])         # undo the rotation joint
        c, s = np.cos(ang), np.sin(ang)
        self.local = np.column_stack([c * v[:, 0] - s * v[:, 1],
                                      s * v[:, 0] + c * v[:, 1], v[:, 2]])
        self.tree = cKDTree(self.local)

        # `hold` (||gravity torque||) exists only per GNG node, not per FK
        # sample. The executor reads hold[i] for the NODE nearest the target in
        # task space, so look it up the same way to stay faithful to its J.
        self.hold_tree, self.hold = None, None
        mp = os.path.join(maps_dir, f'{name}_model.npz')
        sp = os.path.join(maps_dir, f'{name}_model_stats.npz')
        if os.path.exists(mp) and os.path.exists(sp):
            m, st = np.load(mp), np.load(sp)
            key = 'W' if 'W' in m.files else m.files[0]
            nodes = m[key][:, :3].astype(np.float64)
            if 'hold' in st.files and len(st['hold']) == len(nodes):
                self.hold_tree, self.hold = cKDTree(nodes), st['hold']

    def hold_at(self, world_pt):
        if self.hold_tree is None:
            return 0.0
        return float(self.hold[self.hold_tree.query(np.asarray(world_pt))[1]])

    def sweep(self, world_pt, poses):
        """(dist, sample_idx) of the nearest FK sample for EVERY gantry pose.

        `poses` is (P, 2) of (rail, theta); the point is transformed into each
        pose's local frame and queried in one batch, so scoring a whole gantry
        sweep costs one KD-tree call.
        """
        pt = np.asarray(world_pt, dtype=np.float64).reshape(1, 3)
        base = GANTRY_BASE[self.gantry]
        loc = np.vstack([to_local(pt, base, r, t) for r, t in poses])
        return self.tree.query(loc)

    def arm_cost(self, idx, dist, world_pt, cur_arm_q):
        """Non-gantry part of J: task gap + gravity hold - manipulability + arm travel."""
        j = (W['dist'] * dist / REF['dist']
             + W['hold'] * self.hold_at(world_pt) / REF['hold']
             - W['manip'] * self.manip[idx] / REF['manip'])
        if cur_arm_q is not None:
            j += W['arm'] * np.abs(self.q[idx, 2:] - cur_arm_q).sum() / REF['arm']
        return float(j)


def gantry_cost(rail, theta, cur_rail, cur_theta):
    """Gantry travel, charged ONCE per gantry move (see module docstring)."""
    return float(W['gantry_lin'] * abs(rail - cur_rail) / REF['gantry_lin']
                 + W['gantry_rot'] * abs(wrap(theta - cur_theta)) / REF['gantry_rot'])


class Item:
    """A transport task. `n_arms` follows from mass, not from a scenario flag."""

    def __init__(self, name, xyz, mass, size=0.25, yaw=0.0):
        self.name, self.xyz, self.mass = name, np.asarray(xyz, float), float(mass)
        self.size, self.yaw = size, yaw
        self.n_arms = max(1, int(np.ceil(mass / PAYLOAD_PER_ARM)))

    def handles(self):
        """World handle positions: the centroid for a 1-arm item, the 4 top-face
        corners for a co-manipulated one."""
        if self.n_arms == 1:
            return self.xyz.reshape(1, 3)
        return handle_corners(self.xyz.reshape(1, 3), self.yaw, self.size)[:, 0, :]

    def __repr__(self):
        return (f'{self.name}(m={self.mass:.2f}kg, {self.n_arms}-arm, '
                f'xyz={np.round(self.xyz, 2)})')


class WorkcellEnv:
    """Surrogate workcell. Gymnasium-shaped reset/step; no gym dependency yet.

    The observation/reward contract is the one a PettingZoo ParallelEnv wrapper
    needs (obs, reward, terminated, truncated, info), so wiring MAPPO later is a
    wrapper rather than a rewrite. It is kept dependency-free because the greedy
    vs auction comparison needs no learning at all.
    """

    def __init__(self, maps_dir='data/maps', reach_radius=0.05, rail_step=0.2,
                 rot_step=0.4, min_link_clearance=0.10, max_rounds=12):
        self.oracles = {a: ArmOracle(maps_dir, a, reach_radius) for a in ARMS}
        rails = np.arange(RAIL_LIMITS[0], RAIL_LIMITS[1] + 1e-9, rail_step)
        rots = np.arange(-np.pi, np.pi, rot_step)
        self.poses = np.array([(r, t) for r in rails for t in rots])
        self.reach_radius = reach_radius
        self.min_link_clearance = min_link_clearance
        self.max_rounds = max_rounds
        self.items = []

    # ---- scene ------------------------------------------------------------
    def _reachable_mask(self, pts, arm):
        """Which of `pts` this arm can reach at SOME gantry pose."""
        ok = np.zeros(len(pts), bool)
        for i, p in enumerate(pts):
            d, _ = self.oracles[arm].sweep(p, self.poses)
            ok[i] = bool((d < self.reach_radius).any())
        return ok

    def sample_scene(self, seed=0, n_light=5, light_mass=0.3, block_mass=1.8,
                     block_size=0.25, z=1.30, xlim=(0.2, 2.2), ylim=(-0.8, 0.8),
                     n_try=400):
        """5 light 1-arm items + 1 heavy 4-arm block, all drawn from the VERIFIED
        feasible set rather than hand-placed, so a policy can never fail for the
        trivial reason that a task was unreachable to begin with."""
        rng = np.random.default_rng(seed)
        cand = np.column_stack([rng.uniform(*xlim, n_try),
                                rng.uniform(*ylim, n_try),
                                np.full(n_try, z)])

        # light items: reachable by at least one arm
        any_arm = np.zeros(len(cand), bool)
        for a in ARMS:
            any_arm |= self._reachable_mask(cand, a)
        pool = cand[any_arm]
        if len(pool) < n_light:
            raise RuntimeError(f'only {len(pool)} reachable light candidates')
        light_xyz = pool[rng.choice(len(pool), n_light, replace=False)]

        # block: needs a centre where a full 4-arm coalition is feasible
        block_xyz = None
        for p in cand[rng.permutation(len(cand))]:
            it = Item('block', p, block_mass, block_size)
            if self.coalition_plan(it, self.zero_state()) is not None:
                block_xyz = p
                break
        if block_xyz is None:
            raise RuntimeError('no 4-arm-feasible block placement found; widen '
                               'the sampling box or lower --min-link-clearance')

        self.items = [Item(f'light{i}', p, light_mass) for i, p in enumerate(light_xyz)]
        self.items.append(Item('block', block_xyz, block_mass, block_size))
        return self.items

    # ---- state ------------------------------------------------------------
    def zero_state(self):
        """Home state: gantries mid-rail at the URDF default rotation, arms tucked
        at the pose ros2_bridge_gui.py starts them in (joint_2=joint_3=2.60)."""
        return dict(rail=np.array([1.0, 1.0]), theta=np.array([0.0, 0.0]),
                    q=np.array([[0.0, 2.60, 2.60, 0.0, 0.0, 0.0]] * 4, float),
                    done=np.zeros(len(self.items), bool), round=0, energy=0.0,
                    wasted=0.0, failed=0, coalition_peak=0)

    def reset(self, seed=0, **kw):
        self.sample_scene(seed=seed, **kw)
        self.state = self.zero_state()
        return self._obs(), {}

    def _obs(self):
        s = self.state
        return dict(rail=s['rail'].copy(), theta=s['theta'].copy(), q=s['q'].copy(),
                    done=s['done'].copy(),
                    items=[(it.name, it.xyz, it.n_arms) for it in self.items])

    # ---- costing ----------------------------------------------------------
    def single_plan(self, item, arm, state, handle=None):
        """Cheapest (handle, gantry pose, J) for ONE arm acting alone on `item`.

        `handle=None` searches all of the item's handles, since an arm offered a
        4-corner block would naturally go for whichever corner is cheapest for
        it -- that is what makes greedy ATTRACTED to the block rather than
        indifferent to it. Returns None when no gantry pose puts any handle in
        reach. J includes the gantry move, so it is directly comparable to a
        coalition's per-gantry total.
        """
        o = self.oracles[arm]
        g = o.gantry
        H = item.handles()
        hs = range(len(H)) if handle is None else [handle]
        ai = ARMS.index(arm)
        best = None
        for h in hs:
            pt = H[h]
            d, idx = o.sweep(pt, self.poses)
            for k in np.flatnonzero(d < self.reach_radius):
                rail, theta = self.poses[k]
                j = (o.arm_cost(idx[k], d[k], pt, state['q'][ai])
                     + gantry_cost(rail, theta, state['rail'][g], state['theta'][g]))
                if best is None or j < best[0]:
                    best = (j, float(rail), float(theta), int(idx[k]), int(h))
        if best is None:
            return None
        return dict(J=best[0], arm=arm, gantry=g, rail=best[1], theta=best[2],
                    idx=best[3], handle=best[4], item=item)

    def coalition_plan(self, item, state, arms=None):
        """Cheapest joint plan for a 4-handle item: partition the handles between
        the gantries, assign each gantry's two arms, and pick each gantry's pose.

        Both gantries must succeed, and each gantry's single pose has to serve
        BOTH its arms at once -- the coupling that makes this problem hard and
        that a per-arm map cannot express. Returns None if no assignment works.
        """
        arms = arms or ARMS
        if len(arms) < 4 or any(a not in arms for a in ARMS):
            return None                     # a full coalition needs all four
        H = item.handles()                                  # (4, 3)
        partitions = [((0, 1), (2, 3)), ((1, 2), (3, 0))]   # opposite top-face edges
        best = None
        for pa, pb in partitions:
            for g_pairs in ((pa, pb), (pb, pa)):            # which gantry takes which
                total, per_gantry = 0.0, {}
                for g in (0, 1):
                    sub = self._gantry_pair_plan(item, g, H, g_pairs[g], state)
                    if sub is None:
                        total = None
                        break
                    total += sub['J']
                    per_gantry[g] = sub
                if total is not None and (best is None or total < best[0]):
                    best = (total, per_gantry)
        if best is None:
            return None
        return dict(J=best[0], item=item, gantries=best[1], kind='coalition')

    def _gantry_pair_plan(self, item, g, H, pair, state):
        """Cheapest single pose for gantry `g` serving both its arms on `pair`."""
        a1, a2 = GANTRY_ARMS[g]
        best = None
        for order in ((a1, a2), (a2, a1)):        # which arm takes which handle
            ha, hb = H[pair[0]], H[pair[1]]
            oa, ob = self.oracles[order[0]], self.oracles[order[1]]
            da, ia = oa.sweep(ha, self.poses)
            db, ib = ob.sweep(hb, self.poses)
            ok = np.flatnonzero((da < self.reach_radius) & (db < self.reach_radius))
            for k in ok:
                rail, theta = self.poses[k]
                if self.min_link_clearance > 0.0:
                    base = GANTRY_BASE[g]
                    la = to_local(ha.reshape(1, 3), base, rail, theta)
                    lb = to_local(hb.reshape(1, 3), base, rail, theta)
                    if arm_clearance(MOUNT_LOCAL[order[0]], la,
                                     MOUNT_LOCAL[order[1]], lb)[0] < self.min_link_clearance:
                        continue
                j = (oa.arm_cost(ia[k], da[k], ha, state['q'][ARMS.index(order[0])])
                     + ob.arm_cost(ib[k], db[k], hb, state['q'][ARMS.index(order[1])])
                     + gantry_cost(rail, theta, state['rail'][g], state['theta'][g]))
                if best is None or j < best[0]:
                    best = (j, float(rail), float(theta), order,
                            (int(ia[k]), int(ib[k])))
        if best is None:
            return None
        return dict(J=best[0], rail=best[1], theta=best[2], arms=best[3],
                    idx=best[4], handles=(pair[0], pair[1]))

    # ---- transition -------------------------------------------------------
    def apply(self, plan):
        """Commit a plan: charge its J, move the arms/gantries it used, mark the
        item transported. A `None` plan is a FAILED attempt -- it still costs the
        travel that was spent finding out, which is how a single-arm allocator
        pays for trying to lift a 1.8 kg block."""
        s = self.state
        if plan is None:
            return 0.0
        i = self.items.index(plan['item'])
        if plan.get('kind') == 'coalition':
            for g, sub in plan['gantries'].items():
                s['rail'][g], s['theta'][g] = sub['rail'], sub['theta']
                for a, ix in zip(sub['arms'], sub['idx']):
                    s['q'][ARMS.index(a)] = self.oracles[a].q[ix, 2:]
            s['coalition_peak'] = max(s['coalition_peak'], 4)
        else:
            g = plan['gantry']
            s['rail'][g], s['theta'][g] = plan['rail'], plan['theta']
            s['q'][ARMS.index(plan['arm'])] = self.oracles[plan['arm']].q[plan['idx'], 2:]
        s['done'][i] = True
        s['energy'] += plan['J']
        s['round'] += 1
        return plan['J']

    def charge_failed(self, plan, arms_committed=0):
        """Charge a failed attempt: the motion happened, the lift did not."""
        s = self.state
        if plan is not None:
            s['energy'] += plan['J']
            s['wasted'] += plan['J']
            if plan.get('kind') != 'coalition':
                g = plan['gantry']
                s['rail'][g], s['theta'][g] = plan['rail'], plan['theta']
                s['q'][ARMS.index(plan['arm'])] = \
                    self.oracles[plan['arm']].q[plan['idx'], 2:]
        s['failed'] += 1
        s['round'] += 1
        s['coalition_peak'] = max(s['coalition_peak'], arms_committed)

    def pending(self):
        return [it for it, d in zip(self.items, self.state['done']) if not d]

    def truncated(self):
        return self.state['round'] >= self.max_rounds


def _self_check():
    env = WorkcellEnv()
    items = env.sample_scene(seed=0)
    print('scene:')
    for it in items:
        print('  ', it)
    st = env.zero_state()
    print('\nsingle-arm plans for light0 (J, gantry pose):')
    for a in ARMS:
        pl = env.single_plan(items[0], a, st)
        print(f'   {a}: ' + (f"J={pl['J']:7.3f} rail={pl['rail']:.2f} "
                             f"theta={np.degrees(pl['theta']):+7.1f}deg"
                             if pl else 'unreachable'))
    blk = items[-1]
    cp = env.coalition_plan(blk, st)
    print(f'\nblock needs {blk.n_arms} arms; coalition plan: '
          + (f"J={cp['J']:.3f}" if cp else 'INFEASIBLE'))
    if cp:
        for g, sub in sorted(cp['gantries'].items()):
            print(f"   gantry_{g + 1}: arms={sub['arms']} handles={sub['handles']} "
                  f"rail={sub['rail']:.2f} theta={np.degrees(sub['theta']):+7.1f}deg "
                  f"J={sub['J']:.3f}")
    print('\nsingle-arm attempt on the block is the payload failure the poster '
          'turns on:')
    for a in ARMS[:2]:
        pl = env.single_plan(blk, a, st)
        print(f"   {a} CAN reach a handle (J={pl['J']:.3f}) but 1 arm x 0.5 kg "
              f"< {blk.mass} kg -> lift fails" if pl else f'   {a}: unreachable')


if __name__ == '__main__':
    _self_check()
