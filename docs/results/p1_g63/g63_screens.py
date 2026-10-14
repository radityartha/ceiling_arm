"""G63: point the SHARED screens at the payload URDF (boards modelled), without editing shared files.

reach_dwell_probe / interarm_collision / env_collision read /tmp/reach_dwell_live.urdf. With a board taped to a
finger that model is wrong (mass + volume missing), so for G63 every screen must use the payload URDF instead:
  * probe._urdf_path              -> torque RNEA + GOAL-MISMATCH FK
  * interarm_collision.InterArmChecker / CrossGantryChecker (urdf=)   -> same-gantry mesh + cross-gantry hull
  * env_collision.EnvChecker (urdf=) + EXPECTED_GEOMS += board geoms  -> frozen env map
install() VERIFIES afterwards that each checker really contains the board geometry; any miss raises (fail loud).
"""
import functools
import os
import sys

REPO = '/home/user1/Documents/ceiling_arm'
sys.path.insert(0, os.path.join(REPO, 'scripts'))

import env_collision  # noqa: E402
import interarm_collision  # noqa: E402

from g63_common import ARM_PREFIX, BOARD_ARM  # noqa: E402

BOARD_GEOMS = sorted(f'{ARM_PREFIX[a]}g63_board_0' for a in BOARD_ARM.values())


def install(payload_urdf, probe=None, boards=('A', 'B')):
    payload_urdf = os.path.abspath(payload_urdf)
    if not os.path.exists(payload_urdf):
        raise FileNotFoundError(payload_urdf)
    want = sorted(f'{ARM_PREFIX[BOARD_ARM[b]]}g63_board_0' for b in boards)
    if getattr(interarm_collision, '_g63_installed', None) not in (None, payload_urdf):
        raise RuntimeError('g63_screens sudah dipasang dengan URDF lain')
    if getattr(interarm_collision, '_g63_installed', None) is None:
        interarm_collision.InterArmChecker = functools.partial(interarm_collision.InterArmChecker, urdf=payload_urdf)
        interarm_collision.CrossGantryChecker = functools.partial(interarm_collision.CrossGantryChecker,
                                                                  urdf=payload_urdf)
        env_collision.EXPECTED_GEOMS = sorted(env_collision.EXPECTED_GEOMS + want)
        env_collision.EnvChecker = functools.partial(env_collision.EnvChecker, urdf=payload_urdf)
        interarm_collision._g63_installed = payload_urdf
    if probe is not None:
        probe._urdf_path = lambda node, cache=None: payload_urdf
    return payload_urdf


def verify(boards=('A', 'B')):
    """Construct each checker the way the probe does and assert the board geometry is inside. Returns a report."""
    rep = {}
    for b in boards:
        arm = BOARD_ARM[b]
        g = 'gantry_1' if ARM_PREFIX[arm].startswith('t1') else 'gantry_2'
        name = f'{ARM_PREFIX[arm]}g63_board_0'
        same = interarm_collision.InterArmChecker(gantry=g)
        cross = interarm_collision.CrossGantryChecker()
        env = env_collision.EnvChecker()
        got = dict(interarm=name in same.names, cross=name in cross.names, env=name in env.names)
        if not all(got.values()):
            raise RuntimeError(f'papan {b}: geometri {name} TIDAK ada di penyaring {got} -- TOLAK')
        rep[b] = got
    return rep
