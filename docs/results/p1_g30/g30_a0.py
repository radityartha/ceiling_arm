"""G30 A0 -- INSTRUMENT (offline, before section A is locked): sweep_rot (true hull, RotCrossChecker) for the
4 legs 0 -> +10 -> 0 -> -10 -> 0 of each gantry, arms REST, rails 0/0 (G26 end state), other gantry held at 0.
Also +-0.5 deg slack around each leg end (bridge debounce / deadband)."""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../p1_g29'))
import g29_rot_screen as R  # noqa: E402

chk = R.RotCrossChecker()
out = {}
for g in (1, 2):
    for a, b in ((0, 10), (10, 0), (0, -10), (-10, 0), (-10.5, 10.5)):
        st = R.rest_state(0.0, 0.0, 0.0, 0.0)
        sw = R.sweep_rot(chk, g, (0.0, math.radians(a)), (0.0, math.radians(b)), st)
        key = f'g{g} {a:+.1f}->{b:+.1f}'
        out[key] = dict(verdict=sw['verdict'], d_mm=sw['d'] * 1000, pair=sw['pair'],
                        at_deg=math.degrees(sw['at'][1]), d_arm_mm=sw['d_arm'] * 1000,
                        d_ss_mm=sw['d_ss'] * 1000, n=sw['n'])
        o = out[key]
        print(f"{key}: {o['verdict']} min {o['d_mm']:.1f} mm {o['pair']} @ {o['at_deg']:+.1f} deg; "
              f"lengan {o['d_arm_mm']:.1f}, SS {o['d_ss_mm']:.1f}, n {o['n']}", flush=True)
# rest-arm tip horizontal radius -> tip arc length for 10 deg
import pinocchio as pin  # noqa: E402
q = chk.q_from(R.rest_state(0.0, 0.0, 0.0, 0.0))
pin.framesForwardKinematics(chk.model, chk.data, q)
for p in R.ARM:
    t = chk.data.oMf[chk.model.getFrameId(f'{p}tool_frame')].translation
    g = 1 if p.startswith('t1') else 2
    yg = 0.36 if g == 1 else -0.36
    rh = math.hypot(t[0] - 0.0, t[1] - yg)
    out[f'{p}rh_mm'] = rh * 1000
    print(f'{p}tool_frame REST r_h {rh*1000:.1f} mm -> busur 10 deg {rh*math.radians(10)*1000:.1f} mm')
json.dump(out, open(os.path.join(HERE, 'g30_a0.json'), 'w'), indent=1, default=str)
