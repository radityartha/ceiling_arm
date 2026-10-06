"""G28-YAML step 1: 24 arm joints -- URDF (live cache) vs MoveIt joint_limits.yaml vs ros2_control command limits.
Effective MoveIt limit = yaml if has_position_limits else URDF. LOOSER = effective range exceeds URDF.

    python3 limits_table.py [--yaml PATH]
"""
import argparse
import re
import xml.etree.ElementTree as ET

import yaml

URDF = '/tmp/reach_dwell_live.urdf'
YAML = '/home/user1/Documents/ceiling_arm/ros2_ws/src/workcell_moveit_config/config/joint_limits.yaml'
RC = ('/home/user1/Documents/ceiling_arm/ros2_ws/src/ros2_kortex/kortex_description/arms/gen3_lite/6dof/'
      'urdf/kortex.ros2_control.xacro')
ARMS = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}

ap = argparse.ArgumentParser()
ap.add_argument('--yaml', default=YAML)
a = ap.parse_args()
lim = {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
       for j in ET.parse(URDF).getroot().iter('joint') if j.find('limit') is not None}
yl = yaml.safe_load(open(a.yaml))['joint_limits']
rc_raw = re.findall(r'joint name="\$\{prefix\}(joint_\d)">.*?name="min">([-\d.]+)<.*?name="max">([-\d.]+)<',
                    open(RC).read(), re.S)
rc = {j: (float(lo), float(hi)) for j, lo, hi in rc_raw}
print(f'{"sendi":16s} {"URDF":>14s} {"yaml":>14s} {"efektif MoveIt":>16s} {"ros2_control":>14s}  status')
n_loose = 0
for arm, p in ARMS.items():
    for i in range(1, 7):
        n = f'{p}joint_{i}'
        u = lim[n]
        y = yl.get(n, {})
        ys = (y['min_position'], y['max_position']) if y.get('has_position_limits') else None
        eff = ys or u
        loose = eff[0] < u[0] - 1e-9 or eff[1] > u[1] + 1e-9
        tight = eff[0] > u[0] + 1e-9 or eff[1] < u[1] - 1e-9
        n_loose += loose
        r = rc[f'joint_{i}']
        f = lambda t: f'[{t[0]:+.2f},{t[1]:+.2f}]'
        print(f'{n:16s} {f(u):>14s} {f(ys) if ys else "-":>14s} {f(eff):>16s} {f(r):>14s}  '
              f'{"LONGGAR" if loose else ("KETAT" if tight else "= URDF")}')
print(f'\nLONGGAR: {n_loose}/24')
