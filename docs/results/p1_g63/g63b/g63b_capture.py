#!/usr/bin/env python3
"""G63b tangkap / gabung: g63_capture.py (G63; hanya ditambah kait STILL_EXEMPT_PP default kosong = perilaku G63 sama)

Tambalan (hanya di proses ini):
  * BOARD_ARM: muka A dan B -> lengan fase (--arm arm_2 | arm_3); link = mount (gripper_base_link);
  * mount muka B = muka A @ T_12 (dari poses_g63b_*.json 'g63b'.T_12 atau mount card_t + square_m);
  * detektor: g63b_detect.detect (papan polos, penjaga terdaftar) menggantikan g63_detect.detect;
    muka yang membelakangi kamera ditolak dari prior (tanpa pencarian).
Aturan diam: sendi 1-3, 5-6 std < 2e-4 rad (tetap); j4 peak-to-peak <= j4_limit.json (§A6.1 operator 2026-10-15: gerak papan
<= 1.5 mm) lewat kait g63_capture.STILL_EXEMPT_PP; drift selama tangkap dinilai tanpa j4.

    g63b_capture.py capture --arm arm_2 --session S --pose-id A-R0 --board A B --role R0 --stage P0 --mount mount_g63b.json [--save-frames]
    g63b_capture.py capture --arm finger ... --mount mount_finger.json   (cadangan JARI, susunan G63, papan polos)
    g63b_capture.py merge --arm arm_2 --session S --square-mm A=42.0 B=42.0 --out ds_arm_2.json --poses poses_g63b_v3.json
merge menamai ulang papan per fase: A -> A2, B -> B2 (arm_2) / A3, B3 (arm_3), sehingga dataset dua fase bisa digabung
(T_lb rakitan berbeda sesudah dipindah). Solver: g63b_solve.py (g63_solve + nama papan per fase).
"""
import json
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
import g63_capture  # noqa: E402
import g63_detect  # noqa: E402
import g63b_detect  # noqa: E402
from g63_common import T_from_xyzrpy, xyzrpy_from_T  # noqa: E402


def two_face_mount(path):
    """mount_g63b.json (assembly = face A frame) -> temp mount with A and B face entries on the same link."""
    m = json.load(open(path))
    a = m['A']
    T12 = np.diag([-1.0, 1.0, -1.0, 1.0])
    T12[:3, 3] = [4 * float(a['square_m']), 0.0, float(a['card_t'])]
    TA = T_from_xyzrpy(*a['xyz'], *a['rpy'])
    xb = xyzrpy_from_T(TA @ T12)
    out = {'A': dict(a), 'B': dict(a, xyz=[float(v) for v in xb[:3]], rpy=[float(v) for v in xb[3:]])}
    f = tempfile.NamedTemporaryFile('w', suffix='_g63b_mount.json', delete=False)
    json.dump(out, f)
    f.close()
    return f.name, out


def main():
    if len(sys.argv) < 3 or '--arm' not in sys.argv:
        sys.exit('pakai: g63b_capture.py {capture|merge} --arm arm_2|arm_3 ...')
    i = sys.argv.index('--arm')
    arm = sys.argv[i + 1]
    if arm not in ('arm_2', 'arm_3', 'finger'):
        sys.exit('--arm harus arm_2, arm_3, atau finger (cadangan jari: A di arm_2, B di arm_3, satu muka)')
    del sys.argv[i:i + 2]
    if arm != 'finger':
        g63_capture.BOARD_ARM = {'A': arm, 'B': arm}
    # §A6.1 (operator 2026-10-15): j4 judged by peak-to-peak <= limit (board motion <= 1.5 mm), see g63b_j4_limit.py
    lim = json.load(open(os.path.join(HERE, 'j4_limit.json')))['j4_pp_max_rad']
    g63_capture.STILL_EXEMPT_PP = {'joint_4': float(lim)}
    cmd = sys.argv[1]
    if cmd == 'capture':
        j = sys.argv.index('--mount')
        if arm == 'finger':
            mt = {k: v for k, v in json.load(open(sys.argv[j + 1])).items() if not k.startswith('_')}
        else:
            tmp, mt = two_face_mount(sys.argv[j + 1])
            sys.argv[j + 1] = tmp
        sq = {k: float(v['square_m']) for k, v in mt.items()}
        g63_detect.detect = lambda gray, priors=None: g63b_detect.detect(gray, priors, sq)
        return g63_capture.main()
    if cmd == 'merge':
        k = sys.argv.index('--out')
        out = sys.argv[k + 1]
        rc = g63_capture.main()
        ds = json.load(open(out))
        ph = arm[-1] if arm != 'finger' else 'J'
        ren = {'A': f'A{ph}', 'B': f'B{ph}'}
        ds['square_m'] = {ren.get(b, b): v for b, v in ds['square_m'].items()}
        for p in ds['poses']:
            p['board'] = ren[p['board']]
            p['id'] = p['id'].split(':')[0] + ':' + p['board']
        ds['g63b'] = dict(arm=arm, renamed=ren)
        json.dump(ds, open(out, 'w'), indent=1)
        print(f'g63b: papan dinamai ulang {ren} -> {out}')
        return rc
    sys.exit(f'perintah tidak dikenal: {cmd}')


if __name__ == '__main__':
    sys.exit(main())
