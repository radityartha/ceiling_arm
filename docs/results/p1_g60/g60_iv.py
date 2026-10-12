"""G60 A2.4: (iv') + (iv'') of the new G60b schedules (g60_selected.json, variant G60: s3 s12 s18)."""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g52'))
import g52_make as G52  # noqa: E402
P = os.path.join(HERE, 'g60_selected.json')
if sys.argv[1] == 'iv':
    G52.K.VAR = 'G60'
    G52.K.iv(P)
else:
    G52.G41.chain('G60', P)
