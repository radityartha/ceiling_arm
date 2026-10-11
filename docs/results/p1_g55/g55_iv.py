"""G55 A1.4: (iv') + (iv'') of the selected schedules (variant R90sel; R35sel is the same schedule for every seed, B)."""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g52'))
import g52_make as G52  # noqa: E402
P = os.path.join(HERE, 'g55_selected.json')
if sys.argv[1] == 'iv':
    G52.K.VAR = 'R90sel'
    G52.K.iv(P)
else:
    G52.G41.chain('R90sel', P)
