#!/bin/bash
# G64-C fase 2a: (a) kisi halus T1, (b) tabel pose T1, K-REST per sumber (docs/p1_g64c.md C2)
cd "$(dirname "$0")"
set -e
python3 -u g64c_make.py fine
python3 -u g64c_make.py poses
python3 -u g64c_make.py krest g55R90 0 3 7 8 11 18
python3 -u g64c_make.py krest g55R35 10
python3 -u g64c_make.py krest g52R90 12
echo CHAIN1 SELESAI
