#!/bin/bash
# G64-C D670: solver 4-kombinasi pada 8 seed g55_selected (per sumber), biaya lipat SEMENTARA
cd "$(dirname "$0")"
python3 -u g64c_make.py combos g55R90 0 3 7 8 11 18
python3 -u g64c_make.py combos g55R35 10
python3 -u g64c_make.py combos g52R90 12
echo CHAIN2 SELESAI
