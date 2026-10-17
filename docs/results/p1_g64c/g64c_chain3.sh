#!/bin/bash
# G64-C fase 4 (iv): solve ulang C3 + D670 dengan biaya lipat/buka TERUKUR mock 77 (A0.9).
#   44.83 = median terbesar (fold g2, event run_g54, k=3) -> utama; 44.39 = median terkecil (unfold g1) -> sensitivitas
cd "$(dirname "$0")"
for C in 44.83 44.39; do
  export G64C_C_FOLD=$C G64C_OUT_SUFFIX=_cfold$C
  echo "== c_fold = c_unfold = $C"
  G64C_ENV=1 python3 -u g64c_demo.py ../p1_demo_sys/tasks_synth.json run_c3_cfold$C
  python3 -u g64c_make.py combos g55R90 0 3 7 8 11 18
  python3 -u g64c_make.py combos g55R35 10
  python3 -u g64c_make.py combos g52R90 12
done
echo CHAIN3 SELESAI
