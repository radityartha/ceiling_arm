#!/usr/bin/env bash
# G60 A2.2: solver job (Pool 15). NOT together with mock plan-only / smoke. Resumable (finished seeds kept, caches append).
cd /home/user1/Documents/ceiling_arm/docs/results/p1_g60
F="UserWarning\|warnings.warn\|buildGeomFromUrdf\|self.geom = \|V = np.array"
for RS in R90 R35; do
  echo "== $RS mulai $(date +%T)"
  python3 -u g60_make.py run $RS 7 10 0 3 8 18 11 12 2>&1 | grep --line-buffered -v "$F"
done
echo "== SEMUA SELESAI $(date +%T)"
