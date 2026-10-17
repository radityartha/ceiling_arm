#!/bin/bash
# G64 task 0: diagnosis chain, resumable (g64_diag.jsonl append). setsid nohup ./g64_diag_chain.sh > g64_diag_chain.log 2>&1 < /dev/null &
cd "$(dirname "$0")"
export PYTHONUNBUFFERED=1
for s in TASRAK DEMOOFF G42 CAP; do
  echo "=== $s $(date +%T)"
  python3 g64_diag.py run --only $s || echo "!!! $s rc $?"
done
echo "=== selesai $(date +%T)"
python3 g64_diag.py report
