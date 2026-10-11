#!/bin/bash
# G55 A1.1-2 (resumable: finished seeds kept in the candidates JSON, tuples cached). Pool 15: NO mock planning meanwhile.
# Order = tomorrow's HW pairs first (s7, s10 R90 + R35), then s12, s11, then the rest. (K already passed: g55_run.log.)
cd "$(dirname "$0")"; export PYTHONUNBUFFERED=1
python3 -u g55_make.py run R90 7 10 && python3 -u g55_make.py run R35 7 10 && python3 -u g55_make.py run R90 12 11 \
  && python3 -u g55_make.py run R90 0 3 8 18 && python3 -u g55_make.py run R35 0 3 8 18
echo "JOB SELESAI rc $? $(date +%T)"
