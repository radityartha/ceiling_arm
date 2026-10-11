#!/bin/bash
# G52 offline job (docs/p1_g52_r90.md A2 1-3). Resumable: finished seeds are kept in the candidates JSON.
cd "$(dirname "$0")"
export PYTHONUNBUFFERED=1
python3 -u g52_make.py k && python3 -u g52_make.py k90 && python3 -u g52_make.py run
echo "JOB SELESAI rc $? $(date +%T)"
