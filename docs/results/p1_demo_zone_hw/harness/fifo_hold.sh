#!/usr/bin/env bash
# Pemegang fifo (tulis-terbuka selamanya) agar target_cli tidak dapat EOF di antara 'g'. SIGINT/kill PID ini di akhir.
source $(dirname $0)/env.sh; [ -p $L/cli.fifo ] || mkfifo $L/cli.fifo
exec sleep infinity > $L/cli.fifo
