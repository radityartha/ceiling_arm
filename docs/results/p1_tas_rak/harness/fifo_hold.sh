#!/usr/bin/env bash
source $(dirname $0)/env.sh; [ -p $L/cli.fifo ] || mkfifo $L/cli.fifo
exec sleep infinity > $L/cli.fifo
