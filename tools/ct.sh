#!/usr/bin/env bash
# Run card in the foreground for 20 s and show how it dies. On the device, ignition on, car in Park:
#   bash /data/openpilot/tools/ct.sh
source "$(dirname "$(readlink -f "$0")")/diag_env.sh"
if pgrep -f "selfdrive.car.card" >/dev/null; then
  echo "card is already running (pid $(pgrep -f selfdrive.car.card | head -1)), nothing to trace"
  exit 0
fi
echo "python: $PY"
timeout 20 "$PY" -u -m iqpilot.selfdrive.car.card > /tmp/card_trace.log 2>&1
echo "card exit code: $? (124 = still alive after 20 s)"
grep -v "^\s*$" /tmp/card_trace.log | tail -25
