#!/usr/bin/env bash
# Run card in the foreground for 20 s with the manager's interpreter/env and show how it dies.
# Run on the device with ignition on: bash /data/openpilot/tools/card_trace.sh
PID=$(pgrep -f "system/manager/manager.py" | head -1)
if [ -z "$PID" ]; then echo "manager not running"; exit 1; fi
PY=$(readlink /proc/$PID/exe)
while IFS= read -r -d '' kv; do
  case "$kv" in PYTHONPATH=*|IQPILOT_*=*|AGNOS_VERSION=*|QCOM_*=*) export "$kv";; esac
done < /proc/$PID/environ
cd /data/openpilot || exit 1
if pgrep -f "selfdrive.car.card" >/dev/null; then echo "card is already running (pid $(pgrep -f selfdrive.car.card))"; exit 0; fi
timeout 20 "$PY" -u -m iqpilot.selfdrive.car.card > /tmp/card_trace.log 2>&1
echo "card exit code: $? (124 = still alive after 20 s)"
grep -v "^\s*$" /tmp/card_trace.log | tail -25
