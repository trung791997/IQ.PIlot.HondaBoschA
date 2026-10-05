#!/usr/bin/env bash
# Why does card die? On the device:  bash /data/openpilot/tools/ct.sh
source "$(dirname "$(readlink -f "$0")")/diag_env.sh"
echo "== last card crash in the manager's terminal log (full traceback)"
LOG=$(for s in $(tmux list-sessions -F '#S' 2>/dev/null); do tmux capture-pane -J -pt "$s" -S -5000 2>/dev/null; done)
N=$(echo "$LOG" | grep -n "^Process card:" | tail -1 | cut -d: -f1)
if [ -n "$N" ]; then echo "$LOG" | tail -n +"$N" | head -40 | cut -c1-400
else echo "(no 'Process card:' crash in the log)"; fi
echo
if pgrep -f "selfdrive.car.card" >/dev/null; then echo "card is running now (pid $(pgrep -f selfdrive.car.card | head -1))"; exit 0; fi
if [ "$1" = "run" ]; then
  echo "== foreground run (20 s) as root. This WILL silence the radar and take control; car in Park."
  SUDO=""; [ "$(id -u)" != 0 ] && SUDO="sudo env PYTHONPATH=$PYTHONPATH PYTHONSAFEPATH=1 IQPILOT_SOURCE_ROOT=$IQPILOT_SOURCE_ROOT IQPILOT_PROPRIETARY_ROOT=$IQPILOT_PROPRIETARY_ROOT"
  timeout 20 $SUDO "$PY" -u -m iqpilot.selfdrive.car.card > /tmp/card_trace.log 2>&1
  echo "card exit code: $? (124 = still alive after 20 s)"
  grep -v "^\s*$" /tmp/card_trace.log | tail -25 | cut -c1-400
else
  echo "(add 'run' to also start card for 20 s: bash tools/ct.sh run)"
fi
