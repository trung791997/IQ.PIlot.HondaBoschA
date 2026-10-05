#!/usr/bin/env bash
# Why does card die? On the device, ignition on, car in Park:  bash /data/openpilot/tools/ct.sh
source "$(dirname "$(readlink -f "$0")")/diag_env.sh"
echo "== card lines in the manager's terminal log (the real crash, if it happened under the manager)"
for s in $(tmux list-sessions -F '#S' 2>/dev/null); do tmux capture-pane -pt "$s" -S -3000 2>/dev/null; done \
  | grep -nE "card|Traceback|Error" | tail -12
if pgrep -f "selfdrive.car.card" >/dev/null; then echo "card is running now (pid $(pgrep -f selfdrive.car.card | head -1))"; exit 0; fi
echo "== foreground run (20 s), as root like the manager's processes; this WILL silence the radar and take control"
SUDO=""; [ "$(id -u)" != 0 ] && SUDO="sudo -E env PATH=$PATH"
timeout 20 $SUDO "$PY" -u -m iqpilot.selfdrive.car.card > /tmp/card_trace.log 2>&1
echo "card exit code: $? (124 = still alive after 20 s)"
grep -v "^\s*$" /tmp/card_trace.log | tail -25
