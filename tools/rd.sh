#!/usr/bin/env bash
# One-screen relay-malfunction diagnostic. Run on the device: bash /data/openpilot/tools/relay_diag.sh
# Borrows the running manager's interpreter and PYTHONPATH so it sees exactly what the processes see.
PID=$(pgrep -f "system/manager/manager.py" | head -1)
if [ -n "$PID" ]; then
  PY=$(readlink /proc/$PID/exe)
  export $(tr '\0' '\n' < /proc/$PID/environ | grep -E '^PYTHONPATH=')
else
  PY=/usr/local/venv/bin/python3
  export PYTHONPATH=/data/openpilot/artifacts/package_runtime:/data/openpilot
fi
cd /data/openpilot || exit 1
echo "OS: $(cat /VERSION)  git: $(git log --oneline -1 | cut -c1-40)"
exec "$PY" - <<'EOF'
import time, glob, os, re
from collections import Counter
from iqpilot.cereal import car, log
import iqpilot.cereal.messaging as messaging
from iqpilot.common.params import Params

p = Params()
try:
  alpha = p.get_bool("AlphaLongitudinalEnabled")
except Exception as e:
  alpha = f"? {e}"
print(f"AlphaLongitudinalEnabled={alpha}")
cp_bytes = p.get("CarParams")
if cp_bytes:
  with car.CarParams.from_bytes(cp_bytes) as CP:
    sc = [(str(s.safetyModel), s.safetyParam) for s in CP.safetyConfigs]
    print(f"CP: {CP.carFingerprint} opLong={CP.openpilotLongitudinalControl} pcmCruise={CP.pcmCruise} "
          f"radarUnavail={CP.radarUnavailable} flags={CP.flags} safety={sc}")
else:
  print("CP: none (car not recognized / not onroad yet)")

sm = messaging.SubMaster(["pandaStates", "selfdriveState", "onroadEvents", "managerState"])
can = messaging.sub_sock("can", conflate=False)
WATCH = {0xE4: "STEER", 0xE5: "SUPP", 0x1DF: "ACC_CTRL", 0x1EF: "ACC_CTRL2", 0x1FA: "BRAKE",
         0x30C: "ACC_HUD", 0x33D: "LKAS_HUD", 0x39F: "RADAR_HUD", 0x18DAB0F1: "UDS->radar", 0x18DAF1B0: "radar->UDS"}
cnt = Counter()
t0 = time.monotonic()
while time.monotonic() - t0 < 3.0:
  sm.update(50)
  for m in messaging.drain_sock(can):
    for c in m.can:
      if c.address in WATCH:
        cnt[(c.address, c.src)] += 1

for ps in sm["pandaStates"]:
  print(f"panda: ign={ps.ignitionLine or ps.ignitionCan} safety={ps.safetyModel}/{ps.safetyParam} "
        f"allowed={ps.controlsAllowed} harness={ps.harnessStatus} rxInv={ps.safetyRxInvalid} "
        f"txBlk={ps.safetyTxBlocked}")
  print(f"  faults={[str(f) for f in ps.faults]}")
print("CAN 3s (src: 0-2 rx from car, 128+ = openpilot tx):")
for addr, name in WATCH.items():
  row = {s: n for (a, s), n in cnt.items() if a == addr}
  if row:
    print(f"  {addr:#x} {name:10s} " + " ".join(f"b{s}:{n}" for s, n in sorted(row.items())))
ss = sm["selfdriveState"]
print(f"alert: {ss.alertText1!r} / {ss.alertText2!r}  state={ss.state}")
print("events:", [str(e.name) for e in sm["onroadEvents"]][:8])
dead = [pr.name for pr in sm["managerState"].processes if pr.shouldBeRunning and not pr.running]
print("should-run but dead:", dead)

# last exception line from each recent process log
hits = []
for f in sorted(glob.glob("/data/log/*"), key=os.path.getmtime)[-40:]:
  try:
    txt = open(f, errors="ignore").read()[-200000:]
  except Exception:
    continue
  for m in re.finditer(r"\b(\w+(?:Error|Exception))\b[:\\]+ ?([^\"\\]{0,90})", txt):
    hits.append(f"{m.group(1)}: {m.group(2)}")
print("recent exceptions:", list(dict.fromkeys(hits))[-5:] or "none")
EOF
