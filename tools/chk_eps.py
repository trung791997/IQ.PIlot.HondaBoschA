import sys
from iqpilot.common.params import Params
from iqpilot.cereal import car, custom
import iqpilot.cereal.messaging as messaging
from iqpilot.selfdrive.controls.lib.latcontrol_honda_eps import use_honda_eps_controller
from iqdbc.lvbs.car.honda.iq_values import HondaFlagsIQ

p = Params()
cp_bytes = p.get("CarParams")
iq_bytes = p.get("IQCarParams")

if not cp_bytes or not iq_bytes:
    print("Params not found")
    sys.exit(1)

CP = messaging.log_from_bytes(cp_bytes, car.CarParams)
CP_IQ = messaging.log_from_bytes(iq_bytes, custom.IQCarParams)

is_honda_eps = use_honda_eps_controller(CP, CP_IQ)
eps_mod = bool(CP_IQ.flags & HondaFlagsIQ.EPS_MODIFIED)

print(f"CP.carFingerprint: {CP.carFingerprint}")
print(f"CP_IQ.flags: {CP_IQ.flags} (EPS_MODIFIED={eps_mod})")
print(f"CP.lateralTuning: {CP.lateralTuning.which()}")
print(f"use_honda_eps_controller() = {is_honda_eps}")
