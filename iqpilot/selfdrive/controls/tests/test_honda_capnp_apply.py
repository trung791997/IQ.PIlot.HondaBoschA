"""Honda CarController.apply with the real capnp CarControl card passes on the device.

carControl arrives as a read-only capnp reader there; the plain-struct car-interface tests cannot see writes into it
(the VFN override once did `actuators.torque = ...`, which crashed card on the first cycle and left the stock radar live).
"""
import time

import pytest

from iqpilot.cereal import car, custom
from iqpilot.selfdrive.car.helpers import convert_iq_car_control_compact
from iqdbc.car.car_helpers import interfaces
from iqdbc.car.honda.values import CAR

LongState = car.CarControl.Actuators.LongControlState


def _interface(eps_modified: bool):
  cand = str(CAR.HONDA_CIVIC_BOSCH)
  CI = interfaces[cand]
  fp = {0: {}, 1: {}, 2: {}}
  CP = CI.get_params(cand, fp, [], alpha_long=True, is_release=False, docs=False)
  CP_IQ = CI.get_params_iq(CP, cand, fp, [], alpha_long=True, is_release_iq=False, docs=False)
  ci = CI(CP, CP_IQ)
  ci.CC.vfn_override = eps_modified
  ci.update([])
  return ci


@pytest.mark.parametrize("eps_modified", [False, True])
@pytest.mark.parametrize("enabled", [False, True])
def test_apply_with_capnp_carcontrol(eps_modified, enabled):
  ci = _interface(eps_modified)
  b = car.CarControl.new_message()
  b.enabled = b.latActive = b.longActive = enabled
  b.actuators.accel = -1.0 if enabled else 0.0
  b.actuators.torque = 0.3 if enabled else 0.0
  b.actuators.longControlState = LongState.pid
  cc = b.as_reader()
  cc_iq = convert_iq_car_control_compact(custom.IQCarControl.new_message().as_reader(), include_leads=False)
  for _ in range(5):
    out, _sends = ci.apply(cc, cc_iq, int(time.monotonic() * 1e9), None)
  assert out.torque == pytest.approx(ci.CC.last_torque)
