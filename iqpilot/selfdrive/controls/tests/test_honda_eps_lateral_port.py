"""StarPilot LatControlHondaEps pieces the first port missed (JamesL787 PRs 14/17): the Civic Bosch modified-EPS
tune and torque map, the torque rate limit being off for the EPS-controller cars, and the model action ramp."""
import pytest

from iqdbc.car import structs
from iqdbc.car.car_helpers import interfaces
from iqdbc.car.honda.values import CAR
from iqpilot.common.realtime import DT_CTRL, DT_MDL
from iqpilot.selfdrive.controls.lib.model_action_interp import ModelActionInterp

MPH = 0.44704


def _params(cand, eps_fw: bytes | None):
  CI = interfaces[cand]
  fp = {0: {}, 1: {}, 2: {}}
  car_fw = [] if eps_fw is None else [structs.CarParams.CarFw(ecu="eps", fwVersion=eps_fw)]
  CP = CI.get_params(cand, fp, car_fw, alpha_long=True, is_release=False, docs=False)
  CP_IQ = CI.get_params_iq(CP, cand, fp, car_fw, alpha_long=True, is_release_iq=False, docs=False)
  return CI, CP, CP_IQ


@pytest.mark.parametrize("fw, torque_max", [(b"39990-TBA,C020", 4096), (b"39990-TBA,C120", 3840)])
def test_civic_bosch_modified_eps_tune(fw, torque_max):
  _, CP, _ = _params(str(CAR.HONDA_CIVIC_BOSCH), fw)
  pid = CP.lateralTuning.pid
  bp = [0., 25. * MPH - 1e-3, 25. * MPH, 50. * MPH]
  assert list(pid.kpBP) == pytest.approx(bp) and list(pid.kpV) == pytest.approx([0.018, 0.024, 0.048, 0.060])
  assert list(pid.kiBP) == pytest.approx(bp) and list(pid.kiV) == pytest.approx([0.006, 0.008, 0.016, 0.020])
  assert pid.kf == pytest.approx(3.6e-6)
  # identity map: what CIVIC_BOSCH_C020's feedforward (e4_per_output 4096) assumes
  assert list(CP.lateralParams.torqueBP) == [0, torque_max]
  assert list(CP.lateralParams.torqueV) == [0, torque_max]


def test_civic_bosch_stock_eps_untouched():
  _, CP, _ = _params(str(CAR.HONDA_CIVIC_BOSCH), None)
  assert CP.lateralTuning.pid.kf != pytest.approx(3.6e-6)
  assert list(CP.lateralTuning.pid.kpV) != pytest.approx([0.018, 0.024, 0.048, 0.060])


@pytest.mark.parametrize("cand, fw, limited", [
  (str(CAR.HONDA_CIVIC_BOSCH), b"39990-TBA,C020", False),
  (str(CAR.HONDA_CIVIC_BOSCH), None, True),
  (str(CAR.HONDA_ACCORD), None, True),
])
def test_steer_delta_limiter_gate(cand, fw, limited):
  CI, CP, CP_IQ = _params(cand, fw)
  ci = CI(CP, CP_IQ)
  assert ci.CC.steer_delta_limiter is limited


def test_model_action_interp_ramps_across_frame():
  m = ModelActionInterp()
  assert m.update(0.0, True, True) == 0.0
  m.update(0.01, True, True)  # new model frame: ramp starts from what is commanded now
  steps = int(round(DT_MDL / DT_CTRL))
  out = [m.update(0.01, False, True) for _ in range(steps)]
  assert all(b > a for a, b in zip(out, out[1:], strict=False))
  assert out[0] == pytest.approx(0.01 * DT_CTRL / DT_MDL)
  assert out[-1] == pytest.approx(0.01)
  assert m.update(0.01, False, True) == pytest.approx(0.01)  # holds once the frame is covered


def test_model_action_interp_continuous_on_early_frame():
  m = ModelActionInterp()
  m.update(0.0, True, True)
  m.update(0.01, True, True)
  mid = [m.update(0.01, False, True) for _ in range(2)][-1]
  # next frame arrives mid-ramp: no jump, ramps from the commanded value
  assert m.update(-0.01, True, True) == pytest.approx(mid)


def test_model_action_interp_passthrough_when_inactive():
  m = ModelActionInterp()
  m.update(0.0, True, True)
  m.update(0.02, True, True)
  assert m.update(0.005, False, False) == 0.005
  assert m.update(0.007, False, True) == pytest.approx(0.005)  # re-engage starts from the reset value
