
import math
import numpy as np

NRDR_CIVIC_BOSCH_SR_CURVE_V = [14.960, 14.960, 14.910, 14.740, 14.210, 13.630, 13.120, 12.904, 12.774]

NRDR_CRV_5G_SR_CURVE_V = [18.10, 17.80, 16.30, 15.30, 14.90, 14.60]

NRDR_INSIGHT_SR_CURVE_V = [16.82, 12.58]

NRDR_MODIFIED_EPS_KF_V = [2.4e-6, 1.8e-6, 3.6e-6, 6.0e-6]

NRDR_ANGLE_RATE_LIMIT_DEG_S = 300.0  # 0 disables

NRDR_SR_CURVE_BY_FP = {
  "HONDA_CLARITY": (NRDR_CLARITY_SR_CURVE_BP, NRDR_CLARITY_SR_CURVE_V),
  "HONDA_CIVIC_BOSCH": (NRDR_CIVIC_BOSCH_SR_CURVE_BP, NRDR_CIVIC_BOSCH_SR_CURVE_V),
  "HONDA_CRV_5G": (NRDR_CRV_5G_SR_CURVE_BP, NRDR_CRV_5G_SR_CURVE_V),
  "HONDA_INSIGHT": (NRDR_INSIGHT_SR_CURVE_BP, NRDR_INSIGHT_SR_CURVE_V),
}

NRDR_SR_CURVE_INVERSE_BY_FP = {
  fingerprint: build_steer_ratio_inverse(*curve) for fingerprint, curve in NRDR_SR_CURVE_BY_FP.items()
}

def build_steer_ratio_inverse(curve_bp, curve_v) -> list[float]:
  """Unit-ratio angle breakpoints for a measured effective-ratio curve.

  VehicleModel.get_steer_from_curvature is exactly linear in sR -- curvature_factor and
  roll_compensation depend only on the vehicle's mass, geometry and speed -- so the desired
  wheel angle satisfies

      theta_des = A * sR(|theta_des|)

  where A is the angle the same command would need at sR = 1. Evaluating x / sR(x) at each
  knot gives the A that lands exactly on that knot, and since sR is non-increasing while x
  increases those values are strictly increasing -- so these bracket the solution for any A,
  in one step and without ever referring to the measured angle. solve_angle_from_ratio_curve
  then closes the segment exactly; see there for why interpolating this table is not enough.
  """
  assert all(low < high for low, high in zip(curve_bp, curve_bp[1:], strict=False)), \
    "ratio curve angle breakpoints must be strictly increasing"
  inverse_bp = [angle / ratio for angle, ratio in zip(curve_bp, curve_v, strict=True)]
  assert all(low < high for low, high in zip(inverse_bp, inverse_bp[1:], strict=False)), \
    "ratio curve is not invertible: x / sR(x) must be strictly increasing"
  return inverse_bp

def _get_param_bool(params, key, default=False):
  try:
    return bool(params.get_bool(key))
  except Exception:
    return default

def _get_param_float(params, key, default, min_value=None, max_value=None, scale=1.0):
  try:
    value = params.get(key)
  except Exception:
    value = None
  if value is None:
    ret = default
  else:
    try:
      if isinstance(value, bytes):
        value = value.decode("utf-8")
      ret = float(value) / scale
    except (AttributeError, TypeError, ValueError):
      ret = default

  if min_value is not None:
    ret = max(min_value, ret)
  if max_value is not None:
    ret = min(max_value, ret)
  return ret

def rate_limit_desired_angle(angle_deg: float, prev_angle_deg: float, max_rate_deg_s: float, dt: float) -> float:
  if max_rate_deg_s <= 0.0 or not math.isfinite(angle_deg):
    return angle_deg
  max_delta = max_rate_deg_s * dt
  return float(min(max(angle_deg, prev_angle_deg - max_delta), prev_angle_deg + max_delta))

def solve_angle_from_ratio_curve(unit_ratio_angle_deg: float, curve_bp, curve_v, inverse_bp) -> float:
  magnitude = abs(unit_ratio_angle_deg)
  if magnitude >= inverse_bp[-1]:
    # Past the table, np.interp would clamp the ANGLE. The curve clamps the RATIO, so keep
    # extrapolating at the final ratio -- exactly what selecting sR by interpolation did.
    solved = magnitude * curve_v[-1]
  else:
    # inverse_bp is exact AT the knots but interpolating it is not: that would make the
    # solution linear in A, and the inverse of a piecewise-linear sR is not. The drift shows
    # up mid-segment where the knots are widest -- 0.3 deg at 120 deg, 1.7 deg at 350 deg on
    # the Clarity curve, which is the same order as the measured-angle error being removed.
    #
    # Inside the bracketing segment sR is linear, sR(x) = m*x + c, so x = A * sR(x) closes in
    # one step: x = A*c / (1 - A*m). m <= 0 because the ratio is non-increasing and A >= 0, so
    # the denominator is never below 1 and this cannot blow up.
    index = int(np.searchsorted(inverse_bp, magnitude, side="right")) - 1
    index = min(max(index, 0), len(curve_bp) - 2)
    slope = (curve_v[index + 1] - curve_v[index]) / (curve_bp[index + 1] - curve_bp[index])
    intercept = curve_v[index] - slope * curve_bp[index]
    solved = magnitude * intercept / (1.0 - magnitude * slope)
  return math.copysign(solved, unit_ratio_angle_deg)