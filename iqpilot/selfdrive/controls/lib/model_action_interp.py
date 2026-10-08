"""Ramp modeld's 20 Hz lateral action across the model frame instead of holding it.

Ported from StarPilot controlsd (NrdrLatModelActionInterp, default on there; used with LatControlHondaEps, JamesL787
PRs 14/17). modeld publishes at 20 Hz while controlsd runs at 100 Hz, so holding the newest value is a zero-order
hold: the target steps once per model frame and sits still for the other four. clip_curvature smears that at road
speed, but its allowance is MAX_LATERAL_JERK / v_ego**2 and below roughly 20 mph it does not bind, so the staircase
reaches the rack. StarPilot measured it on route 00000276 under 15 mph: 2.0 deg p50 / 6.9 deg p90 wheel-angle steps
per model frame, and 19 % of the command's chatter power in the 15-22 Hz band.

Ramping from the value currently commanded keeps the target continuous by construction, at a cost of at most one
model frame of transport delay (which lagd learns anyway), instead of the broadband lag a low-pass would add.
"""
from iqpilot.common.realtime import DT_CTRL, DT_MDL


class ModelActionInterp:
  def __init__(self):
    self.held = 0.0
    self.start = 0.0
    self.target = 0.0
    self.elapsed = 0.0

  def reset(self, value: float) -> float:
    self.held = self.start = self.target = float(value)
    self.elapsed = 0.0
    return self.held

  def update(self, value: float, model_updated: bool, active: bool) -> float:
    if not active:
      return self.reset(value)
    if model_updated:
      self.start = self.held
      self.target = float(value)
      self.elapsed = 0.0
    else:
      self.elapsed += DT_CTRL
    blend = min(self.elapsed / DT_MDL, 1.0)
    self.held = self.start + blend * (self.target - self.start)
    return self.held
