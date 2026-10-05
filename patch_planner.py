import re

with open('iqpilot/selfdrive/controls/lib/longitudinal_planner.py', 'r') as f:
    content = f.read()

# 1. Fix AccelBoost init
content = content.replace("self.accel_boost = AccelBoost(accel_boost_enabled, dt)", "self.accel_boost = AccelBoost(dt)")

# 2. Fix AccelBoost update & apply
old_boost = "    accel_boost = self.accel_boost.update(sm['selfdriveState'].enabled, v_ego, sm['carState'].gasPressed)\n    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration + accel_boost"
new_boost = """    accel_boost_on = self.params.get_bool("GasOverrideBoost")
    model_limited = False # simplify for now
    self.accel_boost.update(bool(sm['selfdriveState'].enabled), bool(sm['carState'].gasPressed), v_ego, model_limited)
    if accel_boost_on:
      output_a_target_e2e = self.accel_boost.apply(sm['modelV2'].action.desiredAcceleration)
    else:
      output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
"""
content = content.replace(old_boost, new_boost)

# 3. Short Plan Read-Ahead (0.3s)
old_action_t = "    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL"
new_action_t = """    if self.params.get_bool("PlannerShortActionTime"):
      action_t = 0.30
    else:
      action_t = self.CP.longitudinalActuatorDelay + DT_MDL
"""
content = content.replace(old_action_t, new_action_t)

with open('iqpilot/selfdrive/controls/lib/longitudinal_planner.py', 'w') as f:
    f.write(content)
