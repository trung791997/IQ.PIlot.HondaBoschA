import re

with open('iqpilot/selfdrive/controls/lib/longitudinal_planner.py', 'r') as f:
    content = f.read()

# 1. Init
content = content.replace("self.accel_boost = AccelBoost(dt)", "self.accel_boost = AccelBoost(accel_boost_enabled, dt)")

# 2. Update and apply
old_block = """    try:
      accel_boost_on = self.params.get_bool("GasOverrideBoost")
    except Exception:
      accel_boost_on = True
    model_limited = False # simplify for now
    self.accel_boost.update(bool(sm['selfdriveState'].enabled), bool(sm['carState'].gasPressed), v_ego, model_limited)
    if accel_boost_on:
      output_a_target_e2e = self.accel_boost.apply(sm['modelV2'].action.desiredAcceleration)
    else:
      output_a_target_e2e = sm['modelV2'].action.desiredAcceleration"""

new_block = """    accel_boost = self.accel_boost.update(sm['selfdriveState'].enabled, v_ego, sm['carState'].gasPressed)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration + accel_boost"""

content = content.replace(old_block, new_block)

with open('iqpilot/selfdrive/controls/lib/longitudinal_planner.py', 'w') as f:
    f.write(content)

