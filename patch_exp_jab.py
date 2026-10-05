import re

with open('iqpilot/selfdrive/controls/lib/longitudinal_planner.py', 'r') as f:
    content = f.read()

# find where output_a_target_e2e is set
old_e2e = "    else:\n      output_a_target_e2e = sm['modelV2'].action.desiredAcceleration"

new_e2e = """    else:
      output_a_target_e2e = sm['modelV2'].action.desiredAcceleration

    # Exp Mode Brake Jab fix (approximate for e2e since close_lead_cap is removed)
    if self.is_e2e(sm):
      chill_floor = -1.0
      if output_a_target_e2e < chill_floor:
        v_rel = sm['radarState'].leadOne.vRel if sm['radarState'].leadOne.status else 0.0
        if v_rel > -1.0: # not fast closing
          if not hasattr(self, 'exp_close_lead_floor'):
            self.exp_close_lead_floor = chill_floor
          self.exp_close_lead_floor = max(-3.5, self.exp_close_lead_floor - 1.5 * self.dt)
          output_a_target_e2e = max(output_a_target_e2e, self.exp_close_lead_floor)
        else:
          self.exp_close_lead_floor = output_a_target_e2e
      else:
        self.exp_close_lead_floor = chill_floor
"""
content = content.replace(old_e2e, new_e2e)

with open('iqpilot/selfdrive/controls/lib/longitudinal_planner.py', 'w') as f:
    f.write(content)
