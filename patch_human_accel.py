import re

with open('iqpilot/selfdrive/controls/lib/longitudinal_planner.py', 'r') as f:
    content = f.read()

old_accel = "  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)"
new_accel = """  if e2e:
    max_accel = ACCEL_MAX
  else:
    max_accel = get_max_accel(v_ego)
    # HumanAcceleration from Starpilot
    max_accel = float(np.interp(v_cruise, [0., 12.5, 25.0], [max_accel / 4, max_accel / 2, max_accel]))
    max_accel = float(np.interp(v_cruise - v_ego, [0., 1., 5.], [0., 0.5, max_accel]))
"""
content = content.replace(old_accel, new_accel)

with open('iqpilot/selfdrive/controls/lib/longitudinal_planner.py', 'w') as f:
    f.write(content)

