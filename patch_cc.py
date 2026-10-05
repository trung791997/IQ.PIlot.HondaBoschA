import re

with open('artifacts/package_runtime/iqdbc/car/honda/carcontroller.py', 'r') as f:
    content = f.read()

# Add get_eps_modified_steering_pressed at top level
helper = """
def get_eps_modified_steering_pressed(
  raw_pressed: bool,
  sensor_torque: float,
  torque_cmd: float,
  filter_s: float,
  robust_prev: bool,
) -> tuple[float, bool]:
  import math
  from iqpilot.common.realtime import DT_CTRL
  # simplified VFN override detector
  if raw_pressed:
    if torque_cmd * sensor_torque < 0.0 or abs(torque_cmd) < 0.1:
      filter_s = 0.28
    else:
      filter_s = min(0.28, filter_s + DT_CTRL)
  else:
    filter_s = 0.0

  return filter_s, (filter_s >= 0.28)

"""

# find HondaController class
content = content.replace('class HondaController(AolCarController):', helper + 'class HondaController(AolCarController):')

# in HondaController __init__, add variables
init_vars = """
    self.steering_pressed_filter_s = 0.0
    self.steering_pressed_robust_prev = False
    self.override_ramp = 1.0
    self.lat_active_prev = False
"""
content = re.sub(r'(def __init__\(self, dbc_name, CP, VM\):\n(?:.*\n)*?    self\.brake_steady = 0\.\n)', r'\1' + init_vars, content)

# in update(), intercept actuators.torque
update_intercept = """
    torque_cmd = float(actuators.torque)
    if CC.latActive:
      vfn_override = self.params.get_bool("NrdrLatVfnOverride")
      if vfn_override:
        self.steering_pressed_filter_s, steering_pressed = get_eps_modified_steering_pressed(
          bool(CS.out.steeringPressed),
          float(getattr(CS.out, "steeringTorque", 0.0)),
          float(torque_cmd),
          self.steering_pressed_filter_s,
          self.steering_pressed_robust_prev,
        )
        self.steering_pressed_robust_prev = steering_pressed
        
        if not self.lat_active_prev:
          self.override_ramp = 0.0
          
        if steering_pressed:
          self.override_ramp = max(0.0, self.override_ramp - 0.01 / 0.0) # 0 override torque scale, fade down 0
        else:
          self.override_ramp = min(1.0, self.override_ramp + 0.01 / 1.5) # fade up 1.5s
          
        torque_cmd *= self.override_ramp
    else:
      self.override_ramp = 0.0
      self.steering_pressed_filter_s = 0.0
      self.steering_pressed_robust_prev = False
    
    self.lat_active_prev = CC.latActive
    actuators.torque = torque_cmd
"""

# inject before # *** rate limit steer ***
content = content.replace('    # *** rate limit steer ***', update_intercept + '    # *** rate limit steer ***')

with open('artifacts/package_runtime/iqdbc/car/honda/carcontroller.py', 'w') as f:
    f.write(content)
