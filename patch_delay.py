import re

with open('iqpilot/selfdrive/controls/controlsd.py', 'r') as f:
    content = f.read()

# Add schedule init
init_sched = """
    self.turn_shaping = isinstance(self.LaC, LatControlHondaEps)
    self.lat_delay_schedule = eps_lateral_delay_schedule(self.CP, self.params) if self.turn_shaping else None
"""
content = re.sub(r'(self\.sm = messaging\.SubMaster\(\[\n)', init_sched + r'\n    \1', content)

# Add delay application
new_delay = """    if self.turn_shaping:
      lat_delay_base = eps_lateral_delay(self.lat_delay_schedule, CS.vEgo, self.sm["lateralDelay"].lateralDelay)
    else:
      lat_delay_base = lateral_action_delay(self.params, self.CP, self.sm["lateralDelay"].lateralDelay)
    lat_delay = lat_delay_base + LAT_SMOOTH_SECONDS"""

content = re.sub(r'lat_delay = lateral_action_delay\(self\.params, self\.CP, self\.sm\["lateralDelay"\]\.lateralDelay\) \+ LAT_SMOOTH_SECONDS', new_delay, content)

with open('iqpilot/selfdrive/controls/controlsd.py', 'w') as f:
    f.write(content)
