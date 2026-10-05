import re

with open('iqpilot/selfdrive/controls/controlsd.py', 'r') as f:
    content = f.read()

init_sched = """
    self.turn_shaping = hasattr(self, 'LaC') and isinstance(self.LaC, LatControlHondaEps)
    self.lat_delay_schedule = eps_lateral_delay_schedule(self.CP, self.params) if self.turn_shaping else None
"""
content = re.sub(r'(self\.sm = messaging\.SubMaster\()', init_sched + r'    \1', content)

# Also fix the indentation of `if self.turn_shaping:`
content = content.replace("        if self.turn_shaping:", "    if self.turn_shaping:")

with open('iqpilot/selfdrive/controls/controlsd.py', 'w') as f:
    f.write(content)
