import re

with open('iqpilot/selfdrive/controls/controlsd.py', 'r') as f:
    content = f.read()

# 1. Add imports
import_str = "from iqpilot.selfdrive.controls.lib.latcontrol_honda_eps import LatControlHondaEps, use_honda_eps_controller, eps_lateral_delay, eps_lateral_delay_schedule\n"
content = re.sub(r'(from iqpilot\.selfdrive\.controls\.lib\.latcontrol_torque_v0 import.*?\n)', r'\1' + import_str, content)

# 2. Add LatControlHondaEps initialization
init_str = """    elif use_honda_eps_controller(self.CP, self.params):
      self.LaC = LatControlHondaEps(self.CP, self.CP_IQ, self.CI, DT_CTRL)
"""
content = re.sub(r"(elif self\.CP\.lateralTuning\.which\(\) == 'pid':\n      self\.LaC = LatControlPID\(.*?\n)", init_str + r"\1", content)

# 3. Handle delay schedule application in state update (PR 14/17 logic usually applies delays)
# I need to know where lateral_action_delay is used in controlsd.py to apply eps_lateral_delay.

with open('iqpilot/selfdrive/controls/controlsd.py', 'w') as f:
    f.write(content)
