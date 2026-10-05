import re

with open('iqpilot/selfdrive/controls/controlsd.py', 'r') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if "elif use_honda_eps_controller" in line:
        lines[i] = "    elif use_honda_eps_controller(self.CP, self.params):\n"
    elif "elif self.CP.lateralTuning.which() == 'pid':" in line:
        lines[i] = "    elif self.CP.lateralTuning.which() == 'pid':\n"

with open('iqpilot/selfdrive/controls/controlsd.py', 'w') as f:
    f.writelines(lines)
