import ast

with open('/Users/peternguyen/openpilot-radar/selfdrive/controls/lib/latcontrol_pid.py', 'r') as f:
    source = f.read()

import re

# Also extract the BP/V variables
bps = re.findall(r"^NRDR_[A-Z0-9_]+_BP = \[.*?\]$", source, re.MULTILINE)
vs = re.findall(r"^NRDR_[A-Z0-9_]+_V = \[.*?\]$", source, re.MULTILINE)

helpers = bps + vs

# NRDR constants
for const in ["NRDR_ANGLE_RATE_LIMIT_DEG_S", "NRDR_SR_CURVE_BY_FP", "NRDR_SR_CURVE_INVERSE_BY_FP"]:
    match = re.search(f"^{const}.*?(?=\\n\\n|\\Z)", source, re.MULTILINE | re.DOTALL)
    if match:
        helpers.append(match.group(0))

class FuncExtractor(ast.NodeVisitor):
    def __init__(self):
        self.funcs = {}
    def visit_FunctionDef(self, node):
        if node.name in ["_get_param_bool", "_get_param_float", "rate_limit_desired_angle", "solve_angle_from_ratio_curve", "build_steer_ratio_inverse"]:
            self.funcs[node.name] = ast.get_source_segment(source, node)

extractor = FuncExtractor()
extractor.visit(ast.parse(source))

# append in order
for f in ["build_steer_ratio_inverse", "_get_param_bool", "_get_param_float", "rate_limit_desired_angle", "solve_angle_from_ratio_curve"]:
    if f in extractor.funcs:
        helpers.append(extractor.funcs[f])

with open('helpers.py', 'w') as f:
    f.write("\nimport math\nimport numpy as np\n\n" + "\n\n".join(helpers))

