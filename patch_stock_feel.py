import re

with open('/Users/peternguyen/openpilot-radar/selfdrive/controls/lib/longitudinal_planner.py', 'r') as f:
    source = f.read()

# Extract the constants block
start_idx = source.find('STOCK_FEEL_DEPTH_BP = [2.0')
end_idx = source.find('NEWBORN_LEAD_FRAMES = 40')
stock_consts = source[start_idx:end_idx].strip()

# Extract functions
import ast
class FuncExtractor(ast.NodeVisitor):
    def __init__(self):
        self.funcs = {}
    def visit_FunctionDef(self, node):
        if node.name in ["brake_onset_ttc", "stock_feel_target", "brake_onset_limited_target", "brake_release_limited_target", "brake_release_dwell_target"]:
            self.funcs[node.name] = ast.get_source_segment(source, node)

extractor = FuncExtractor()
extractor.visit(ast.parse(source))

funcs = "\n\n".join(extractor.funcs[n] for n in ["brake_onset_ttc", "stock_feel_target", "brake_onset_limited_target", "brake_release_limited_target", "brake_release_dwell_target"] if n in extractor.funcs)

# We also need BRAKE_RELEASE_JERK and BRAKE_RELEASE_DWELL_TICKS
release_consts = """
BRAKE_RELEASE_JERK = 2.0  # m/s^3
BRAKE_RELEASE_DWELL_TICKS = int(0.2 / 0.05)  # 0.2s at 20Hz
"""

with open('iqpilot/selfdrive/controls/lib/longitudinal_planner.py', 'r') as f:
    target_content = f.read()

# Inject globals right after imports
inject_idx = target_content.find('\nclass LongitudinalPlanner')
injection = f"\n\n{stock_consts}\n\n{release_consts}\n\n{funcs}\n\n"
target_content = target_content[:inject_idx] + injection + target_content[inject_idx:]

# In update(), inject the application
apply_code = """
    # Starpilot custom behaviors (Stock Brake Feel, Resume Brake Ramp)
    stock_brake_feel_on = self.params.get_bool("StockBrakeFeel")
    if stock_brake_feel_on and not sm['carState'].standstill and not output_should_stop_mpc:
      leads = (sm['radarState'].leadOne, sm['radarState'].leadTwo)
      output_a_target = stock_feel_target(leads, a_prev, output_a_target, self.dt)
      
    if not sm['carState'].standstill:
      output_a_target = brake_release_limited_target(a_prev, output_a_target, self.dt)
"""

# Replace `output_a_target = output_a_target_mpc` or whatever the final target logic is.
# Wait, IQ.Pilot doesn't have `output_a_target`. It has `self.output_a_target`. 
# Let's find where `self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)` is.
target_content = target_content.replace(
    "self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)",
    apply_code + "    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)"
)

with open('iqpilot/selfdrive/controls/lib/longitudinal_planner.py', 'w') as f:
    f.write(target_content)

