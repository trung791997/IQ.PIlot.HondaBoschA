with open('/Users/peternguyen/openpilot-radar/selfdrive/controls/lib/longitudinal_planner.py', 'r') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if line.startswith('STOCK_FEEL_DEPTH_BP'):
        print("STOCK constants at", i)
    if 'def brake_onset_ttc' in line:
        print("ttc at", i)
    if 'def brake_release_limited_target' in line:
        print("release at", i)
