with open('/Users/peternguyen/openpilot-radar/opendbc/car/honda/carcontroller.py', 'r') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if 'def _filtered_steering_pressed' in line:
        print("FOUND", i)
