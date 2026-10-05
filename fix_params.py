import re

def replace_file(path):
    with open(path, 'r') as f:
        content = f.read()

    # fix GasOverrideBoost
    content = content.replace('accel_boost_on = self.params.get_bool("GasOverrideBoost")',
        'try:\n      accel_boost_on = self.params.get_bool("GasOverrideBoost")\n    except Exception:\n      accel_boost_on = True')

    # fix StockBrakeFeel
    content = content.replace('stock_brake_feel_on = self.params.get_bool("StockBrakeFeel")',
        'try:\n      stock_brake_feel_on = self.params.get_bool("StockBrakeFeel")\n    except Exception:\n      stock_brake_feel_on = True')

    # fix PlannerShortActionTime
    content = content.replace('if self.params.get_bool("PlannerShortActionTime"):',
        'try:\n      short_action = self.params.get_bool("PlannerShortActionTime")\n    except Exception:\n      short_action = True\n    if short_action:')

    # fix VfnOverride
    content = content.replace('vfn_override = self.params.get_bool("NrdrLatVfnOverride")',
        'try:\n        vfn_override = self.params.get_bool("NrdrLatVfnOverride")\n      except Exception:\n        vfn_override = True')

    with open(path, 'w') as f:
        f.write(content)

replace_file('iqpilot/selfdrive/controls/lib/longitudinal_planner.py')
replace_file('artifacts/package_runtime/iqdbc/car/honda/carcontroller.py')
