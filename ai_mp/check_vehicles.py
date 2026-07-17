import json
with open('/root/ardupilot/Tools/autotest/pysim/vehicleinfo.json') as f:
    d = json.load(f)
for k in sorted(d.keys()):
    print(k)
