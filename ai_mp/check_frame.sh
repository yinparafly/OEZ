#!/bin/bash
cd /root/ardupilot/Tools/autotest
python3 -c "
from pysim import vehicleinfo
v = vehicleinfo.VehicleInfo()
d = v.get('ArduPlane')
print('default_frame:', d.get('default_frame', 'unknown'))
frames = d.get('frames', {})
plane_frame = frames.get('plane', {})
print('plane frame:', plane_frame)
"
