#!/bin/bash
cd /root/ardupilot/Tools/autotest
nohup /root/ardupilot/build/sitl/bin/arduplane \
  --model plane \
  --defaults default_params/plane.parm \
  --home -35.363261,149.165230,584,353 \
  --serial1 udp:127.0.0.1:14550 \
  --autotest-dir . \
  > /tmp/arduplane.log 2>&1 &
echo "SITL started, PID: $!"
sleep 3
ps aux | grep arduplane | grep -v grep | head -3
