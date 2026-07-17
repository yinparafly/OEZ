#!/bin/bash
pkill -f arduplane 2>/dev/null
sleep 1
/root/ardupilot/build/sitl/bin/arduplane -I0 --model plane --speedup 3 --home CMAC --defaults /root/ardupilot/Tools/autotest/models/plane.parm > /dev/null 2>&1 &
sleep 4
python3 /mnt/d/oezcon/ai_mp/display_continuous.py 2>&1
pkill -f arduplane
