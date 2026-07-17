#!/bin/bash
pkill -f arduplane 2>/dev/null
pkill -f sim_vehicle 2>/dev/null
sleep 2

WIN_IP=$(grep nameserver /etc/resolv.conf | awk '{print $2}')
echo "[1] Windows IP: $WIN_IP"
echo "[2] Starting sim_vehicle.py (may take 60s to compile)..."

cd /root/ardupilot

# Start in background, capture output
nohup python3 Tools/autotest/sim_vehicle.py \
    -v ArduPlane \
    --no-mavproxy \
    --no-rebuild \
    --out "udp:${WIN_IP}:14550" \
    --instance 0 \
    > /tmp/sim_final.log 2>&1 &

SIM_PID=$!
echo "    sim_vehicle.py PID: $SIM_PID"

echo "[3] Waiting up to 90s for SITL to start..."
for i in $(seq 1 18); do
    sleep 5
    ARDUPID=$(pgrep -f "arduplane.*model plane" 2>/dev/null)
    if [ -n "$ARDUPID" ]; then
        echo "    arduplane running! PID=$ARDUPID (after $((i*5))s)"
        break
    fi
    echo "    waiting... ($((i*5))s)"
done

echo "[4] Final status:"
pgrep -la arduplane 2>&1 || echo "arduplane NOT running"
echo "---"
tail -30 /tmp/sim_final.log 2>&1
