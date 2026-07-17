#!/bin/bash
# ============================================
# ArduPilot SITL 一键启动脚本
# 用法: bash test_run.sh [命令]
# 默认: 启动 SITL + 运行 status 测试
# ============================================

CMD=${1:-"status"}
SITL_LOG="/tmp/sitl_run.log"
SITL_PORT="tcp:127.0.0.1:5760"

echo "=========================================="
echo "ArduPilot SITL 自动化测试"
echo "=========================================="
echo "命令: $CMD"
echo "时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

# Step 1: 清理旧进程
echo "[1] 清理旧进程..."
pkill -f arduplane 2>/dev/null
sleep 1

# Step 2: 启动 SITL
echo "[2] 启动 ArduPlane SITL..."
/root/ardupilot/build/sitl/bin/arduplane \
    -I0 --model plane --speedup 1 --home CMAC \
    > $SITL_LOG 2>&1 &
SITL_PID=$!
sleep 4

# 检查 SITL 是否运行
if ! kill -0 $SITL_PID 2>/dev/null; then
    echo "    SITL 启动失败!"
    cat $SITL_LOG
    exit 1
fi
echo "    SITL PID=$SITL_PID"

# Step 3: 等待端口就绪
echo "[3] 等待 SITL 端口就绪..."
for i in $(seq 1 10); do
    if python3 -c "import socket; s=socket.socket(); s.settimeout(1); s.connect(('127.0.0.1',5760)); s.close()" 2>/dev/null; then
        echo "    端口 5760 就绪"
        break
    fi
    sleep 1
done

# Step 4: 运行命令
echo "[4] 运行命令: ai_mp2.py --port sitl --cmd $CMD"
echo ""
python3 /mnt/d/oezcon/ai_mp/ai_mp2.py --port sitl --cmd $CMD ${@:2}
EXIT_CODE=$?

echo ""
echo "=========================================="
echo "测试完成 (exit=$EXIT_CODE)"
echo "=========================================="

# Step 5: 清理
echo "[5] 停止 SITL..."
kill $SITL_PID 2>/dev/null
echo "Done."
