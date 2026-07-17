#!/usr/bin/env python3
"""
气球投放飞行剖面仿真分析

本脚本对 BalloonDrop 飞行任务的各阶段进行数学仿真，
验证参数合理性和飞行安全性。

无需 PX4 运行环境，纯数学计算。
"""

import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ===== 飞行参数 =====
CRUISE_SPEED = 25.0       # 大巡航速度 m/s
TRIGGER_RATIO = 0.7       # 触发空速比例
LOAD_FACTOR = 1.7         # 拉起过载 g
MOTOR_RAMP_TIME = 2.0     # 电机启动时间 s
CRUISE_THROTTLE = 0.6     # 巡航油门
RELEASE_ALT = 150.0       # 释放高度 m
G = 9.81                  # 重力加速度 m/s^2

# ===== 计算触发参数 =====
trigger_airspeed = TRIGGER_RATIO * CRUISE_SPEED
pullup_omega = (LOAD_FACTOR - 1) * G / trigger_airspeed  # rad/s
pullup_total_time = (math.pi / 2) / pullup_omega
pullup_radius = trigger_airspeed**2 / ((LOAD_FACTOR - 1) * G)

print("=" * 60)
print("气球投放飞行剖面仿真分析")
print("=" * 60)
print(f"大巡航速度:       {CRUISE_SPEED} m/s")
print(f"触发空速:         {trigger_airspeed} m/s ({TRIGGER_RATIO*100:.0f}%)")
print(f"拉起过载:         {LOAD_FACTOR} g")
print(f"拉起角速度:       {math.degrees(pullup_omega):.1f} deg/s ({pullup_omega:.3f} rad/s)")
print(f"拉起总时间:       {pullup_total_time:.2f} s")
print(f"拉起半径:         {pullup_radius:.1f} m")
print(f"释放高度:         {RELEASE_ALT} m")
print()

# ===== Phase 1: 自由下落 =====
# 从静止自由下落，忽略空气阻力（保守估计）
# V = g * t → t = V_trigger / g
freefall_time = trigger_airspeed / G
freefall_distance = 0.5 * G * freefall_time**2
alt_after_freefall = RELEASE_ALT - freefall_distance

print("--- Phase 1: 自由下落 ---")
print(f"下落时间:         {freefall_time:.2f} s")
print(f"下落距离:         {freefall_distance:.1f} m")
print(f"触发时高度:       {alt_after_freefall:.1f} m")
print()

# ===== Phase 2: 拉起 =====
# 恒定过载拉起，从 -90° 到 0°
# 高度损失 ≈ 拉起半径（四分之一圆弧）
pullup_height_loss = pullup_radius
alt_after_pullup = alt_after_freefall - pullup_height_loss

print("--- Phase 2: 拉起过渡 ---")
print(f"拉起时间:         {pullup_total_time:.2f} s")
print(f"高度损失:         {pullup_height_loss:.1f} m")
print(f"拉起后高度:       {alt_after_pullup:.1f} m")
print()

# ===== Phase 3: 电机启动 =====
# 油门渐增，保持高度
alt_after_motor = alt_after_pullup

print("--- Phase 3: 电机启动 ---")
print(f"渐增时间:         {MOTOR_RAMP_TIME} s")
print(f"油门:             0 → {CRUISE_THROTTLE}")
print(f"高度:             {alt_after_motor:.1f} m (保持)")
print()

# ===== 安全检查 =====
print("=" * 60)
print("安全检查")
print("=" * 60)

min_safe_alt = 50.0  # 最低安全高度

if alt_after_pullup < min_safe_alt:
    print(f"⚠️  警告: 拉起后高度 {alt_after_pullup:.1f}m < 安全高度 {min_safe_alt}m")
    print(f"   建议: 释放高度至少 {RELEASE_ALT + (min_safe_alt - alt_after_pullup):.0f}m")
else:
    print(f"✅ 安全: 拉起后高度 {alt_after_pullup:.1f}m > 安全高度 {min_safe_alt}m")

# 检查拉起过载是否合理
if LOAD_FACTOR > 2.0:
    print(f"⚠️  警告: 过载 {LOAD_FACTOR}g 较高，可能超出结构强度")
elif LOAD_FACTOR < 1.3:
    print(f"⚠️  警告: 过载 {LOAD_FACTOR}g 较低，拉起可能不够快")
else:
    print(f"✅ 合理: 过载 {LOAD_FACTOR}g 在安全范围内")

# 检查空速触发是否合理
if trigger_airspeed < 10:
    print(f"⚠️  警告: 触发空速 {trigger_airspeed:.1f}m/s 较低，可能过早触发")
elif trigger_airspeed > CRUISE_SPEED * 0.9:
    print(f"⚠️  警告: 触发空速 {trigger_airspeed:.1f}m/s 接近巡航速度，留给拉起的高度不足")
else:
    print(f"✅ 合理: 触发空速 {trigger_airspeed:.1f}m/s")

print()

# ===== 轨迹图 =====
# 生成轨迹数据
dt = 0.01
time_data = []
alt_data = []
speed_data = []
pitch_data = []

t = 0
alt = RELEASE_ALT
v垂直 = 0
phase = 1

while phase <= 3 and alt > 0:
    time_data.append(t)
    alt_data.append(alt)
    speed_data.append(v垂直)
    pitch_data.append(-90 if phase <= 2 else 0)

    if phase == 1:  # 自由下落
        v垂直 += G * dt
        alt -= v垂直 * dt
        if v垂直 >= trigger_airspeed:
            phase = 2
            pullup_t = 0

    elif phase == 2:  # 拉起
        pullup_t += dt
        # 简化的拉起轨迹（圆弧）
        angle = -90 + math.degrees(pullup_omega * pullup_t)
        if angle >= 0:
            angle = 0
            phase = 3
            motor_t = 0

        # 速度方向变化
        v垂直_at_pullup = v垂直
        v垂直 = v垂直_at_pullup * math.cos(math.radians(angle + 90))
        alt -= v垂直 * dt

    elif phase == 3:  # 电机启动
        motor_t += dt
        # 保持高度
        pass

    t += dt

    if len(time_data) > 5000:
        break

# ===== 绘图 =====
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
fig.suptitle('Balloon Drop Flight Profile Simulation', fontsize=14)

# 高度图
axes[0].plot(time_data, alt_data, 'b-', linewidth=2)
axes[0].axhline(y=min_safe_alt, color='r', linestyle='--', label=f'Safety Alt ({min_safe_alt}m)')
axes[0].axhline(y=alt_after_pullup, color='g', linestyle='--', label=f'After Pull-up ({alt_after_pullup:.0f}m)')
axes[0].set_ylabel('Altitude (m)')
axes[0].set_title('Altitude Profile')
axes[0].legend()
axes[0].grid(True)

# 空速图
axes[1].plot(time_data, speed_data, 'r-', linewidth=2)
axes[1].axhline(y=trigger_airspeed, color='g', linestyle='--', label=f'Trigger ({trigger_airspeed:.1f} m/s)')
axes[1].axhline(y=CRUISE_SPEED, color='b', linestyle='--', label=f'Cruise ({CRUISE_SPEED} m/s)')
axes[1].set_ylabel('Vertical Speed (m/s)')
axes[1].set_title('Speed Profile')
axes[1].legend()
axes[1].grid(True)

# 俯仰角图
axes[2].plot(time_data, pitch_data, 'g-', linewidth=2)
axes[2].set_ylabel('Pitch Angle (deg)')
axes[2].set_xlabel('Time (s)')
axes[2].set_title('Pitch Angle Profile')
axes[2].grid(True)
axes[2].set_ylim([-100, 10])

plt.tight_layout()
plt.savefig('D:/oezcon/balloon_drop_simulation.png', dpi=150)
print(f"轨迹图已保存: D:/oezcon/balloon_drop_simulation.png")

# ===== 输出推荐参数 =====
print()
print("=" * 60)
print("推荐参数（PX4 中设置）")
print("=" * 60)
print(f"BD_TRIG_RATIO  = {TRIGGER_RATIO}")
print(f"BD_LOAD_FACTOR = {LOAD_FACTOR}")
print(f"BD_CRUISE_SPD  = {CRUISE_SPEED}")
print(f"BD_MOTOR_RAMP  = {MOTOR_RAMP_TIME}")
print(f"BD_CRUISE_THR  = {CRUISE_THROTTLE}")
print(f"BD_PITCH_THRESH= -10")
print()
print(f"释放高度建议:    {RELEASE_ALT} m")
print(f"触发时高度:      {alt_after_freefall:.0f} m")
print(f"拉起后高度:      {alt_after_pullup:.0f} m")
print(f"拉起时间:        {pullup_total_time:.2f} s")
print(f"总时间:          {freefall_time + pullup_total_time + MOTOR_RAMP_TIME:.2f} s")
