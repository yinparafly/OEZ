#!/usr/bin/env python3
"""
Balloon Drop 精确仿真 — 变半径拉起
考虑重力在不同俯仰角时对向心力的影响
"""
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ===== 飞机参数 =====
m = 5.0        # kg
S = 0.55       # m²
b = 2.0        # m
V_stall = 15.0 # m/s (水平失速)
V_cruise = 22.0
V_trigger = 12.0
n_cmd = 2.0    # 命令过载 g
g = 9.81
rho = 1.225

# ===== 精确拉起仿真 (数值积分) =====
def simulate_pullup(V_entry, n_cmd, pitch_start_deg, pitch_end_deg, dt=0.01):
    """
    精确拉起仿真:
    - 飞机以恒定n_cmd倍重力的升力拉起
    - 重力在不同pitch角时对向心加速度的贡献不同
    - 垂直平面内的运动方程:
      切向: m*dV/dt = -m*g*sin(theta) (重力切向分量)
      法向: n*m*g - m*g*cos(theta) = m*V^2/R (升力 - 重力法向分量 = 向心力)
    """
    theta = math.radians(pitch_start_deg)  # 当前俯仰角 (负=低头)
    V = V_entry
    x, y = 0, 0  # x=水平前进, y=高度(正=上)
    
    trajectory = []
    t = 0
    
    while theta < math.radians(pitch_end_deg) and t < 30:
        # 法向力: L = n*m*g (恒定过载)
        # 向心加速度: a_n = (L - m*g*cos(theta)) / m = (n-1)*g*cos(theta)  (不对,更精确)
        # 实际上: L指向速度垂直方向, 重力分解:
        # 切向加速度: a_t = -g*sin(theta) (theta负=低头, sin负=加速)
        # 法向加速度: a_n = (L/m) - g*cos(theta) = n*g - g*cos(theta)
        # 曲率半径: R = V^2 / a_n
        # 角速度: omega = V / R = a_n / V
        
        a_t = -g * math.sin(theta)  # 切向: 低头时加速
        a_n = n_cmd * g - g * math.cos(theta)  # 法向: 升力 - 重力法向分量
        
        if a_n <= 0:
            break  # 升力不足以维持拉起
        
        omega = a_n / V  # 角速度 rad/s (抬头为正)
        
        # 更新
        V += a_t * dt
        theta += omega * dt
        
        # 位置积分
        x += V * math.cos(theta) * dt
        y += V * math.sin(theta) * dt
        
        R = V**2 / a_n if a_n > 0 else 999
        trajectory.append({
            't': t, 'theta_deg': math.degrees(theta), 'V': V,
            'x': x, 'y': y, 'R': R, 'a_n': a_n, 'omega_deg': math.degrees(omega)
        })
        t += dt
    
    return trajectory

# ===== 运行仿真 =====
traj = simulate_pullup(V_trigger, n_cmd, -80, 0)

print("=" * 60)
print("精确拉起仿真 (变半径, 考虑重力)")
print("=" * 60)
print(f"入口速度: {V_trigger} m/s")
print(f"命令过载: {n_cmd}g")
print(f"入口俯仰角: -80°")
print(f"目标俯仰角: 0°")
print()

# 关键指标
if traj:
    final = traj[-1]
    print(f"拉起时间: {final['t']:.2f} s")
    print(f"最终速度: {final['V']:.1f} m/s")
    print(f"高度损失: {abs(final['y']):.1f} m")
    print(f"水平距离: {final['x']:.1f} m")
    print(f"最终俯仰角: {final['theta_deg']:.1f}°")
    print()
    
    # 半径变化
    Rs = [p['R'] for p in traj]
    print(f"半径范围: {min(Rs):.1f} ~ {max(Rs):.1f} m")
    print(f"初始半径: {traj[0]['R']:.1f} m (theta=-80°)")
    mid_idx = len(traj)//2
    print(f"中间半径: {traj[mid_idx]['R']:.1f} m (theta={traj[mid_idx]['theta_deg']:.0f}°)")
    print(f"末端半径: {traj[-1]['R']:.1f} m (theta=0°)")
    
    # 对比恒定半径(简化模型)
    R_simple = V_trigger**2 / ((n_cmd - 1) * g)
    print(f"\n简化模型(恒定R): {R_simple:.1f} m, 时间 {math.pi/2/((n_cmd-1)*g/V_trigger):.2f} s")
    print(f"精确模型(变R):   {final['R']:.1f} m, 时间 {final['t']:.2f} s")
    
    # ===== 绘图 =====
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(f'Balloon Drop Pull-up Simulation (n={n_cmd}g, V0={V_trigger}m/s, m={m}kg)', fontsize=14)
    
    # 1. 飞行轨迹 (x-y)
    ax = axes[0, 0]
    xs = [p['x'] for p in traj]
    ys = [p['y'] for p in traj]
    ax.plot(xs, ys, 'b-', linewidth=2)
    ax.set_xlabel('Horizontal distance (m)')
    ax.set_ylabel('Altitude change (m)')
    ax.set_title('Flight Trajectory')
    ax.grid(True)
    ax.set_aspect('equal')
    ax.invert_yaxis()  # Y轴反转 (下降为正)
    
    # 2. 俯仰角 vs 时间
    ax = axes[0, 1]
    ts = [p['t'] for p in traj]
    thetas = [p['theta_deg'] for p in traj]
    ax.plot(ts, thetas, 'r-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pitch angle (deg)')
    ax.set_title('Pitch Angle')
    ax.grid(True)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    
    # 3. 速度 vs 时间
    ax = axes[1, 0]
    vs = [p['V'] for p in traj]
    ax.plot(ts, vs, 'g-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Airspeed (m/s)')
    ax.set_title('Airspeed')
    ax.grid(True)
    ax.axhline(y=V_stall, color='red', linestyle='--', alpha=0.5, label=f'Stall={V_stall}m/s')
    ax.legend()
    
    # 4. 曲率半径 vs 俯仰角
    ax = axes[1, 1]
    ax.plot(thetas, Rs, 'm-', linewidth=2)
    ax.set_xlabel('Pitch angle (deg)')
    ax.set_ylabel('Turn radius (m)')
    ax.set_title('Turn Radius vs Pitch')
    ax.grid(True)
    ax.invert_xaxis()
    
    plt.tight_layout()
    plt.savefig('D:/oezcon/ai_mp/pullup_trajectory.png', dpi=150)
    print(f"\n图表已保存: D:/oezcon/ai_mp/pullup_trajectory.png")
    
    # 高度损失 vs 释放高度
    print(f"\n--- 最低安全释放高度 ---")
    alt_loss = abs(final['y'])
    print(f"拉起高度损失: {alt_loss:.1f} m")
    print(f"建议最低释放高度: {alt_loss + 30:.0f} m (含30m安全裕度)")
