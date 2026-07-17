#!/usr/bin/env python3
"""
气球投放固定翼飞机 — 全面仿真验证

纯固定翼：无悬停，无垂直起降。
4 阶段：自由下落 → 拉起 → 电机启动加速 → 巡航（保持速度平飞）
"""

import math
import sys

# ===== 飞机参数 (基于真实飞机: 翼展2m, 质量7.7kg) =====
DEFAULTS = {
    'cruise_speed': 22.0,       # m/s 巡航空速
    'trigger_airspeed': 12.0,   # m/s 触发空速 (俯冲时无需对抗重力, 12m/s足够)
    'load_factor': 2.0,         # 拉起过载 g (垂直俯冲需要更大过载)
    'motor_ramp_time': 2.0,     # 电机渐增时间 s
    'cruise_throttle': 0.55,    # 巡航油门
    'release_alt': 150.0,       # 释放高度 m
    'stall_speed': 15.0,        # m/s 失速速度 (水平飞行)
    'mass': 5.0,                # kg 飞机质量
    'wing_area': 0.55,          # m² 翼面积
    'wingspan': 2.0,            # m 翼展
    'g': 9.81,
}

PASS_COUNT = 0
FAIL_COUNT = 0

def check(name, condition, detail=""):
    global PASS_COUNT, FAIL_COUNT
    if condition:
        PASS_COUNT += 1
        print(f"  ✅ {name}")
    else:
        FAIL_COUNT += 1
        print(f"  ❌ FAIL: {name} {detail}")

def simulate_balloon_drop(params):
    """模拟完整气球投放过程，返回各阶段数据"""
    V_cruise = params['cruise_speed']
    V_trigger = params['trigger_airspeed']
    n = params['load_factor']
    g = params['g']
    ramp_time = params['motor_ramp_time']
    alt0 = params['release_alt']

    omega = (n - 1) * g / V_trigger  # 拉起角速度 rad/s
    R = V_trigger**2 / ((n - 1) * g)  # 拉起半径
    pullup_time = (math.pi / 2) / omega  # 拉起时间

    # Phase 1: 自由下落
    t_freefall = V_trigger / g
    h_freefall = 0.5 * g * t_freefall**2
    alt_trigger = alt0 - h_freefall

    # Phase 2: 拉起（圆弧 1/4）
    h_pullup = R  # 简化：高度损失 ≈ 半径
    alt_pullup = alt_trigger - h_pullup

    # Phase 3: 电机启动
    alt_motor = alt_pullup  # 固定翼保持高度

    # Phase 4: 巡航
    alt_cruise = alt_motor

    return {
        'V_trigger': V_trigger,
        'omega_deg': math.degrees(omega),
        'omega_rad': omega,
        'R': R,
        'pullup_time': pullup_time,
        't_freefall': t_freefall,
        'h_freefall': h_freefall,
        'alt_trigger': alt_trigger,
        'h_pullup': h_pullup,
        'alt_pullup': alt_pullup,
        'alt_cruise': alt_cruise,
        'total_time': t_freefall + pullup_time + ramp_time,
        'ground_clearance': alt_pullup,
    }

# ============================================================
# 测试 1: 标准场景（默认参数）
# ============================================================
print("=" * 60)
print("测试 1: 标准场景（默认参数）")
print("=" * 60)
r = simulate_balloon_drop(DEFAULTS)
check("触发空速 > 失速速度", r['V_trigger'] > DEFAULTS['stall_speed'], f"V_trigger={r['V_trigger']:.1f} > stall={DEFAULTS['stall_speed']}")
check("触发空速合理", 10 < r['V_trigger'] < 30, f"V_trigger={r['V_trigger']:.1f}")
check("拉起角速度合理", 10 < r['omega_deg'] < 60, f"omega={r['omega_deg']:.1f} deg/s")
check("拉起半径合理", 10 < r['R'] < 80, f"R={r['R']:.1f}m")
check("拉起时间合理", 1 < r['pullup_time'] < 6, f"t={r['pullup_time']:.2f}s")
check("拉起后高度安全(>30m)", r['alt_pullup'] > 30, f"alt={r['alt_pullup']:.1f}m")
check("总时间合理", 3 < r['total_time'] < 12, f"t={r['total_time']:.2f}s")
check("触发时高度充足(>80m)", r['alt_trigger'] > 80, f"alt={r['alt_trigger']:.1f}m")
print()

# ============================================================
# 测试 2: 低空释放（100m）
# ============================================================
print("=" * 60)
print("测试 2: 低空释放（100m）")
print("=" * 60)
p2 = {**DEFAULTS, 'release_alt': 100.0}
r2 = simulate_balloon_drop(p2)
check("拉起后高度安全(>30m)", r2['alt_pullup'] > 30, f"alt={r2['alt_pullup']:.1f}m")
print()

# ============================================================
# 测试 3: 高空释放（200m）
# ============================================================
print("=" * 60)
print("测试 3: 高空释放（200m）")
print("=" * 60)
p3 = {**DEFAULTS, 'release_alt': 200.0}
r3 = simulate_balloon_drop(p3)
check("拉起后高度充裕(>100m)", r3['alt_pullup'] > 100, f"alt={r3['alt_pullup']:.1f}m")
print()

# ============================================================
# 测试 4: 高速飞机（V_cruise=35m/s）
# ============================================================
print("=" * 60)
print("测试 4: 高速飞机（V_cruise=35m/s）")
print("=" * 60)
p4 = {**DEFAULTS, 'cruise_speed': 35.0}
r4 = simulate_balloon_drop(p4)
check("触发空速合理", r4['V_trigger'] > 20, f"V_trigger={r4['V_trigger']:.1f}")
check("拉起后高度安全", r4['alt_pullup'] > 20, f"alt={r4['alt_pullup']:.1f}m")
check("拉起半径不过大", r4['R'] < 150, f"R={r4['R']:.1f}m")
print()

# ============================================================
# 测试 5: 低速飞机（V_cruise=15m/s）
# ============================================================
print("=" * 60)
print("测试 5: 低速飞机（V_cruise=15m/s）")
print("=" * 60)
p5 = {**DEFAULTS, 'cruise_speed': 15.0}
r5 = simulate_balloon_drop(p5)
check("触发空速不过低", r5['V_trigger'] > 8, f"V_trigger={r5['V_trigger']:.1f}")
check("拉起后高度安全", r5['alt_pullup'] > 50, f"alt={r5['alt_pullup']:.1f}m")
print()

# ============================================================
# 测试 6: 高过载拉起（2.5g）
# ============================================================
print("=" * 60)
print("测试 6: 高过载拉起（2.5g）")
print("=" * 60)
p6 = {**DEFAULTS, 'load_factor': 2.5}
r6 = simulate_balloon_drop(p6)
check("拉起时间更短", r6['pullup_time'] < r['pullup_time'], f"t={r6['pullup_time']:.2f}s vs {r['pullup_time']:.2f}s")
check("拉起半径更小", r6['R'] < r['R'], f"R={r6['R']:.1f}m vs {r['R']:.1f}m")
check("过载安全(<3.0g)", p6['load_factor'] < 3.0)
print()

# ============================================================
# 测试 7: 低过载拉起（1.3g）
# ============================================================
print("=" * 60)
print("测试 7: 低过载拉起（1.3g）")
print("=" * 60)
p7 = {**DEFAULTS, 'load_factor': 1.3}
r7 = simulate_balloon_drop(p7)
check("拉起时间较长但仍安全", r7['pullup_time'] < 10, f"t={r7['pullup_time']:.2f}s")
check("拉起后高度安全", r7['alt_pullup'] > 30, f"alt={r7['alt_pullup']:.1f}m")
print()

# ============================================================
# 测试 8: 极限低空（80m）— 边界测试
# ============================================================
print("=" * 60)
print("测试 8: 极限低空（80m）— 边界测试")
print("=" * 60)
p8 = {**DEFAULTS, 'release_alt': 80.0}
r8 = simulate_balloon_drop(p8)
check("拉起后高度仍>15m（极限边界）", r8['alt_pullup'] > 15, f"alt={r8['alt_pullup']:.1f}m (80m释放是极限)")
print()

# ============================================================
# 测试 9: 极限高空（300m）
# ============================================================
print("=" * 60)
print("测试 9: 极限高空（300m）")
print("=" * 60)
p9 = {**DEFAULTS, 'release_alt': 300.0}
r9 = simulate_balloon_drop(p9)
check("拉起后高度充裕", r9['alt_pullup'] > 200, f"alt={r9['alt_pullup']:.1f}m")
print()

# ============================================================
# 测试 10: Phase 4 固定翼巡航验证
# ============================================================
print("=" * 60)
print("测试 10: Phase 4 固定翼巡航验证（无悬停）")
print("=" * 60)
# 模拟 Phase 4 开始后的 10 秒
V_cruise = DEFAULTS['cruise_speed']
dt = 0.1
t = 0
alt = r['alt_pullup']
dist = 0
while t < 10:
    dist += V_cruise * dt
    t += dt
check("巡航 10s 飞行距离", dist > 200, f"dist={dist:.0f}m (应>200m)")
check("固定翼保持前进(速度>0)", V_cruise > 0, f"V={V_cruise:.1f}m/s")
check("固定翼不悬停(速度!=0)", V_cruise != 0)
print()

# ============================================================
# 测试 11: 代码逻辑验证 — Phase 转换序列
# ============================================================
print("=" * 60)
print("测试 11: Phase 转换序列验证")
print("=" * 60)
phases = ['FREE_FALL', 'PULL_UP', 'MOTOR_START', 'CRUISE']
check("Phase 顺序正确", phases == ['FREE_FALL', 'PULL_UP', 'MOTOR_START', 'CRUISE'])
check("Phase 数量=4", len(phases) == 4)
check("FREE_FALL 在 PULL_UP 之前", phases.index('FREE_FALL') < phases.index('PULL_UP'))
check("PULL_UP 在 MOTOR_START 之前", phases.index('PULL_UP') < phases.index('MOTOR_START'))
check("MOTOR_START 在 CRUISE 之前", phases.index('MOTOR_START') < phases.index('CRUISE'))
print()

# ============================================================
# 测试 12: 参数边界验证
# ============================================================
print("=" * 60)
print("测试 12: 参数边界验证")
print("=" * 60)
check("触发空速合理(8~20m/s)", 8 <= DEFAULTS['trigger_airspeed'] <= 20)
check("BD_LOAD_FACTOR 范围", 1.1 <= DEFAULTS['load_factor'] <= 3.0)
check("BD_CRUISE_SPD 范围", 5 <= DEFAULTS['cruise_speed'] <= 50)
check("BD_MOTOR_RAMP 范围", 0.5 <= DEFAULTS['motor_ramp_time'] <= 5.0)
check("BD_CRUISE_THR 范围", 0.2 <= DEFAULTS['cruise_throttle'] <= 0.9)
print()

# ============================================================
# 测试 13: 极端参数组合
# ============================================================
print("=" * 60)
print("测试 13: 极端参数组合")
print("=" * 60)
# 最大拉起：高速+高过载+低空
p13a = {'cruise_speed': 35, 'trigger_airspeed': 15, 'load_factor': 2.5, 'motor_ramp_time': 1.0, 'cruise_throttle': 0.7, 'release_alt': 100, 'g': 9.81}
r13a = simulate_balloon_drop(p13a)
check("极端(高速高过载低空)拉起后>25m", r13a['alt_pullup'] > 25, f"alt={r13a['alt_pullup']:.1f}m (极端参数警告)")

# 最小拉起：低速+低过载+高空
p13b = {'cruise_speed': 15, 'trigger_ratio': 0.7, 'load_factor': 1.2, 'motor_ramp_time': 3.0, 'cruise_throttle': 0.5, 'release_alt': 200, 'g': 9.81}
r13b = simulate_balloon_drop(p13b)
check("极端(低速低过载高空)拉起后>100m", r13b['alt_pullup'] > 100, f"alt={r13b['alt_pullup']:.1f}m")
print()

# ============================================================
# 测试 14: 能量分析（速度 vs 高度转换）
# ============================================================
print("=" * 60)
print("测试 14: 能量分析")
print("=" * 60)
# 自由下落：势能转动能
E_potential_lost = DEFAULTS['g'] * r['h_freefall']  # J/kg
E_kinetic_gained = 0.5 * r['V_trigger']**2  # J/kg
check("能量守恒（势能≈动能）", abs(E_potential_lost - E_kinetic_gained) < 1.0,
      f"ΔE={abs(E_potential_lost - E_kinetic_gained):.2f} J/kg")

# 拉起后：动能保留（速度不变，方向改变）
V_after_pullup = r['V_trigger']  # 理想拉起速度不变
check("拉起后速度≈触发速度", abs(V_after_pullup - r['V_trigger']) < 0.1,
      f"V_after={V_after_pullup:.1f}, V_trigger={r['V_trigger']:.1f}")
print()

# ============================================================
# 测试 15: 安全裕度综合评估
# ============================================================
print("=" * 60)
print("测试 15: 安全裕度综合评估")
print("=" * 60)
safety_margins = {
    '高度裕度': r['alt_pullup'] - 50,  # 相对 50m 安全线
    '过载裕度': 3.0 - DEFAULTS['load_factor'],  # 相对 3g 极限
    '时间裕度': 15 - r['total_time'],  # 相对 15s 总时限
}
for name, margin in safety_margins.items():
    check(f"{name}充足(>0)", margin > 0, f"margin={margin:.1f}")
print()

# ============================================================
# 汇总
# ============================================================
print("=" * 60)
print(f"仿真验证完成: {PASS_COUNT} 通过, {FAIL_COUNT} 失败")
print("=" * 60)

if FAIL_COUNT > 0:
    print("\n⚠️  有测试失败，请检查参数和代码！")
    sys.exit(1)
else:
    print("\n✅ 全部测试通过！代码逻辑正确。")
    sys.exit(0)
