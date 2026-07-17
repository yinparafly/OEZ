import math

b = 2.0
V_cruise = 22.0
V_stall = 15.0
rho = 1.225
g = 9.81

S = 0.55
CLmax = 1.0
m = 0.5 * rho * S * CLmax * V_stall**2 / g
AR = b**2 / S
WS = m * g / S
CLa = 2 * math.pi / (1 + 2/AR)
CLa_deg = math.degrees(CLa)
oswald = 0.85
CL_cruise = 2 * m * g / (rho * V_cruise**2 * S)
alpha_stall = math.degrees(CLmax / CLa)
CDi_cruise = CL_cruise**2 / (math.pi * oswald * AR)
CDp = CDi_cruise * 0.8

print("=" * 55)
print("飞机参数推导结果")
print("=" * 55)
print(f"翼面积 S       = {S} m2")
print(f"质量 m          = {m:.2f} kg")
print(f"展弦比 AR       = {AR:.2f}")
print(f"翼载荷 W/S      = {WS:.1f} N/m2")
print(f"升力线斜率 CLa  = {CLa:.3f}/rad = {CLa_deg:.2f}/deg")
print(f"零迎角升力 CL0  = 0.30")
print(f"最大升力系数    = {CLmax}")
print(f"失速迎角        = {alpha_stall:.1f} deg")
print(f"Oswald效率      = {oswald}")
print(f"寄生阻力系数    = {CDp:.4f}")
print(f"")
print(f"--- 飞行包线 ---")
print(f"失速速度        = {V_stall} m/s ({V_stall*3.6:.0f} km/h)")
print(f"巡航速度        = {V_cruise} m/s ({V_cruise*3.6:.0f} km/h)")
print(f"最大速度(估)    = {V_cruise*1.3:.0f} m/s ({V_cruise*1.3*3.6:.0f} km/h)")
print(f"")
print(f"--- 空投参数 ---")
print(f"俯冲释放俯仰角  = -90 deg")
print(f"PTCH_LIM_MIN    = -90")
print(f"Lua触发空速(70%)= {0.7*V_cruise:.1f} m/s")
print(f"拉起过载        = 2.0g")
print(f"拉起角速度      = {(2.0-1)*g/(0.7*V_cruise):.3f} rad/s = {math.degrees((2.0-1)*g/(0.7*V_cruise)):.1f} deg/s")
print(f"拉起半径        = {(0.7*V_cruise)**2/((2.0-1)*g):.1f} m")
print(f"拉起时间(90)    = {math.pi/2/((2.0-1)*g/(0.7*V_cruise)):.2f} s")
