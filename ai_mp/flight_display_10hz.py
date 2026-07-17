#!/usr/bin/env python3
"""
ArduPlane SITL 10Hz 实时飞行显示器
- 地平仪 (姿态指示器)
- 轨迹曲线 (3D飞行路径)
- 实时数据面板
- Balloon Drop 阶段指示
"""
import time
import math
import sys
import threading
from collections import deque
from pymavlink import mavutil
import matplotlib
matplotlib.use('Agg')  # 非交互后端
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec
import numpy as np

# 数据存储
MAX_TRAIL = 500
trail_lat = deque(maxlen=MAX_TRAIL)
trail_lon = deque(maxlen=MAX_TRAIL)
trail_alt = deque(maxlen=MAX_TRAIL)
trail_time = deque(maxlen=MAX_TRAIL)

# 飞行数据
fd = {
    'mode': 'UNKNOWN', 'airspeed': 0, 'groundspeed': 0,
    'alt': 0, 'heading': 0, 'pitch': 0, 'roll': 0,
    'throttle': 0, 'armed': False, 'lat': 0, 'lon': 0,
    'voltage': 0, 'current': 0, 'phase': 'IDLE',
    'yaw': 0, 'climb': 0, 'satellites': 0
}

running = True
data_ready = threading.Event()

def mavlink_thread(master):
    """MAVLink 接收线程 - 10Hz"""
    global fd
    mode_map = {0:'MANUAL',1:'CIRCLE',2:'STABILIZE',5:'TRAINING',
               7:'FBWA',8:'FBWB',10:'AUTO',13:'CRUISE',3:'RTL',9:'LAND',
               11:'TAKEOFF',4:'GUIDED'}

    while running:
        msg = master.recv_match(blocking=True, timeout=0.1)
        if not msg:
            continue

        mtype = msg.get_type()
        if mtype == 'HEARTBEAT':
            fd['mode'] = mode_map.get(msg.custom_mode, f'M{msg.custom_mode}')
            fd['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        elif mtype == 'VFR_HUD':
            fd['airspeed'] = msg.airspeed
            fd['groundspeed'] = msg.groundspeed
            fd['alt'] = msg.alt
            fd['heading'] = msg.heading
            fd['throttle'] = msg.throttle
            fd['climb'] = msg.climb
        elif mtype == 'ATTITUDE':
            fd['pitch'] = math.degrees(msg.pitch)
            fd['roll'] = math.degrees(msg.roll)
            fd['yaw'] = math.degrees(msg.yaw)
        elif mtype == 'GLOBAL_POSITION_INT':
            fd['lat'] = msg.lat / 1e7
            fd['lon'] = msg.lon / 1e7
            now = time.time()
            trail_lat.append(fd['lat'])
            trail_lon.append(fd['lon'])
            trail_alt.append(fd['alt'])
            trail_time.append(now)
        elif mtype == 'SYS_STATUS':
            fd['voltage'] = msg.voltage_battery / 1000.0
            fd['current'] = msg.current_battery / 100.0
        elif mtype == 'GPS_RAW_INT':
            fd['satellites'] = msg.satellites_visible

        data_ready.set()

def draw_horizon(ax, pitch, roll):
    """绘制地平仪"""
    ax.clear()
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect('equal')
    ax.axis('off')

    # 天空
    sky = patches.Rectangle((-2, 0), 4, 2, color='#4A90D9')
    ax.add_patch(sky)
    # 地面
    ground = patches.Rectangle((-2, -2), 4, 2, color='#8B6914')
    ax.add_patch(ground)

    # 地平线 (带滚转)
    angle = -roll
    pitch_offset = pitch / 45.0  # 缩放到视图

    # 绘制俯仰线
    for p in range(-40, 41, 10):
        y = (p - pitch) / 45.0
        if abs(y) < 1.4:
            w = 0.8 if p % 20 == 0 else 0.4
            line = patches.Rectangle((-w/2, y-0.01), w, 0.02, color='white')
            ax.add_patch(line)

    # 中心标记
    ax.plot([-0.3, -0.1], [0, 0], 'w-', lw=2)
    ax.plot([0.1, 0.3], [0, 0], 'w-', lw=2)
    ax.plot([0, 0], [-0.05, 0.05], 'w-', lw=2)
    ax.plot(0, 0, 'wo', markersize=5)

    # 滚转指示器
    ax.plot(0, 1.3, 'w^', markersize=8)
    ax.plot([-0.15, 0.15], [1.2, 1.2], 'w-', lw=2)

    # 标题
    ax.set_title('ATTITUDE', fontsize=10, color='white', fontweight='bold')

def draw_speed_alt(ax, aspd, gspd, alt, climb):
    """绘制速度高度条"""
    ax.clear()
    ax.axis('off')

    # 空速条
    aspd_norm = min(aspd / 30.0, 1.0)
    ax.barh(0.8, aspd_norm, height=0.15, color='#2196F3', alpha=0.8)
    ax.barh(0.8, 1.0, height=0.15, color='gray', alpha=0.3, zorder=0)
    ax.text(-0.05, 0.8, f'AS\n{aspd:.1f}', ha='right', va='center', fontsize=8, color='white')
    ax.text(1.05, 0.8, 'm/s', ha='left', va='center', fontsize=7, color='gray')

    # 地速
    gspd_norm = min(gspd / 30.0, 1.0)
    ax.barh(0.55, gspd_norm, height=0.15, color='#4CAF50', alpha=0.8)
    ax.barh(0.55, 1.0, height=0.15, color='gray', alpha=0.3, zorder=0)
    ax.text(-0.05, 0.55, f'GS\n{gspd:.1f}', ha='right', va='center', fontsize=8, color='white')

    # 高度
    alt_norm = min(alt / 1000.0, 1.0)
    ax.barh(0.3, alt_norm, height=0.15, color='#FF9800', alpha=0.8)
    ax.barh(0.3, 1.0, height=0.15, color='gray', alpha=0.3, zorder=0)
    ax.text(-0.05, 0.3, f'ALT\n{alt:.0f}', ha='right', va='center', fontsize=8, color='white')
    ax.text(1.05, 0.3, 'm', ha='left', va='center', fontsize=7, color='gray')

    # 爬升率
    climb_color = '#4CAF50' if climb >= 0 else '#F44336'
    ax.barh(0.05, min(abs(climb)/10, 1.0) * (1 if climb >= 0 else -1),
            height=0.15, color=climb_color, alpha=0.8)
    ax.text(-0.05, 0.05, f'VS\n{climb:+.1f}', ha='right', va='center', fontsize=8, color='white')

    ax.set_xlim(-0.5, 1.3)
    ax.set_ylim(-0.15, 1.0)
    ax.set_title('FLIGHT DATA', fontsize=10, color='white', fontweight='bold')

def draw_trajectory(ax, lats, lons, alts):
    """绘制3D轨迹曲线"""
    ax.clear()
    if len(lats) > 1:
        lats_arr = np.array(lats)
        lons_arr = np.array(lons)
        alts_arr = np.array(alts)

        # 颜色按高度映射
        colors = plt.cm.viridis((alts_arr - alts_arr.min()) /
                                 max(alts_arr.max() - alts_arr.min(), 1))

        # 绘制轨迹线
        for i in range(1, len(lats_arr)):
            ax.plot([lons_arr[i-1], lons_arr[i]],
                   [lats_arr[i-1], lats_arr[i]],
                   color=colors[i], linewidth=1.5, alpha=0.8)

        # 当前位置
        ax.plot(lons_arr[-1], lats_arr[-1], 'r*', markersize=12, label='Current')

        # 起点
        ax.plot(lons_arr[0], lats_arr[0], 'go', markersize=8, label='Start')

        ax.legend(loc='upper right', fontsize=7, facecolor='#333', edgecolor='gray',
                 labelcolor='white')

    ax.set_xlabel('Longitude', fontsize=8, color='white')
    ax.set_ylabel('Latitude', fontsize=8, color='white')
    ax.set_title('TRAJECTORY', fontsize=10, color='white', fontweight='bold')
    ax.tick_params(colors='gray', labelsize=7)
    ax.set_facecolor('#1a1a2e')
    for spine in ax.spines.values():
        spine.set_color('gray')

def draw_altitude_profile(ax, times, alts):
    """绘制高度剖面"""
    ax.clear()
    if len(times) > 1:
        t_arr = np.array(times) - times[0]
        a_arr = np.array(alts)
        ax.fill_between(t_arr, a_arr, alpha=0.3, color='#FF9800')
        ax.plot(t_arr, a_arr, color='#FF9800', linewidth=1.5)

    ax.set_xlabel('Time (s)', fontsize=8, color='white')
    ax.set_ylabel('Alt (m)', fontsize=8, color='white')
    ax.set_title('ALTITUDE PROFILE', fontsize=10, color='white', fontweight='bold')
    ax.tick_params(colors='gray', labelsize=7)
    ax.set_facecolor('#1a1a2e')
    for spine in ax.spines.values():
        spine.set_color('gray')

def draw_phase_indicator(ax, phase):
    """绘制 Balloon Drop 阶段指示"""
    ax.clear()
    ax.axis('off')

    phases = ['FREE_FALL', 'PULL_UP', 'MOTOR_START', 'CRUISE']
    colors = ['#F44336', '#FF9800', '#4CAF50', '#2196F3']

    phase_idx = -1
    for i, p in enumerate(phases):
        if p in phase or (phase == 'LUA_LOADED' and i == 0):
            phase_idx = i
            break

    for i, (p, c) in enumerate(zip(phases, colors)):
        x = 0.1 + i * 0.22
        if i <= phase_idx:
            ax.add_patch(patches.FancyBboxPatch((x-0.08, 0.3), 0.16, 0.4,
                         boxstyle="round,pad=0.02", facecolor=c, alpha=0.9))
            ax.text(x, 0.5, p, ha='center', va='center', fontsize=7,
                   color='white', fontweight='bold')
        else:
            ax.add_patch(patches.FancyBboxPatch((x-0.08, 0.3), 0.16, 0.4,
                         boxstyle="round,pad=0.02", facecolor='gray', alpha=0.3))
            ax.text(x, 0.5, p, ha='center', va='center', fontsize=7,
                   color='gray')

        if i < len(phases) - 1:
            arrow_x = x + 0.12
            ax.annotate('', xy=(arrow_x+0.04, 0.5), xytext=(arrow_x-0.02, 0.5),
                       arrowprops=dict(arrowstyle='->', color='gray', lw=1.5))

    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_title('BALLOON DROP PHASES', fontsize=10, color='white', fontweight='bold')

def draw_info_panel(ax, fd):
    """绘制信息面板"""
    ax.clear()
    ax.axis('off')

    info_lines = [
        f"Mode: {fd['mode']}",
        f"Armed: {'YES' if fd['armed'] else 'NO'}",
        f"Heading: {fd['heading']:.0f} deg",
        f"Throttle: {fd['throttle']:.0f}%",
        f"Battery: {fd['voltage']:.1f}V {fd['current']:.1f}A",
        f"GPS: {fd['satellites']} sats",
        f"Pos: {fd['lat']:.6f}, {fd['lon']:.6f}",
        f"Phase: {fd['phase']}",
    ]

    for i, line in enumerate(info_lines):
        ax.text(0.05, 0.95 - i*0.12, line, transform=ax.transAxes,
               fontsize=9, color='white', fontfamily='monospace',
               verticalalignment='top')

    ax.set_title('STATUS', fontsize=10, color='white', fontweight='bold')

def update_display(fig, axes):
    """更新所有图表"""
    horizon_ax, speed_ax, traj_ax, alt_ax, phase_ax, info_ax = axes

    draw_horizon(horizon_ax, fd['pitch'], fd['roll'])
    draw_speed_alt(speed_ax, fd['airspeed'], fd['groundspeed'], fd['alt'], fd['climb'])
    draw_trajectory(traj_ax, list(trail_lat), list(trail_lon), list(trail_alt))
    draw_altitude_profile(alt_ax, list(trail_time), list(trail_alt))
    draw_phase_indicator(phase_ax, fd['phase'])
    draw_info_panel(info_ax, fd)

    fig.canvas.draw()
    fig.canvas.flush_events()

def main():
    global running

    print("Connecting to SITL tcp:127.0.0.1:5760...")
    master = mavutil.mavlink_connection(
        'tcp:127.0.0.1:5760',
        source_system=255,
        dialect='ardupilotmega'
    )

    hb = master.wait_heartbeat(timeout=15)
    if not hb:
        print("Connection failed!")
        exit(1)

    print(f"Connected! sys={master.target_system}")
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1  # 50Hz for 10Hz display
    )

    # 启动 MAVLink 接收线程
    mav_thread = threading.Thread(target=mavlink_thread, args=(master,), daemon=True)
    mav_thread.start()

    # 创建图形界面
    plt.ion()
    fig = plt.figure(figsize=(16, 9), facecolor='#0d1117')
    fig.canvas.manager.set_window_title('ArduPlane SITL Flight Display - 10Hz')

    gs = GridSpec(3, 4, figure=fig, hspace=0.35, wspace=0.3,
                  left=0.03, right=0.97, top=0.93, bottom=0.05)

    horizon_ax = fig.add_subplot(gs[0:2, 0:2])
    speed_ax = fig.add_subplot(gs[0:2, 2])
    traj_ax = fig.add_subplot(gs[0:2, 3])
    alt_ax = fig.add_subplot(gs[2, 0:2])
    phase_ax = fig.add_subplot(gs[2, 2])
    info_ax = fig.add_subplot(gs[2, 3])

    axes = (horizon_ax, speed_ax, traj_ax, alt_ax, phase_ax, info_ax)

    for ax in axes:
        ax.set_facecolor('#0d1117')

    print("Display running at 10Hz. Press Ctrl+C to stop.")
    print("=" * 50)

    frame_count = 0
    t_start = time.time()

    try:
        while running:
            data_ready.wait(timeout=0.1)
            data_ready.clear()

            update_display(fig, axes)
            frame_count += 1

            # 每秒打印状态
            if frame_count % 10 == 0:
                elapsed = time.time() - t_start
                print(f"  [{elapsed:5.1f}s] mode={fd['mode']:10s} aspd={fd['airspeed']:5.1f} "
                      f"alt={fd['alt']:7.1f} hdg={fd['heading']:3.0f} "
                      f"pitch={fd['pitch']:6.1f} roll={fd['roll']:6.1f}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        running = False
        master.close()
        plt.ioff()
        plt.close()

if __name__ == '__main__':
    main()
