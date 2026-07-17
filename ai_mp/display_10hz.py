#!/usr/bin/env python3
"""
ArduPlane SITL 10Hz Flight Display - PNG output
"""
import time, math, threading, os
from pymavlink import mavutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec
import numpy as np
from collections import deque

OUTPUT_DIR = '/mnt/d/oezcon/ai_mp/flight_frames'
os.makedirs(OUTPUT_DIR, exist_ok=True)

fd = {'mode':'MANUAL','aspd':0,'gspd':0,'alt':584,'hdg':353,'pitch':0,'roll':0,'thr':0,'armed':False,'lat':0,'lon':0,'bat':0}
trail_lat = deque(maxlen=300)
trail_lon = deque(maxlen=300)
trail_alt = deque(maxlen=300)
trail_t = deque(maxlen=300)
running = True

def mav_thread(m):
    global fd
    mode_map = {0:'MANUAL',1:'CIRCLE',2:'STABILIZE',7:'FBWA',8:'FBWB',10:'AUTO',13:'CRUISE',3:'RTL',9:'LAND',11:'TAKEOFF',4:'GUIDED'}
    while running:
        msg = m.recv_match(blocking=True, timeout=0.05)
        if not msg: continue
        t = msg.get_type()
        if t=='HEARTBEAT':
            fd['mode']=mode_map.get(msg.custom_mode,f'M{msg.custom_mode}')
            fd['armed']=bool(msg.base_mode&32)
        elif t=='VFR_HUD':
            fd['aspd']=msg.airspeed; fd['gspd']=msg.groundspeed
            fd['alt']=msg.alt; fd['hdg']=msg.heading; fd['thr']=msg.throttle
        elif t=='ATTITUDE':
            fd['pitch']=math.degrees(msg.pitch); fd['roll']=math.degrees(msg.roll)
        elif t=='GLOBAL_POSITION_INT':
            fd['lat']=msg.lat/1e7; fd['lon']=msg.lon/1e7
            trail_lat.append(fd['lat']); trail_lon.append(fd['lon'])
            trail_alt.append(fd['alt']); trail_t.append(time.time())
        elif t=='SYS_STATUS':
            fd['bat']=msg.voltage_battery/1000.0

def draw_frame(fig, frame_num, total_time):
    fig.clear()
    gs = GridSpec(2, 3, figure=fig, hspace=0.35, wspace=0.35, left=0.04, right=0.96, top=0.91, bottom=0.06)
    fig.patch.set_facecolor('#0d1117')
    fig.suptitle(f'ArduPlane SITL - {total_time:.1f}s', fontsize=11, color='#aaaaaa', fontweight='bold')

    ax1 = fig.add_subplot(gs[0, 0])
    ax1.set_xlim(-1.5,1.5); ax1.set_ylim(-1.5,1.5); ax1.set_aspect('equal'); ax1.axis('off')
    ax1.add_patch(patches.Rectangle((-2,0),4,2,color='#4A90D9'))
    ax1.add_patch(patches.Rectangle((-2,-2),4,2,color='#8B6914'))
    for p in range(-60,61,10):
        y=(p-fd['pitch'])/45.0
        if abs(y)<1.4:
            w=0.9 if p%20==0 else 0.4
            ax1.add_patch(patches.Rectangle((-w/2,y-0.01),w,0.02,color='white'))
    ax1.plot([-0.35,-0.12],[0,0],'w-',lw=2.5); ax1.plot([0.12,0.35],[0,0],'w-',lw=2.5)
    ax1.plot(0,0,'wo',markersize=6); ax1.plot(0,1.3,'w^',markersize=8)
    ax1.set_title('ATTITUDE',fontsize=9,color='white')

    ax2 = fig.add_subplot(gs[0, 1]); ax2.axis('off')
    items=[('AS',fd['aspd'],30,'#2196F3'),('GS',fd['gspd'],30,'#4CAF50'),('ALT',fd['alt'],1000,'#FF9800'),('THR',fd['thr'],100,'#E91E63')]
    for i,(label,val,mx,col) in enumerate(items):
        y=0.85-i*0.22
        norm=min(val/mx,1.0) if mx>0 else 0
        ax2.barh(y,norm,height=0.12,color=col,alpha=0.85)
        ax2.barh(y,1,height=0.12,color='gray',alpha=0.2,zorder=0)
        ax2.text(-0.05,y,f'{label}\n{val:.1f}',ha='right',va='center',fontsize=8,color='white')
    ax2.set_xlim(-0.4,1.2); ax2.set_ylim(-0.05,1.0)
    ax2.set_title('DATA',fontsize=9,color='white')

    ax3 = fig.add_subplot(gs[0, 2])
    if len(trail_lat)>1:
        lats=np.array(list(trail_lat)); lons=np.array(list(trail_lon)); alts=np.array(list(trail_alt))
        colors=plt.cm.plasma((alts-alts.min())/max(alts.max()-alts.min(),1))
        for i in range(1,len(lats)):
            ax3.plot([lons[i-1],lons[i]],[lats[i-1],lats[i]],color=colors[i],lw=1.5,alpha=0.8)
        ax3.plot(lons[-1],lats[-1],'r*',markersize=10); ax3.plot(lons[0],lats[0],'go',markersize=7)
    ax3.set_xlabel('Lon',fontsize=7,color='gray'); ax3.set_ylabel('Lat',fontsize=7,color='gray')
    ax3.set_title('TRACK',fontsize=9,color='white'); ax3.tick_params(colors='gray',labelsize=6); ax3.set_facecolor('#1a1a2e')

    ax4 = fig.add_subplot(gs[1, 0:2])
    if len(trail_t)>1:
        t_arr=np.array(list(trail_t))-trail_t[0]; a_arr=np.array(list(trail_alt))
        ax4.fill_between(t_arr,a_arr,alpha=0.35,color='#FF9800'); ax4.plot(t_arr,a_arr,color='#FF9800',lw=2)
        ax4.set_xlim(0,max(t_arr[-1],1))
    ax4.set_xlabel('Time (s)',fontsize=7,color='gray'); ax4.set_ylabel('Alt (m)',fontsize=7,color='gray')
    ax4.set_title('ALTITUDE',fontsize=9,color='white'); ax4.tick_params(colors='gray',labelsize=6); ax4.set_facecolor('#1a1a2e')

    ax5 = fig.add_subplot(gs[1, 2]); ax5.axis('off')
    fig.text(0.02,0.015,f"Mode: {fd['mode']} | Armed: {fd['armed']} | HDG: {fd['hdg']} | Pitch: {fd['pitch']:.1f} | Roll: {fd['roll']:.1f} | Thr: {fd['thr']}% | Bat: {fd['bat']:.1f}V | Frame: {frame_num}",
             fontsize=7.5, color='#888888', fontfamily='monospace')

def main():
    global running
    print("Connecting to SITL tcp:127.0.0.1:5760...")
    m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, dialect='ardupilotmega')
    hb = m.wait_heartbeat(timeout=15)
    if not hb: print("Failed!"); exit(1)
    print(f"Connected! sys={m.target_system}")
    m.mav.request_data_stream_send(m.target_system, m.target_component, 0, 50, 1)

    t = threading.Thread(target=mav_thread, args=(m,), daemon=True)
    t.start()

    fig = plt.figure(figsize=(16, 9), facecolor='#0d1117')
    print(f"Saving 10Hz frames to {OUTPUT_DIR}/")

    frame = 0
    t_start = time.time()
    try:
        while running and time.time()-t_start < 30:
            time.sleep(0.1)
            frame += 1
            elapsed = time.time() - t_start
            draw_frame(fig, frame, elapsed)
            fig.savefig(os.path.join(OUTPUT_DIR, f'frame_{frame:04d}.png'), dpi=80, facecolor='#0d1117')
            if frame % 10 == 0:
                print(f"  [{elapsed:5.1f}s] Frame {frame}: mode={fd['mode']} aspd={fd['aspd']:.1f} alt={fd['alt']:.1f}")
    except KeyboardInterrupt:
        pass

    running = False; m.close(); plt.close()
    print(f"Done! {frame} frames saved")

if __name__ == '__main__':
    main()
