"""Test mission upload then download"""
import sys, time
sys.path.insert(0, 'D:/oezcon/ai_mp')
from ai_mp2 import FullMAVLink

mav = FullMAVLink('COM15', 115200)
mav.connect()

# Upload mission
mgr = mav.mission_init()
mgr.clear()

test_wps = [
    ('TAKEOFF', 39.9042, 116.4074, 100, 10, 0, 0, 0),
    ('WAYPOINT', 39.9050, 116.4080, 100, 0, 0, 50, 0),
    ('WAYPOINT', 39.9060, 116.4090, 80, 0, 0, 30, 0),
    ('LOITER_TIME', 39.9055, 116.4085, 90, 30, 0, 50, 0),
    ('RTL', 0, 0, 0, 0, 0, 0, 0),
]

for cmd, lat, lon, alt, p1, p2, p3, p4 in test_wps:
    mgr.add_waypoint(cmd, lat, lon, alt, p1, p2, p3, p4)

print("\n=== Upload ===")
mgr.upload_to_fc()

print("\n=== Wait 2s ===")
time.sleep(2)

print("\n=== Download ===")
download_ok = mgr.download_from_fc()

if mgr.waypoints:
    print(f"\nDownloaded {len(mgr.waypoints)} waypoints:")
    for i, wp in enumerate(mgr.waypoints):
        print(f"  #{i+1} {wp['cmd']} Lat={wp.get('lat',0):.4f} Lon={wp.get('lon',0):.4f} Alt={wp.get('alt',0):.1f}")
    
    # Compare with uploaded
    print("\n=== Comparison ===")
    for i, (cmd, lat, lon, alt, *_) in enumerate(test_wps):
        if i < len(mgr.waypoints):
            wp = mgr.waypoints[i]
            cmd_ok = wp['cmd'] == cmd
            lat_ok = abs(wp.get('lat', 0) - lat) < 0.0001
            lon_ok = abs(wp.get('lon', 0) - lon) < 0.0001
            ok = cmd_ok and lat_ok and lon_ok
            icon = "OK" if ok else "DIFF"
            print(f"  WP#{i+1}: {cmd} {'==' if ok else '!='} {wp['cmd']} Lat={wp.get('lat',0):.4f} {'==' if lat_ok else '!='} {lat:.4f}")
else:
    print("Download returned no waypoints")

mav.close()
