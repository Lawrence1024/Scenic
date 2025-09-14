import time

# Placeholder: update once VEOS axes/yaw conventions are confirmed
# Scenic default: +Y forward, +X right, +Z up; yaw CCW (deg) from +Y
def scenic_to_dspace_xyz(p):
    return (p.x, p.y, getattr(p, "z", 0.0))
def scenic_to_dspace_ypr(orient):
    return (orient.yaw, orient.pitch, orient.roll)

def now_ms():
    return int(time.time() * 1000)
