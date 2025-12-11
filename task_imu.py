import time

def task_imu(shares):
    (imu, heading_deg, yaw_rate_dps) = shares
    # Optional: small delay to let fusion settle at boot
    next_ok = time.ticks_add(time.ticks_ms(), 200)
    while time.ticks_diff(time.ticks_ms(), next_ok) < 0:
        yield 0

    while True:
        try:
            h  = imu.heading()     # degrees
            wz = imu.yaw_rate()    # deg/s
            heading_deg.put(h)
            yaw_rate_dps.put(wz)
        except Exception as e:
            # Keep running even if a read hiccups
            pass
        yield 0   # scheduler will enforce task period    
