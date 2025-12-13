import time


def task_imu(shares):
    """IMU task: periodically update heading and yaw-rate shares."""
    (imu, heading_deg, yaw_rate_dps) = shares

    # Optional: short delay so fusion mode stabilizes
    next_ok = time.ticks_add(time.ticks_ms(), 200)
    while time.ticks_diff(time.ticks_ms(), next_ok) < 0:
        yield 0

    while True:
        try:
            # Read fused heading and yaw rate from BNO055
            h = imu.heading()      # degrees
            wz = imu.yaw_rate()    # deg/s
            heading_deg.put(h)
            yaw_rate_dps.put(wz)
        except Exception:
            # Ignore transient read failures
            pass

        # Let scheduler control timing
        yield 0
