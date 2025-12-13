import math


def observer_task(shares):
    """Standalone observer task (similar to version embedded in main.py)."""
    (LEFT, RIGHT, Lvel_sh, Rvel_sh,
     heading_deg, yaw_rate_dps,
     xhat_omegaL, xhat_omegaR,
     xhat_s, xhat_psi, xhat_psidot) = shares

    dt = 0.02
    t = 0.0
    x_omegaL = 0.0
    x_omegaR = 0.0
    x_s = 0.0
    x_psi = 0.0

    RAD_PER_COUNT = (2 * math.pi) / 1444.0

    # Wheel and motor parameters
    r = 0.035
    L = 0.141
    tau = 0.1
    K = 250 * 2 * math.pi / 60 / 4.5

    a = -1.0 / tau
    b = K / tau

    # Observer gains
    L1 = 16.0
    L2 = 16.0
    L3 = 10.0
    L4 = 4.0

    initiated = False

    def RK4_solver(x, dt, f1, f2, f3, f4):
        """Scalar RK4 integration helper."""
        return x + (dt / 6.0) * (f1 + 2 * f2 + 2 * f3 + f4)

    def wrap_pi(x):
        """Wrap angle into [-pi, pi]."""
        return (x + math.pi) % (2 * math.pi) - math.pi

    while True:
        vbat = 7.4

        # Approximate motor voltages from last efforts
        if hasattr(LEFT, 'last_effort'):
            uL = (LEFT.last_effort / 100.0) * vbat
        else:
            uL = 0.0

        if hasattr(RIGHT, 'last_effort'):
            uR = (RIGHT.last_effort / 100.0) * vbat
        else:
            uR = 0.0

        # Measured wheel speeds [rad/s]
        omegaL_meas = float(Lvel_sh.read())
        omegaR_meas = float(Rvel_sh.read())

        # IMU heading + yaw rate [rad, rad/s]
        psi_meas = math.radians(heading_deg.read())
        psidot_meas = math.radians(yaw_rate_dps.read())

        # Encoder angles to distance (assumes ENCL/ENCR exist in global scope)
        thetaL = ENCL.get_position() * RAD_PER_COUNT
        thetaR = ENCR.get_position() * RAD_PER_COUNT
        s_meas = (r / 2) * (thetaL + thetaR)

        # Initialize observer states once from measurements
        if not initiated:
            x_omegaL = omegaL_meas
            x_omegaR = omegaR_meas
            x_s = s_meas
            x_psi = psi_meas
            initiated = True

        # Innovations
        psidot_hat = (r / L) * (x_omegaR - x_omegaL)

        e0 = omegaL_meas - x_omegaL
        e1 = omegaR_meas - x_omegaR
        e_s = s_meas - x_s
        e_psi = wrap_pi(psi_meas - x_psi)
        e_psidot = psidot_meas - psidot_hat

        # RK4 for left wheel speed
        f1 = a * x_omegaL + b * uL + L1 * e0
        f2 = a * (x_omegaL + 0.5 * dt * f1) + b * uL + L1 * e0
        f3 = a * (x_omegaL + 0.5 * dt * f2) + b * uL + L1 * e0
        f4 = a * (x_omegaL + dt * f3) + b * uL + L1 * e0
        x_omegaL = RK4_solver(x_omegaL, dt, f1, f2, f3, f4)

        # RK4 for right wheel speed
        f1 = a * x_omegaR + b * uR + L2 * e1
        f2 = a * (x_omegaR + 0.5 * dt * f1) + b * uR + L2 * e1
        f3 = a * (x_omegaR + 0.5 * dt * f2) + b * uR + L2 * e1
        f4 = a * (x_omegaR + dt * f3) + b * uR + L2 * e1
        x_omegaR = RK4_solver(x_omegaR, dt, f1, f2, f3, f4)

        # Forward distance estimate
        v_s = (r / 2) * (x_omegaL + x_omegaR)
        f1 = v_s + L3 * e_s
        f2 = ((r / 2) * (x_omegaL + x_omegaR) + L3 * e_s)
        f3 = ((r / 2) * (x_omegaL + x_omegaR) + L3 * e_s)
        f4 = ((r / 2) * (x_omegaL + x_omegaR) + L3 * e_s)
        x_s = RK4_solver(x_s, dt, f1, f2, f3, f4)

        # Heading estimate
        yaw_dot = (r / L) * (x_omegaR - x_omegaL)
        f1 = yaw_dot + L4 * e_psidot
        f2 = yaw_dot + L4 * e_psidot
        f3 = yaw_dot + L4 * e_psidot
        f4 = yaw_dot + L4 * e_psidot
        x_psi = wrap_pi(RK4_solver(x_psi, dt, f1, f2, f3, f4))

        # Publish observer outputs
        xhat_omegaL.write(x_omegaL)
        xhat_omegaR.write(x_omegaR)
        xhat_s.write(x_s)
        xhat_psi.write(x_psi)
        xhat_psidot.write((r / L) * (x_omegaR - x_omegaL))

        yield 0
