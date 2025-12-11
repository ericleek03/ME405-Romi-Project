import math
from controller import StraightLinePID
def straight_turn_task(shares):
    (encL, encR,state, go_flag, motion_done,straight_flag, turn_flag, target_distance, pid_mode) = shares 

    test_speed = 20
    test_turn_speed = 20
    overshoot = 0.03
    running = False

    # ---- Physical Constants ----
    R = 0.035            # wheel radius (m)
    W = 0.141             # track width (m)  ***SET THIS***
    TPR = 1440           # ticks per revolution ***SET THIS***

    # ---- Internal State ----
    start_s = 0.0
    start_psi = 0.0

    last_L = 0
    last_R = 0

    def get_ds_and_dpsi():
        nonlocal last_L, last_R
        
        # raw tick counts
        L_now = encL.read()
        R_now = encR.read()

        # delta ticks
        dL = L_now - last_L
        dR = R_now - last_R

        last_L = L_now
        last_R = R_now

        # convert ticks → radians
        dtheta_L = (2*math.pi) * (dL / TPR)
        dtheta_R = (2*math.pi) * (dR / TPR)

        # arc lengths
        ds_L = R * dtheta_L
        ds_R = R * dtheta_R

        # straight and angular components
        ds = 0.5 * (ds_L + ds_R)
        dpsi = (ds_R - ds_L) / W

        return ds, dpsi

    # ---- MAIN LOOP ----
    while True:

        if state.read() == 7:

            # initialize on first run
            if not running:
                running = True
                go_flag.write(1)
                motion_done.write(0)

                start_s = 0.0
                start_psi = 0.0

                last_L = encL.read()
                last_R = encR.read()

            # compute incremental motion
            ds, dpsi = get_ds_and_dpsi()
            start_s += ds
            start_psi += dpsi

            # ----------------------
            #   STRAIGHT MOTION
            # ----------------------
            if straight_flag.read():

                target_dist = target_distance.read() - overshoot
                spL.write(test_speed)
                spR.write(test_speed)

                if abs(start_s) >= target_dist:
                    spL.write(0); spR.write(0)
                    motion_done.write(1)
                    straight_flag.write(0)
                    turn_flag.write(0)
                    pid_mode.write(1)
                    running = False

            # ----------------------
            #        TURNING
            # ----------------------
            elif turn_flag.read():

                target_angle = math.radians(target_turn.read())

                if turn_left.read() == 1:
                    spL.write(-test_turn_speed)
                    spR.write(test_turn_speed)
                elif turn_right.read() == 1:
                    spL.write(test_turn_speed)
                    spR.write(-test_turn_speed)

                if abs(start_psi) >= abs(target_angle):
                    spL.write(0); spR.write(0)
                    motion_done.write(1)
                    straight_flag.write(0)
                    turn_flag.write(0)
                    pid_mode.write(1)
                    running = False

        yield 0
