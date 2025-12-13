import utime as time
import math
from pyb import Pin, Timer, I2C
import cotask
import task_share
import utime
from ulab import numpy as np
from motor import Motor
from encoder import Encoder
from controller_pid import PIDController, voltage_to_duty  # add this file to /flash
from LineSensor import LineSensorArray
import sys
from bno055 import BNO055
from bump_sensor import BumpBoard

# -----------------------------------------------------------------------------
# CONFIGURATION: Timers, pins, and encoder setup
# -----------------------------------------------------------------------------

# PWM ~20 kHz; change Timer/Channel and pins according to wiring
PWM_TIM_L = Timer(1, freq=20000); PWM_CH_L = 1
PWM_TIM_R = Timer(2, freq=20000); PWM_CH_R = 1
PIN_PWM_L = Pin('PA8'); PIN_DIR_L = Pin.cpu.C0; PIN_NS_L = Pin.cpu.A15
PIN_PWM_R = Pin('PA0'); PIN_DIR_R = Pin.cpu.C1; PIN_NS_R = Pin.cpu.A4

# Quadrature encoders (hardware timers in encoder mode)
ENC_TIM_L = Timer(4, prescaler=0, period=65535)
PIN_ENC_LA = Pin.cpu.B6; PIN_ENC_LB = Pin.cpu.B7
ENC_AR_L = 65535

ENC_TIM_R = Timer(3, prescaler=0, period=65535)  # PA6/PA7 are TIM3 CH1/CH2
PIN_ENC_RA = Pin.cpu.A6; PIN_ENC_RB = Pin.cpu.A7
ENC_AR_R = 65535

# -----------------------------------------------------------------------------
# Task periods (ms)
# -----------------------------------------------------------------------------
UI_MS     = 50
CTRL_MS   = 10        # PID control period
ENC_MS    = 10        # Encoder update
LINE_MS   = 5         # Line sensor
CSV_MS    = 20        # Logging / debug

# Encoder signs (flip here if wheel directions are swapped)
ENC_SIGN_L = 1
ENC_SIGN_R = 1

# Encoder counts → radians conversion
RAD_PER_COUNT = (2 * math.pi) / 1444.0

# Serial over Bluetooth (HC-05/06) or UART3; mirrors to REPL
# NOTE: requires 'pyb.UART' available on this board
bt = pyb.UART(3, 115200)


def _rdline():
    """Read one line from Bluetooth UART if available."""
    try:
        if bt.any():
            return bt.readline()
    except Exception:
        return None


def _print(s=""):
    """Print to both Bluetooth UART and normal REPL."""
    try:
        bt.write((str(s) + "\r\n").encode())
    except Exception:
        pass
    try:
        print(s)
    except Exception:
        pass


# -----------------------------------------------------------------------------
# Shared variables and state flags
# -----------------------------------------------------------------------------

# Basic control flags
go_flag   = task_share.Share('B', thread_protect=False, name="go")
done_flag = task_share.Share('B', thread_protect=False, name="done")
step_duty = task_share.Share('h', thread_protect=False, name="duty")   # -100..100
test_ms   = task_share.Share('I', thread_protect=False, name="test_ms")
state     = task_share.Share(
    'B', thread_protect=False, name="state"
)  # 0=IDLE,1=RUN,2=PIDRUN,3=CAL,4=FOLLOW,...

# PID mode shares and speed setpoints (counts/sec)
pid_mode  = task_share.Share('B', thread_protect=False, name="pid_mode")  # 0=pwm,1=pid
spL       = task_share.Share('i', thread_protect=False, name="spL")
spR       = task_share.Share('i', thread_protect=False, name="spR")

# Separate setpoints for follow vs map (current code mostly uses spL/spR)
spL_follow = task_share.Share('i', thread_protect=False, name='spL_follow')
spR_follow = task_share.Share('i', thread_protect=False, name='spR_follow')
spL_map    = task_share.Share('i', thread_protect=False, name='spL_map')
spR_map    = task_share.Share('i', thread_protect=False, name='spR_map')

# Wheel velocity PID gains
kp_sh = task_share.Share('f', thread_protect=False, name="kp")
ki_sh = task_share.Share('f', thread_protect=False, name="ki")
kd_sh = task_share.Share('f', thread_protect=False, name="kd")
ff_sh = task_share.Share('f', thread_protect=False, name="ff")         # volts per cps
alp_sh = task_share.Share('f', thread_protect=False, name="alpha")
ipd_sh = task_share.Share('B', thread_protect=False, name="ipd")       # 0/1

# Battery voltage (for volts → duty mapping)
vbat_sh = task_share.Share('f', thread_protect=False, name="vbat")
vbat_sh.put(7.4)  # default until ADC wired

# Measured wheel velocities [rad/s] from encoder_task
Lvel_sh  = task_share.Share('f', thread_protect=False, name="Lvel")
Rvel_sh  = task_share.Share('f', thread_protect=False, name="Rvel")

# Line following shares
line_error_sh = task_share.Share('f', thread_protect=False, name="line_err")
line_lost     = task_share.Share('B', thread_protect=False, name="line_lost")
fork_detected = task_share.Share('B', thread_protect=False, name="fork_detected")
right_branch_detected = task_share.Share('B', thread_protect=False, name="right_branch_detected")
left_branch_detected  = task_share.Share('B', thread_protect=False, name="left_branch_detected")

# Line controller gains and base speed
line_kp_sh       = task_share.Share('f', thread_protect=False, name="line_kp")
line_ki_sh       = task_share.Share('f', thread_protect=False, name="line_ki")
line_base_sp_sh  = task_share.Share('i', thread_protect=False, name="line_base")
line_kp_sh.put(100.0)
line_ki_sh.put(0.0)
line_base_sp_sh.put(350)

# IMU shares (heading and yaw rate)
heading_deg  = task_share.Share('f', thread_protect=False, name="heading_deg")
yaw_rate_dps = task_share.Share('f', thread_protect=False, name="yaw_rate_dps")

# Observer state estimates
xhat_omegaL    = task_share.Share('f', thread_protect=False, name="xhat_omegaL")
xhat_omegaR    = task_share.Share('f', thread_protect=False, name="xhat_omegaR")
xhat_s         = task_share.Share('f', thread_protect=False, name="xhat_s")
xhat_psi       = task_share.Share('f', thread_protect=False, name="xhat_psi")
xhat_psidot    = task_share.Share('f', thread_protect=False, name="xhat_psidot")
observer_debug = task_share.Share('B', thread_protect=False, name="observer_debug")
observer_debug_once = task_share.Share('B', thread_protect=False, name="observer_debug_once")

# Straight/turn motion flags and parameters
straight_flag   = task_share.Share('B', thread_protect=False, name="straight_flag")
turn_flag       = task_share.Share('B', thread_protect=False, name="turn_flag")
turn_left       = task_share.Share('f', thread_protect=False, name="turn_left")
turn_right      = task_share.Share('f', thread_protect=False, name="turn_right")
target_distance = task_share.Share('B', thread_protect=False, name="target_distance")
target_turn     = task_share.Share('B', thread_protect=False, name="target_turn")
motion_done     = task_share.Share('B', thread_protect=False, name="motion_done")
speed           = task_share.Share('B', thread_protect=False, name="Speed")

# Sensor pinlists
LINE_PINS = ['PB0', 'PB1', 'PA1', 'PC2', 'PC3']
BUMP_PINS = [("LEFT", "PA0"), ("RIGHT", "PA4")]  # adjust to real bump pins

# Map state and calibration flag
nav_state     = task_share.Share('B', thread_protect=False, name="nav_state")
line_cal_done = task_share.Share('B', thread_protect=False, name="line_cal_done")
line_cal_done.put(0)

# Line sensor array instance
sensors = LineSensorArray(LINE_PINS)
sensors.load_calibration()

# Logged CSV lists (used by test modes)
t_ms = []; L_pos = []; L_vel = []; R_pos = []; R_vel = []; U_cmd = []

# -----------------------------------------------------------------------------
# Hardware objects and controllers
# -----------------------------------------------------------------------------
LEFT  = Motor(PIN_PWM_L, PIN_DIR_L, PIN_NS_L, PWM_TIM_L, PWM_CH_L)
RIGHT = Motor(PIN_PWM_R, PIN_DIR_R, PIN_NS_R, PWM_TIM_R, PWM_CH_R)
ENCL  = Encoder(ENC_TIM_L, PIN_ENC_LA, PIN_ENC_LB, ENC_AR_L)
ENCR  = Encoder(ENC_TIM_R, PIN_ENC_RA, PIN_ENC_RB, ENC_AR_R)

# Wheel-velocity PID controllers (Volts out; duty computed later)
_pid_dt = CTRL_MS / 1000.0
kp = 0.1
ki = 0.002
kd = 0.001
pidL = PIDController(kp, ki, kd, out_max=12.0, delta_t=_pid_dt, alpha=0.8, ipd=1)
pidR = PIDController(kp, ki, kd, out_max=12.0, delta_t=_pid_dt, alpha=0.8, ipd=1)

# Basic robot geometry and motor model
r = 0.035   # wheel radius [m]
L = 0.141   # track width [m]
tau = 0.1   # motor time constant [s]
K = 250 * 2 * math.pi / 60 / 4.5  # speed gain [rad/s per V]

a = -1.0 / tau
b = K / tau

# Observer gains (tuned experimentally)
L1 = 16.0
L2 = 16.0
L3 = 10.0
L4 = 4.0

# -----------------------------------------------------------------------------
# BNO055 calibration helpers
# -----------------------------------------------------------------------------

def _calib_read_file(path="/flash/calibration.txt"):
    """Load calibration blob from hex file, if present."""
    try:
        with open(path, "r") as f:
            txt = f.read().strip()
        blob = bytes.fromhex(txt)
        return blob if len(blob) == 22 else None
    except Exception:
        return None


def _calib_write_file(blob, path="/flash/calibration.txt"):
    """Save calibration blob as hex text file."""
    try:
        with open(path, "w") as f:
            f.write(blob.hex())
        _print(f"[IMU] Saved calibration to {path}")
        return True
    except Exception as e:
        _print(f"[IMU] Save failed: {e}")
        return False


def _fmt_status(st):
    """Format BNO055 calibration status into short string."""
    return f"SYS:{st['sys']} G:{st['gyro']} A:{st['accel']} M:{st['mag']}"


def imu_bootstrap_calibration(imu, fusion_mode='NDOF', path="/flash/calibration.txt"):
    """Load IMU calibration from file or run a one-time manual calibration."""
    # Try file first
    blob = _calib_read_file(path)
    if blob:
        _print("[IMU] calibration.txt found; loading into BNO055…")
        try:
            imu.load_calibration(blob)  # driver handles CONFIG mode internally
            imu.set_mode(fusion_mode)
            st = imu.get_calibration_status()
            _print("[IMU] Calibration loaded. Status " + _fmt_status(st))
            return
        except Exception as e:
            _print(f"[IMU] Load failed ({e}); doing manual calibration…")

    # Manual calibration path
    _print("[IMU] No valid calibration file. Entering manual calibration.")
    _print("      Move Romi: slow figure-8s, gentle pitch/roll, and small rotations.")
    imu.set_mode(fusion_mode)

    # Poll until fully calibrated (3,3,3,3)
    last_report = 0
    while True:
        st = imu.get_calibration_status()
        now = utime.ticks_ms()
        if utime.ticks_diff(now, last_report) > 500:
            _print("[IMU] Cal status " + _fmt_status(st))
            last_report = now
        if st['sys'] == 3 and st['gyro'] == 3 and st['accel'] == 3 and st['mag'] == 3:
            break
        utime.sleep_ms(50)

    # Capture and persist coefficients
    blob = imu.dump_calibration()
    _calib_write_file(blob, path)
    _print("[IMU] Manual calibration complete and stored.")


def wrap_pi(x):
    """Wrap angle to [-pi, pi]."""
    return (x + math.pi) % (2 * math.pi) - math.pi


def observer_f(t, xhat, u, y):
    """(Unused) example continuous-time observer function with angle wrap."""
    Ax = np.dot(A, xhat)
    Bu = np.dot(B, u)
    yhat = np.dot(C, xhat)
    e = y - yhat
    # wrap the psi (3rd output) innovation to [-pi, pi]
    e[2, 0] = wrap_pi(float(e[2, 0]))
    Le = np.dot(L_obs, e)
    return Ax + Bu + Le


# -----------------------------------------------------------------------------
# IMU setup and calibration
# -----------------------------------------------------------------------------
try:
    i2c = I2C(1, I2C.CONTROLLER, baudrate=400_000)   # PB8=SCL, PB9=SDA
except AttributeError:
    i2c = I2C(1, I2C.MASTER, baudrate=400_000)

imu = BNO055(i2c, address=0x28)   # no hardware reset pin
imu_bootstrap_calibration(imu, fusion_mode='NDOF', path="/flash/calibration.txt")

# -----------------------------------------------------------------------------
# TASKS
# -----------------------------------------------------------------------------

def Motion_task():
    """Simple open-loop PWM step test for tuning/logging."""
    while True:
        if go_flag.read() and state.read() == 1:
            # Enable motors and zero encoders
            try:
                LEFT.enable(); RIGHT.enable()
                if hasattr(ENCL, 'zero'):
                    ENCL.zero()
                if hasattr(ENCR, 'zero'):
                    ENCR.zero()
            except Exception:
                pass

            # Apply fixed duty to both motors
            d = int(step_duty.read())
            try:
                LEFT.set_effort(d)
                RIGHT.set_effort(d)
            except Exception:
                pass

            # Hold for requested duration
            t0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t0) < int(test_ms.read()):
                yield 0

            # Stop and disable motors
            try:
                LEFT.set_effort(0); RIGHT.set_effort(0)
            except Exception:
                pass
            time.sleep_ms(200)
            try:
                LEFT.disable(); RIGHT.disable()
            except Exception:
                pass

            done_flag.write(1); go_flag.write(0)
        yield 0


def PID_task():
    """Closed-loop wheel velocity task.

    Uses Lvel_sh/Rvel_sh from Encoder_task and spL/spR setpoints.
    Runs whenever state ∈ {2,4,5,7}.
    """
    import gc

    started  = False   # True once motors enabled and logs cleared
    t0       = 0       # start time for logs (if used)
    last_dbg = time.ticks_ms()

    while True:
        st = state.read()
        # Active whenever in a PID-driven state
        running = (st == 2 and go_flag.read()) or (st == 4) or (st == 5) or (st == 7)

        if running == 1:
            ramp_step = 40  # (unused now; kept for reference)

            if not started:
                # Enable motors and zero encoders at start of a run
                try:
                    LEFT.enable(); RIGHT.enable()
                    if hasattr(ENCL, 'zero'):
                        ENCL.zero()
                    if hasattr(ENCR, 'zero'):
                        ENCR.zero()
                except Exception:
                    pass

                # Clear log buffers
                del t_ms[:]; del L_pos[:]; del L_vel[:]
                del R_pos[:]; del R_vel[:]; del U_cmd[:]
                t0 = time.ticks_ms()
                started = True

            # --- Measurements (rad/s) from encoder_task ---
            lv = float(Lvel_sh.read())
            rv = float(Rvel_sh.read())

            # --- Targets (counts/sec) from behavior tasks ---
            tgtL_cps = float(spL.read())
            tgtR_cps = float(spR.read())

            # Convert to rad/s if needed (tgt*_cps_rad kept for reference)
            tgtL_cps_rad = tgtL_cps * RAD_PER_COUNT
            tgtR_cps_rad = tgtR_cps * RAD_PER_COUNT

            # PID setpoints currently kept in cps units
            pidL.set_setpoint(tgtL_cps)
            pidR.set_setpoint(tgtR_cps)

            # Simple feedforward in V per cps
            ff = 0.01
            uL = pidL.update(lv, feedforward=ff * tgtL_cps)
            uR = pidR.update(rv, feedforward=ff * tgtR_cps)

            # Convert volts → duty [-100, 100]
            vbat = 7.4
            dutyL = 100.0 * (uL / vbat)
            dutyR = 100.0 * (uR / vbat)

            # Clamp to valid range
            if dutyL > 100: dutyL = 100
            if dutyL < -100: dutyL = -100
            if dutyR > 100: dutyR = 100
            if dutyR < -100: dutyR = -100

            # Apply efforts
            LEFT.enable(); RIGHT.enable()
            LEFT.set_effort(dutyL)
            RIGHT.set_effort(dutyR)

        else:
            # If we were running, cleanly stop motors once
            if started:
                try:
                    LEFT.set_effort(0); RIGHT.set_effort(0)
                    time.sleep_ms(150)
                    LEFT.disable(); RIGHT.disable()
                except Exception:
                    pass
                started = False
            yield 0

        # Periodic GC to keep heap healthy
        now = time.ticks_ms()
        if time.ticks_diff(now, last_dbg) >= 500:
            last_dbg = now
            gc.collect()

        yield 0


def Encoder_task():
    """Update encoder positions/velocities and feed shared variables.

    Also handles open-loop logging when state == 1.
    """
    started_ol = False     # open-loop logging flag
    last_lp = last_rp = 0  # last encoder counts
    last_us = time.ticks_us()
    t0_ms  = 0

    MAX_SAMPLES = 4096

    while True:
        # Always keep encoders updated
        if hasattr(ENCL, 'update'):
            ENCL.update()
        if hasattr(ENCR, 'update'):
            ENCR.update()

        # Current positions in counts (signed)
        lp = ENCL.get_position() * ENC_SIGN_L
        rp = ENCR.get_position() * ENC_SIGN_R

        # Time delta in microseconds
        now_us = time.ticks_us()
        dt = time.ticks_diff(now_us, last_us)
        if dt <= 0:
            dt = 1
        if dt > 50000:
            dt = 50000  # cap to avoid crazy spikes

        # Compute wheel velocities in rad/s
        lv = (lp - last_lp) * (1_000_000.0 / dt) * RAD_PER_COUNT
        rv = (rp - last_rp) * (1_000_000.0 / dt) * RAD_PER_COUNT

        # Simple low-pass filter on velocities
        alpha = 0.2
        lv_filt = (1 - alpha) * Lvel_sh.read() + alpha * lv
        rv_filt = (1 - alpha) * Rvel_sh.read() + alpha * rv
        Lvel_sh.write(lv_filt)
        Rvel_sh.write(rv_filt)

        last_lp = lp; last_rp = rp; last_us = now_us

        # Open-loop logging mode for state==1
        if state.read() == 1 and go_flag.read():
            if not started_ol:
                # Reset logs and zero encoders at start of test
                del t_ms[:]; del L_pos[:]; del L_vel[:]
                del R_pos[:]; del R_vel[:]; del U_cmd[:]
                if hasattr(ENCL, 'zero'):
                    ENCL.zero()
                if hasattr(ENCR, 'zero'):
                    ENCR.zero()
                last_lp = 0; last_rp = 0
                last_us = time.ticks_us()
                t0_ms = time.ticks_ms()
                started_ol = True

            # Append current sample
            t_ms.append(time.ticks_diff(time.ticks_ms(), t0_ms))
            L_pos.append(int(lp)); R_pos.append(int(rp))
            L_vel.append(float(lv)); R_vel.append(float(rv))
            U_cmd.append(int(step_duty.read()))

            # Stop once buffer is full
            if len(t_ms) >= MAX_SAMPLES:
                go_flag.write(0); done_flag.write(1)
        else:
            started_ol = False

        yield 0


def LineSensor_task():
    """Handle automatic line sensor calibration when state==3."""
    while True:
        if state.read() == 3:
            _print("# Auto line calibration starting (white)...")
            sensors.calibrate_white_all()
            time.sleep_ms(4000)

            _print("# Now calibrate black line...")
            sensors.calibrate_black_all()
            time.sleep_ms(4000)

            sensors.save_calibration()
            _print("# Calibration done. Entering follow.")
            line_cal_done.write(1)

            # Switch to line-follow state
            state.write(4)

        yield 0


def Follow_task():
    """Line-following behavior using centroid error → wheel speeds."""
    err_int = 0.0
    anti_windup = 2000.0

    e_prev  = 0.0
    e_filt  = 0.0
    last_us = time.ticks_us()

    # Line controller gains
    kp_line = 30.0
    ki_line = 0.02
    kd_line = 0.001

    base_sp = 20   # base speed [cps]
    max_sp  = 25   # max speed
    min_sp  = 5    # min speed to avoid stalling

    centroid_filt = 0.0
    alpha = 0.22   # unused here but kept for reference

    while True:
        if state.read() == 4:
            # Read normalized sensor values
            vals = sensors.read_all_norm()
            N = len(vals)
            centroid = sensors.get_centroid()

            # Compute normalized position error from raw values
            total = sum(vals)
            if total < 0.2:
                # No reliable line, zero error
                error = 0
            else:
                idx = sum(i * v for i, v in enumerate(vals)) / total
                mid = (N - 1) / 2
                error = (idx - mid) / mid  # map to roughly [-1, 1]

            # If using get_centroid() directly, error logic could go here
            # line_error_sh.write(error)

            # Derivative term (currently unused: kd_line=0)
            now_us = time.ticks_us()
            dt_us = time.ticks_diff(now_us, last_us) or 1
            de_dt = (e_filt - e_prev) * (1_000_000.0 / dt_us)
            e_prev = e_filt
            last_us = now_us

            kp_line = 30.0
            ki_line = 0.02
            kd_line = 0
            base_sp = 20

            # PI update with anti-windup
            err_int = max(min(err_int + error, anti_windup), -anti_windup)
            control = kp_line * error + ki_line * err_int

            # Turn by biasing wheel speeds
            balance = 2.0
            spL_val = base_sp + control
            spR_val = base_sp - balance - control

            # Saturate wheel speeds
            if spL_val > max_sp: spL_val = max_sp
            if spL_val < min_sp: spL_val = min_sp
            if spR_val > max_sp: spR_val = max_sp
            if spR_val < min_sp: spR_val = min_sp

            spL.write(int(spL_val))
            spR.write(int(spR_val))

            # Ensure PID mode is enabled
            pid_mode.write(1)

        else:
            # Reset integrator and flags when not following
            err_int = 0.0
            e_prev = 0.0
            centroid_filt = 0.0
            fork_detected.write(0)
            line_lost.write(0)

        yield 0


def task_imu():
    """Periodic IMU task that updates heading and yaw rate shares."""
    # Optional: small delay to let fusion settle at boot
    next_ok = time.ticks_add(time.ticks_ms(), 200)
    while time.ticks_diff(time.ticks_ms(), next_ok) < 0:
        yield 0

    while True:
        try:
            h  = imu.heading()   # degrees
            wz = imu.yaw_rate()  # deg/s
            heading_deg.put(h)
            yaw_rate_dps.put(wz)
        except Exception:
            # Keep running even if a read hiccups
            pass
        yield 0   # scheduler enforces actual period


def observer_task():
    """4-state observer for wheel speeds, distance, and heading."""
    dt = 0.02  # integration step [s]

    # State estimates
    x_omegaL = 0.0
    x_omegaR = 0.0
    x_s = 0.0
    x_psi = 0.0

    initiated = False

    def RK4_solver(x, dt, f1, f2, f3, f4):
        """Scalar RK4 integration step."""
        return x + (dt / 6.0) * (f1 + 2 * f2 + 2 * f3 + f4)

    while True:
        vbat = 7.4

        # Approximate motor voltages from last effort commands
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

        # Measured heading and yaw rate [rad, rad/s]
        psi_meas = math.radians(heading_deg.read())
        psidot_meas = math.radians(yaw_rate_dps.read())

        # Encoder angles → arc length [m]
        thetaL = ENCL.get_position() * RAD_PER_COUNT
        thetaR = ENCR.get_position() * RAD_PER_COUNT
        s_meas = (r / 2) * (thetaL + thetaR)

        # Initialize observer states from measurements on first run
        if not initiated:
            x_omegaL = omegaL_meas
            x_omegaR = omegaR_meas
            x_s = s_meas
            x_psi = psi_meas
            initiated = True

        # Innovations (measurement - estimate)
        psidot_hat = (r / L) * (x_omegaR - x_omegaL)

        e0 = omegaL_meas - x_omegaL
        e1 = omegaR_meas - x_omegaR
        e_s = s_meas - x_s
        e_psi = wrap_pi(psi_meas - x_psi)
        e_psidot = psidot_meas - psidot_hat

        # RK4 integration for left wheel speed
        f1 = a * x_omegaL + b * uL + L1 * e0
        f2 = a * (x_omegaL + 0.5 * dt * f1) + b * uL + L1 * e0
        f3 = a * (x_omegaL + 0.5 * dt * f2) + b * uL + L1 * e0
        f4 = a * (x_omegaL + dt * f3) + b * uL + L1 * e0
        x_omegaL = RK4_solver(x_omegaL, dt, f1, f2, f3, f4)

        # RK4 integration for right wheel speed
        f1 = a * x_omegaR + b * uL + L2 * e1
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

        # Publish estimates
        xhat_omegaL.write(x_omegaL)
        xhat_omegaR.write(x_omegaR)
        xhat_s.write(x_s)
        xhat_psi.write(x_psi)
        xhat_psidot.write((r / L) * (x_omegaR - x_omegaL))

        yield 0


def test_observer_task():
    """Test task: drive straight using observer distance estimate."""
    test_speed = 20
    test_turn_speed = 20
    overshoot = 0.03
    target_dist = 0.15 - overshoot  # [m]
    target_angle = math.radians(90)
    running = False
    start_s = 0.0
    start_psi = 0.0
    R = 0.035
    t_react = 0.30

    while True:
        if state.read() == 5:
            s_hat = xhat_s.read()
            psi_hat = xhat_psi.read()

            if not running:
                running = True
                start_s = s_hat
                go_flag.write(1)

            spL.write(test_speed)
            spR.write(test_speed)

            ds = abs(s_hat - start_s)

            _print("s_hat={:.3f}, start_s={:.3f}, ds={:.3f}".format(
                s_hat, start_s, abs(s_hat - start_s)
            ))

            if ds >= target_dist:
                # Stop when distance reached
                spL.write(0); spR.write(0)
                pid_mode.write(1)
                state.write(0)
                running = False
                nav_state.write(0)

        yield 0


def turn_observer_task():
    """Test task: turn in place using observer heading estimate."""
    test_turn_speed = 20
    target_angle = math.radians(90)
    running = False
    start_psi = 0.0
    R = 0.035

    while True:
        if state.read() == 6:
            psi_hat = xhat_psi.read()

            if not running:
                running = True
                start_psi = psi_hat
                go_flag.write(1)

            # Turn in place (L backward, R forward)
            spL.write(-test_turn_speed)
            spR.write(test_turn_speed)

            dpsi = wrap_pi(psi_hat - start_psi)
            _print("psi={:.3f}, dpsi={:.3f}".format(psi_hat, dpsi))

            if abs(dpsi) >= abs(target_angle):
                # Stop after 90°
                spL.write(0); spR.write(0)
                pid_mode.write(1)
                state.write(0)
                running = False
                nav_state.write(0)

            yield 0

        yield 0


def straight_turn_task():
    """General straight/turn motion primitive using observer."""
    test_speed = 20
    test_turn_speed = 20
    overshoot = 0.03
    target_angle = math.radians(90)
    running = False
    start_s = 0.0
    start_psi = 0.0
    R = 0.035
    t_react = 0.30

    while True:
        if state.read() == 7:
            s_hat = xhat_s.read()
            psi_hat = xhat_psi.read()

            if not running:
                running = True
                start_psi = psi_hat
                start_s = s_hat
                go_flag.write(1)
                motion_done.write(0)

            # ----------------- Straight motion -----------------
            if straight_flag.read():
                target_dist = target_distance.read() - overshoot
                spL.write(test_speed)
                spR.write(test_speed)

                ds = abs(s_hat - start_s)

                if ds >= target_dist:
                    # Reached straight-line distance
                    spL.write(0); spR.write(0)
                    motion_done.write(1)
                    turn_flag.write(0)
                    pid_mode.write(1)
                    state.write(3)  # change back to line following (per FSM)
                    running = False
                    straight_flag.write(0)

            # ----------------- Turn in place -------------------
            elif turn_flag.read():
                target_angle = math.radians(target_turn.read())

                # Direction from turn_left / turn_right flags
                if turn_left.read() == 1:
                    spL.write(-test_turn_speed)
                    spR.write(test_turn_speed)
                elif turn_right.read() == 1:
                    spL.write(test_turn_speed)
                    spR.write(-test_turn_speed)

                dpsi = wrap_pi(psi_hat - start_psi)

                if abs(dpsi) >= abs(target_angle):
                    # Finished turning
                    turn_flag.write(0)
                    straight_flag.write(0)
                    motion_done.write(1)
                    spL.write(0); spR.write(0)
                    pid_mode.write(1)
                    state.write(3)  # change back to following
                    running = False

        yield 0


def bump_task():
    """Bump recovery behavior using observer estimates.

    Sequence:
      1) back up 100 mm
      2) turn right 90°
      3) half-circle left with R = 112.5 mm
      4) stop and go idle
    """
    bumps = BumpBoard(BUMP_PINS)

    # Local bump FSM phases
    PH_IDLE       = 0
    PH_BACKUP     = 1
    PH_TURN_RIGHT = 2
    PH_ARC_LEFT   = 3
    PH_DONE       = 4

    phase = PH_IDLE

    # Saved state when phase starts
    s0   = 0.0
    psi0 = 0.0

    # Geometry / targets
    BACK_DIST = 0.10          # 100 mm in meters
    R_ARC     = 0.1125        # half-circle radius [m]
    ANG_90    = math.radians(90.0)
    ANG_180   = math.pi

    # Speeds in same units as line-follow setpoints
    BACK_SP   = 20
    TURN_SP   = 15
    ARC_SP_R  = 20            # right wheel speed for half-circle

    while True:
        # Always update bump sensors
        bumps.update()
        st = state.read()

        if phase == PH_IDLE:
            # Only react to bump while line following
            if st == 4:
                hits = bumps.hits()
                if hits:
                    _print("[BUMP] Hit on: {}".format(", ".join(hits)))

                    # Switch into bump maneuver state machine
                    state.write(7)

                    # Phase 1: back up 100 mm
                    s0 = float(xhat_s.read())
                    spL.write(-BACK_SP)
                    spR.write(-BACK_SP)
                    phase = PH_BACKUP

        elif phase == PH_BACKUP:
            # Use observer distance to check backup distance
            s = float(xhat_s.read())
            ds = abs(s - s0)

            if ds >= BACK_DIST - 0.005:
                # Stop, then prepare for right turn
                spL.write(0)
                spR.write(0)

                psi0 = float(xhat_psi.read())

                # Phase 2: right 90° pivot (L forward, R backward)
                spL.write(TURN_SP)
                spR.write(-TURN_SP)
                phase = PH_TURN_RIGHT

        elif phase == PH_TURN_RIGHT:
            psi = float(xhat_psi.read())
            dpsi = wrap_pi(psi - psi0)

            if abs(dpsi) >= ANG_90 - math.radians(3.0):
                # Stop, then set up half-circle left
                spL.write(0)
                spR.write(0)

                psi0 = float(xhat_psi.read())

                # Compute left wheel speed ratio for given radius
                # R = (L/2) * (vL + vR) / (vR - vL)
                # => alpha = vL/vR = (2R/L - 1)/(2R/L + 1)
                twoR_over_L = 2.0 * R_ARC / L
                alpha = (twoR_over_L - 1.0) / (twoR_over_L + 1.0)

                spR_val = ARC_SP_R
                spL_val = int(spR_val * alpha)
                if spL_val < 5:     # avoid very small speeds
                    spL_val = 5

                # Left-turn arc: both forward, right faster
                spL.write(spL_val)
                spR.write(spR_val)
                phase = PH_ARC_LEFT

        elif phase == PH_ARC_LEFT:
            psi = float(xhat_psi.read())
            dpsi = wrap_pi(psi - psi0)

            # Stop after about 180° of left-turn
            if abs(dpsi) >= ANG_180 - math.radians(5.0):
                spL.write(0)
                spR.write(0)

                # Make sure motors are fully stopped and disabled
                try:
                    LEFT.set_effort(0)
                    RIGHT.set_effort(0)
                    time.sleep_ms(150)
                    LEFT.disable()
                    RIGHT.disable()
                except Exception:
                    pass

                # Go back to idle
                state.write(0)
                phase = PH_DONE

        # After PH_DONE we sit idle and ignore further bumps
        yield 0


def map_task():
    """High-level map navigation FSM controlling straight/turn segments."""
    _print("task created")

    # Navigation state IDs
    NAV_START        = 0
    NAV_FORK1        = 2
    NAV_DIAMOND      = 3
    NAV_DOT_LINE     = 4
    NAV_FORK2        = 5
    NAV_TOP_CURVE    = 6
    NAV_MAZE         = 7
    NAV_BRIDGE       = 8
    NAV_BRIDGE_TURN  = 9
    NAV_BRIDGE_EXIT  = 10
    NAV_WALL         = 11
    NAV_TURN         = 12
    NAV_FINAL        = 13
    NAV_FINISH       = 14

    NAV_TEST_TURN    = 101  # unused test state

    st = NAV_START
    nav_state.write(st)

    s_start = 0.0
    bridge_entry_psi = 0.0
    bridge_start_s = 0.0

    while True:
        st = nav_state.read()
        psi_hat = xhat_psi.read()
        s_hat = xhat_s.read()

        if st == NAV_START:
            # Start with line sensor calibration
            line_cal_done.write(0)
            state.write(3)  # triggers LineSensor_task
            _print("[NAV] -> LINE_CAL")
            st = NAV_FORK1
            nav_state.write(st)

        elif st == NAV_FORK1:
            _print("[NAV] -> LINE_MAIN")
            # Simplified fork handling: always take right branch
            fork_detected.write(1)
            _print("[NAV] Fork Detected Turn Right")
            st = NAV_DIAMOND
            nav_state.write(st)

        elif st == NAV_DIAMOND:
            _print("[NAV] Crossing Diamond")
            st = NAV_DOT_LINE
            nav_state.write(st)

        elif st == NAV_DOT_LINE:
            # Straight observer-based segment after dot line
            straight_flag.write(1)
            target_distance.write(0.15)
            state.write(7)
            if motion_done.read() == 1:
                st = NAV_FORK2
                nav_state.write(st)

        elif st == NAV_FORK2:
            # Second fork, again choose right
            fork_detected.write(1)
            st = NAV_TOP_CURVE
            nav_state.write(st)

        elif st == NAV_TOP_CURVE:
            # Follow line along top curve
            st = NAV_MAZE
            nav_state.write(st)

        elif st == NAV_MAZE:
            # Straight into maze region
            straight_flag.write(1)
            target_distance.write(0.300)
            state.write(7)
            if motion_done.read() == 1:
                st = NAV_BRIDGE
                nav_state.write(st)

        elif st == NAV_BRIDGE:
            # Straight across bridge
            straight_flag.write(1)
            target_distance.write(0.625)
            state.write(7)
            if motion_done.read() == 1:
                st = NAV_BRIDGE_TURN
                nav_state.write(st)

        elif st == NAV_BRIDGE_TURN:
            # Turn at the end of bridge
            turn_flag.write(1)
            target_turn.write(90)
            state.write(7)
            if motion_done.read() == 1:
                st = NAV_BRIDGE_EXIT
                nav_state.write(st)

        elif st == NAV_BRIDGE_EXIT:
            # Small straight off the bridge
            straight_flag.write(1)
            target_distance.write(0.100)
            state.write(7)
            if motion_done.read() == 1:
                st = NAV_WALL
                nav_state.write(st)

        elif st == NAV_WALL:
            # Segment to bump the wall then back away (negative distance)
            straight_flag.write(1)
            target_distance.write(-0.150)
            state.write(7)
            if motion_done.read() == 1:
                st = NAV_TURN
                nav_state.write(st)

        elif st == NAV_TURN:
            # Final 90° turn toward finish circle
            turn_flag.write(1)
            target_turn.write(90)
            state.write(7)
            if motion_done.read() == 1:
                st = NAV_FINAL
                nav_state.write(st)

        elif st == NAV_FINAL:
            # Short final straight to finish circle
            straight_flag.write(1)
            target_distance.write(0.150)
            state.write(7)
            if motion_done.read() == 1:
                st = NAV_FINISH
                nav_state.write(st)

        elif st == NAV_FINISH:
            # Done with course
            go_flag.write(0)
            state.write(0)

        yield 0


def main():
    """Set up tasks and run the priority scheduler."""
    # Initialize control flags
    state.write(6)        # default state on boot (e.g. turn test)
    go_flag.write(0)
    line_cal_done.write(0)
    nav_state.write(0)

    # Register tasks with scheduler (priority, period in ms)
    # cotask.task_list.append(cotask.Task(Motion_task, name="Motion", priority=2, period=20, profile=False))
    cotask.task_list.append(cotask.Task(PID_task,       name="PID",       priority=5, period=20, profile=False))
    cotask.task_list.append(cotask.Task(Encoder_task,   name="Encoder",   priority=5, period=10, profile=False))
    cotask.task_list.append(cotask.Task(LineSensor_task,name="LineSens",  priority=3, period=30, profile=False))
    cotask.task_list.append(cotask.Task(Follow_task,    name="Follow",    priority=3, period=10, profile=False))
    cotask.task_list.append(cotask.Task(task_imu,       name="IMU",       priority=2, period=80, profile=False, trace=False))
    cotask.task_list.append(cotask.Task(observer_task,  name="Observer",  priority=3, period=20, profile=False))
    # cotask.task_list.append(cotask.Task(map_task, name="Map", priority=2, period=70, profile=False))
    cotask.task_list.append(cotask.Task(test_observer_task, name="TestObserver",   priority=2, period=50, profile=False))
    cotask.task_list.append(cotask.Task(turn_observer_task, name="TurnObserver",   priority=2, period=50, profile=False))
    cotask.task_list.append(cotask.Task(straight_turn_task, name="straight_turn",  priority=2, period=50, profile=False))
    cotask.task_list.append(cotask.Task(bump_task,          name="bump",           priority=3, period=10, profile=False))

    # Main scheduler loop
    while True:
        cotask.task_list.pri_sched()


if __name__ == "__main__":
    main()