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
# ------------------ CONFIG (EDIT ME) ------------------
# PWM ~20 kHz; change Timer/Channel and pins according to wiring
PWM_TIM_L = Timer(1, freq=20000); PWM_CH_L = 1
PWM_TIM_R = Timer(2, freq=20000); PWM_CH_R = 1
PIN_PWM_L = Pin('PA8'); PIN_DIR_L = Pin.cpu.C0; PIN_NS_L = Pin.cpu.A15
PIN_PWM_R = Pin('PA0'); PIN_DIR_R = Pin.cpu.C1; PIN_NS_R = Pin.cpu.A4

ENC_TIM_L = Timer(4, prescaler=0, period=65535)
PIN_ENC_LA = Pin.cpu.B6; PIN_ENC_LB = Pin.cpu.B7
ENC_AR_L = 65535
 
ENC_TIM_R = Timer(3, prescaler=0, period=65535)  # PA6/PA7 are TIM3 CH1/CH2
PIN_ENC_RA = Pin.cpu.A6; PIN_ENC_RB = Pin.cpu.A7
ENC_AR_R = 65535
# ------------------ END CONFIG ------------------------

# Task periods (ms)
UI_MS     = 50
CTRL_MS   = 10       # PID control period
ENC_MS    = 10
LINE_MS =  5
CSV_MS    = 20

# Encoder signs
ENC_SIGN_L = 1
ENC_SIGN_R = 1

RAD_PER_COUNT = (2 * math.pi)/1444.0

# Serial over Bluetooth (HC-05/06) or UART3; mirrors to REPL
bt = pyb.UART(3, 115200)


def _rdline():
    try:
        if bt.any():
            return bt.readline()
    except Exception:
        return None

def _print(s=""):
    try: bt.write((str(s) + "\r\n").encode()) 
    except Exception: pass
    try: print(s)
    except Exception: pass

# ------------------ Shares & State ------------------
go_flag   = task_share.Share('B', thread_protect=False, name="go")
done_flag = task_share.Share('B', thread_protect=False, name="done")
step_duty = task_share.Share('h', thread_protect=False, name="duty")     # -100..100
test_ms   = task_share.Share('I', thread_protect=False, name="test_ms")
state     = task_share.Share('B', thread_protect=False, name="state")    # 0=IDLE,1=RUN,2=PIDRUN

# NEW: PID mode shares
pid_mode  = task_share.Share('B', thread_protect=False, name="pid_mode") # 0=pwm,1=pid
spL       = task_share.Share('i', thread_protect=False, name="spL")      # counts/sec
spR       = task_share.Share('i', thread_protect=False, name="spR")

spL_follow = task_share.Share('i', thread_protect = False, name = 'spL_follow')
spR_follow = task_share.Share('i', thread_protect = False, name = 'spR_follow')
spL_map = task_share.Share('i',thread_protect = False, name = 'spL_map')
spR_map = task_share.Share('i',thread_protect = False, name = 'spR_map')
   
kp_sh = task_share.Share('f', thread_protect=False, name="kp")
ki_sh = task_share.Share('f', thread_protect=False, name="ki")
kd_sh = task_share.Share('f', thread_protect=False, name="kd")
ff_sh = task_share.Share('f', thread_protect=False, name="ff")           # volts per (counts/sec)
alp_sh= task_share.Share('f', thread_protect=False, name="alpha")
ipd_sh= task_share.Share('B', thread_protect=False, name="ipd")          # 0/1

vbat_sh = task_share.Share('f', thread_protect=False, name="vbat")       # volts; user-updated (optional)
vbat_sh.put(7.4)  # default until you wire ADC

# For PID control we need current velocities
Lvel_sh  = task_share.Share('f', thread_protect=False, name="Lvel")
Rvel_sh  = task_share.Share('f', thread_protect=False, name="Rvel")

line_error_sh = task_share.Share('f', thread_protect=False, name ="line_err")
line_lost = task_share.Share('B',thread_protect = False, name = "line_lost")
fork_detected = task_share.Share('B',thread_protect = False, name = "fork_detected")
right_branch_detected = task_share.Share('B', thread_protect=False, name="right_branch_detected")
left_branch_detected = task_share.Share('B', thread_protect = False, name = "left_branch_detected")
line_kp_sh = task_share.Share('f',thread_protect=False,name="line_kp")
line_ki_sh = task_share.Share('f',thread_protect=False,name="line_ki")
line_base_sp_sh = task_share.Share('i',thread_protect=False,name="line_base")
line_kp_sh.put(100.0)
line_ki_sh.put(0.0)
line_base_sp_sh.put(350)

# IMU Shares
heading_deg   = task_share.Share('f', thread_protect=False, name="heading_deg")
yaw_rate_dps  = task_share.Share('f', thread_protect=False, name="yaw_rate_dps")

xhat_omegaL = task_share.Share('f',thread_protect=False,name="xhat_omegaL")
xhat_omegaR = task_share.Share('f',thread_protect=False,name="xhat_omegaR")
xhat_s = task_share.Share('f',thread_protect=False,name="xhat_s")
xhat_psi = task_share.Share('f',thread_protect=False,name="xhat_psi")
xhat_psidot = task_share.Share('f',thread_protect=False,name="xhat_psidot")
observer_debug = task_share.Share('B',thread_protect=False,name="observer_debug")
observer_debug_once = task_share.Share('B',thread_protect=False,name="observer_debug_once")

LINE_PINS = ['PB0','PB1','PA1','PC2','PC3'] # adjust based on pins




nav_state = task_share.Share('B',thread_protect = False, name = "nav_state")
line_cal_done = task_share.Share('B',thread_protect=False, name= "line_cal_done")
line_cal_done.put(0)

sensors = LineSensorArray(LINE_PINS)
sensors.load_calibration()
# Logged CSV lists (kept, same columns/order)
t_ms=[]; L_pos=[]; L_vel=[]; R_pos=[]; R_vel=[]; U_cmd=[]

# Hardware
LEFT  = Motor(PIN_PWM_L, PIN_DIR_L, PIN_NS_L, PWM_TIM_L, PWM_CH_L)
RIGHT = Motor(PIN_PWM_R, PIN_DIR_R, PIN_NS_R, PWM_TIM_R, PWM_CH_R)
ENCL  = Encoder(ENC_TIM_L, PIN_ENC_LA, PIN_ENC_LB, ENC_AR_L)
ENCR  = Encoder(ENC_TIM_R, PIN_ENC_RA, PIN_ENC_RB, ENC_AR_R)

# PID controllers (one per wheel). Output units = Volts; out_max will be clamped later by duty map.
# dt uses CTRL_MS; derivative low-pass via alpha (runtime settable).
_pid_dt = CTRL_MS / 1000.0
pidL = PIDController(kp=0.08, ki=0.002, kd=0.001, out_max=12.0, delta_t=_pid_dt, alpha=0.8, ipd=1)
pidR = PIDController(kp=0.08, ki=0.002, kd=0.001, out_max=12.0, delta_t=_pid_dt, alpha=0.8, ipd=1)

#Wheel Parameters
r = 0.035 # in meters
L = 0.141
tau = 0.1
K = 250*2*math.pi/60/4.5

a = -1.0/tau
b = K/tau
'''
A = np.array([
    [a, 0.0, 0.0, 0.0],
    [0.0, a, 0.0, 0.0],
    [r/2, r/2, 0.0, 0.0],
    [-r/L, r/L, 0.0, 0.0]
])
B = np.array([
    [b, 0.0],
    [0.0, b],
    [0.0, 0.0],
    [0.0, 0.0]
])

C = np.array([
    [1.0, 0.0, 0, 0],
    [0.0, 1.0, 0, 0],
    [0.0, 0.0, 0.0, 1.0],
    [-r/L, r/L, 0.0, 0.0]
])

'''
L1 = 16.0
L2 = 16.0
L3 = 6.0
L4 = 4.0
#observer gain A-L*C adjust these parameters'''
'''
L_obs = np.array([
    [16.0, 0.0, 0.0, 0.0],
    [0.0, 16.0, 0.0, 0.0],
    [0.0, 0.0, 6.0, 0.0],
    [0.0, 0.0, 0.0, 4.0]
])'''
'''
A_L = A-np.dot(L_obs,C)

'''


# ---- BNO055 calibration helpers ----
def _calib_read_file(path="/flash/calibration.txt"):
    try:
        with open(path, "r") as f:
            txt = f.read().strip()
        blob = bytes.fromhex(txt)
        return blob if len(blob) == 22 else None
    except Exception:
        return None

def _calib_write_file(blob, path="/flash/calibration.txt"):
    try:
        with open(path, "w") as f:
            f.write(blob.hex())
        _print(f"[IMU] Saved calibration to {path}")
        return True
    except Exception as e:
        _print(f"[IMU] Save failed: {e}")
        return False

def _fmt_status(st):
    return f"SYS:{st['sys']} G:{st['gyro']} A:{st['accel']} M:{st['mag']}"

def imu_bootstrap_calibration(imu, fusion_mode='NDOF', path="/flash/calibration.txt"):
    """Implements the flowchart: load file if present else interactively calibrate once."""
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
    return (x + math.pi) % (2*math.pi) - math.pi

def observer_f(t,xhat,u,y):
    # Observer with wrapped angle innovation for psi
    Ax = np.dot(A, xhat)
    Bu = np.dot(B, u)
    yhat = np.dot(C, xhat)
    e = y - yhat
    # wrap the psi (3rd output) innovation to [-pi, pi]
    e[2,0] = wrap_pi(float(e[2,0]))
    Le = np.dot(L_obs, e)
    return Ax + Bu + Le

def RK4_solver(x, dt, f):

    k1 = f(x)
    k2 = f(x+0.5*dt*k1)
    k3 = f(x+0.5*dt*k2)
    k4 = f(x+dt*k3)


    return x+(dt/6.0)*(k1+2*k2+2*k3+k4)

# IMU setup + startup calibration procedure
try:
    i2c = I2C(1, I2C.CONTROLLER, baudrate=400_000)   # PB8=SCL, PB9=SDA
except AttributeError:
    i2c = I2C(1, I2C.MASTER,    baudrate=400_000)

imu = BNO055(i2c, address=0x28)   # no hardware reset pin
imu_bootstrap_calibration(imu, fusion_mode='NDOF', path="/flash/calibration.txt")
# ------------------ Tasks ------------------


def Motion_task():
    """Open-loop PWM step (kept)."""
    while True:
        if go_flag.read() and state.read()==1:
            try:
                LEFT.enable(); RIGHT.enable()
                if hasattr(ENCL,'zero'): ENCL.zero()
                if hasattr(ENCR,'zero'): ENCR.zero()
            except Exception:
                pass

            d = int(step_duty.read())
            try:
                LEFT.set_effort(d); RIGHT.set_effort(d)
            except Exception:
                pass

            t0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t0) < int(test_ms.read()):
                yield 0

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
    """Closed-loop wheel velocity. Works for state==2 (timed) and state==4 (follow).
       - Logs CSV ONLY for state==2 to avoid heap growth in follow mode.
       - Expects Lvel_sh/Rvel_sh in RAD/S.
    """
    import gc

    started   = False
    t0        = 0
    last_dbg  = time.ticks_ms()

    # cps ramp (per CTRL tick); CTRL_MS=10 ms => ~ramp_step*100 cps/s


    while True:
        st = state.read()
        running = (st == 2) or (st == 4)

        if running:
            ramp_step = 40
            if not started:
                try:
                    LEFT.enable(); RIGHT.enable()
                    if hasattr(ENCL,'zero'): ENCL.zero()
                    if hasattr(ENCR,'zero'): ENCR.zero()
                except Exception:
                    pass

                # start ramps near current targets (smooth engagement)
                #ramp_spL = float(spL.read())
                #ramp_spR = float(spR.read())

                # reset logs (only used by state==2)
                del t_ms[:]; del L_pos[:]; del L_vel[:]; del R_pos[:]; del R_vel[:]; del U_cmd[:]
                t0 = time.ticks_ms()
                started = True
                _print("[PID] START st={} ms={}".format(st, test_ms.read() if st==2 else -1))

            # --- measurements (RAD/S) ---
            lv = float(Lvel_sh.read())
            rv = float(Rvel_sh.read())
           
            # --- targets in COUNTS/SEC from Follow/UI ---
            tgtL_cps = float(spL.read())
            tgtR_cps = float(spR.read())
           
            # --- ramp cps toward targets ---
     
            # --- set PID setpoints in RAD/S ---
            pidL.set_setpoint(tgtL_cps)
            pidR.set_setpoint(tgtR_cps)

            # --- controller update (feedforward optional: V per cps) ---
            ff  =  0.003
            uL  = pidL.update(lv, feedforward = ff*tgtL_cps)
            uR  = pidR.update(rv, feedforward =ff*tgtR_cps)
           
            # --- volts -> duty% ---
            vbat  = 7.4
            dutyL = 100.0*(uL/vbat)
            dutyR = 100.0*(uR/vbat)
            
            if dutyL > 100: dutyL = 100
            if dutyL < -100: dutyL = -100
            if dutyR > 100: dutyR = 100
            if dutyR < -100: dutyR = -100

           # _print("uL={:.2f}V uR={:.2f}V dutyL={:.1f}% dutyR={:.1f}%".format(uL, uR, dutyL, dutyR))

            LEFT.enable(); RIGHT.enable()

            LEFT.set_effort(dutyL)
            RIGHT.set_effort(dutyR)
            
            '''
            # --- LOGGING: ONLY for timed velocity test (state==2) ---
            if st == 2 and len(t_ms) <2048:
                t_ms.append(time.ticks_diff(time.ticks_ms(), t0))
                lp = ENCL.get_position() * ENC_SIGN_L
                rp = ENCR.get_position() * ENC_SIGN_R
                L_pos.append(int(lp));  R_pos.append(int(rp))
                L_vel.append(lv);       R_vel.append(rv)
                U_cmd.append(int((dutyL + dutyR) // 2))
            '''
            # --- timed stop for state==2; continuous for state==4 ---
            # Only use timed stop when go_flag==1 (i.e., velocity test mode)
            # --- timed stop for state==2 ONLY WHEN go_flag==1 (velocity test mode) ---
            '''if st == 2 and go_flag.read() == 1 and time.ticks_diff(time.ticks_ms(), t0) >= int(test_ms.read()):
                try:
                    LEFT.set_effort(0); RIGHT.set_effort(0)
                    time.sleep_ms(150)
                    LEFT.disable(); RIGHT.disable()
                except Exception:
                    pass

                done_flag.write(1)
                go_flag.write(0)
                state.write(0)
                started = False

            '''

        else:
            if started:
                try:
                    LEFT.set_effort(0); RIGHT.set_effort(0)
                    time.sleep_ms(150)
                    LEFT.disable(); RIGHT.disable()
                except Exception:
                    pass
                started = False
            yield 0

        # --- throttle debug & GC to protect heap ---
        now = time.ticks_ms()
        if time.ticks_diff(now, last_dbg) >= 500:
            #_print("PID L={} R={}  lv={:.2f} rv={:.2f}  spL={} spR={}".format(
                #dutyL, dutyR, lv, rv, int(ramp_spL), int(ramp_spR)))
            last_dbg = now
            gc.collect()

        yield 0


def Encoder_task():
    """Update encoder positions/velocities and feed shares; also support open-loop logging."""
    started_ol = False
    last_lp = last_rp = 0
    last_us = time.ticks_us()
    t0_ms  = 0
  

    MAX_SAMPLES = 4096
    while True:
        # Always update encoders for fresh velocities
        if hasattr(ENCL,'update'): ENCL.update()
        if hasattr(ENCR,'update'): ENCR.update()

        lp = ENCL.get_position()*ENC_SIGN_L
        rp = ENCR.get_position()*ENC_SIGN_R

        now_us = time.ticks_us()
        dt = time.ticks_diff(now_us, last_us)
        if dt<=0:dt = 1
        if dt>50000:
            dt=50000
       
        
        lv = (lp - last_lp) * (1_000_000.0 / dt) *RAD_PER_COUNT
        rv = (rp - last_rp) * (1_000_000.0 / dt) *RAD_PER_COUNT
        alpha = 0.2
        lv_filt = (1-alpha)*Lvel_sh.read() + alpha*lv
        rv_filt = (1-alpha)*Rvel_sh.read() + alpha*rv
        Lvel_sh.write(lv_filt); Rvel_sh.write(rv_filt) 

        last_lp = lp; last_rp = rp; last_us = now_us

        # Open-loop logging (kept)
        if state.read()==1 and go_flag.read():
            if not started_ol:
                del t_ms[:]; del L_pos[:]; del L_vel[:]; del R_pos[:]; del R_vel[:]; del U_cmd[:]
                if hasattr(ENCL, 'zero'): ENCL.zero()
                if hasattr(ENCR, 'zero'): ENCR.zero()
                last_lp = 0; last_rp = 0; last_us = time.ticks_us()
                t0_ms = time.ticks_ms()
                started_ol = True

            t_ms.append(time.ticks_diff(time.ticks_ms(), t0_ms))
            L_pos.append(int(lp)); R_pos.append(int(rp))
            L_vel.append(float(lv)); R_vel.append(float(rv))
            U_cmd.append(int(step_duty.read()))

            if len(t_ms) >= MAX_SAMPLES:
               go_flag.write(0); done_flag.write(1)

        else:
            started_ol = False

        yield 0

     
def LineSensor_task():
    
    while True:
        if state.read()==3:
            _print("# Auto line calibration starting (white)...")
            sensors.calibrate_white_all()
            time.sleep_ms(4000)
            _print("# Now calibrate black line...")
            sensors.calibrate_black_all()
            time.sleep_ms(4000)
            sensors.save_calibration()
            _print("# Calibration done. Entering follow.")
            line_cal_done.write(1)

            state.write(4)
            
        yield 0

def Follow_task():
    err_int = 0.0
    anti_windup = 2000.0
    lasterror = 0.0
    filt_err = 0.0
    filt_deriv = 0.0
    '''
    e_prev      = 0.0
    e_filt      = 0.0
    last_us     = time.ticks_us()
    '''
    base_sp = 25
    min_sp = 10
    max_sp = 30
    centroid_filt = 0.0
    alpha = 0.4
    alpha_derivative = 0.3
    kp_line = 30.0
    ki_line = 0.005
            
    kd_line= 2.0

    noise = 0.01
    while True:
        
        if state.read() == 4:
           
            vals = sensors.read_all_norm()
            
            N = len(vals)
            centroid = sensors.get_centroid()
            _print(sensors.get_centroid())

            
            
            if centroid is None:
                spL.write(min_sp)
                spR.write(min_sp)
                line_lost.write(1)
                yield 0
                continue
            line_lost.write(0)

            error =centroid
            if abs(error) < noise:
                error = 0.0
            filt_err = alpha * error + (1-alpha)*filt_err
            err_int += filt_err
            err_int = max(min(err_int,1500),-1500)

            raw_deriv = filt_err - lasterror
            filt_deriv = alpha_derivative*raw_deriv + (1-alpha_derivative)* filt_deriv

            lasterror = filt_err

            control = kp_line*filt_err + ki_line*err_int + kd_line*filt_deriv
            control = max(min(control,20),-20)
            #wheel speed
            balance = 1.20
           

            
            
            spR_val = base_sp + control 
           
            spL_val = (base_sp - control)*balance
            

            #saturation
            if spL_val > max_sp: spL_val = max_sp
            if spL_val < min_sp: spL_val = min_sp
            if spR_val > max_sp: spR_val = max_sp
            if spR_val < min_sp: spR_val = min_sp
            
            spL.write(int(spL_val))
            spR.write(int(spR_val))
        
            pid_mode.write(1)  
       
            
        else:
            err_int = 0.0
            e_prev = 0.0
            #centroid = 0.0
            fork_detected.write(0)
            line_lost.write(0)

        yield 0



def task_imu():
    
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


def observer_task():

    dt = 0.02
    t = 0.0
    x_omegaL = 0.0
    x_omegaR = 0.0
    x_s = 0.0
    x_psi = 0.0
    
    inited = False

    while True:
        vbat = 7.4

        if hasattr(LEFT, 'last_effort'):
            uL = (LEFT.last_effort/100.0) * vbat
        else:
            uL = 0.0

        if hasattr(RIGHT,'last_effort'):
            uR = (RIGHT.last_effort/100.0)*vbat
        else:
            uR = 0.0
    
    # encoder positions angle (rad) to arc length
        omegaL_meas = float(Lvel_sh.read())
        omegaR_meas = float(Rvel_sh.read())

        psi = math.radians(heading_deg.read())
        psidot = math.radians(yaw_rate_dps.read())

        if not inited:
            x_omegaL = omegaL_meas
            x_omegaR = omegaR_meas
            x_s = 0.0
            x_psi = psi
            inited = True
        
        psidot_hat = (r/L)*(x_omegaR-x_omegaL)

        e0 = omegaL_meas - x_omegaL
        e1 = omegaR_meas - x_omegaR
        e2 = wrap_pi(psi - x_psi)
        e3 = psidot - psidot_hat
        
        def f_omegaL(x):
            return a*x + b*uL + L1*e0
        def f_omegaR(x):
            return a*x + b*uR + L2*e1
        def f_s(x):
            return(r/2.0)*(x_omegaL + x_omegaR) + L3*e2
        def f_psi(x):
            return(r/L)*(x_omegaR - x_omegaL) + L4*e3

        x_omegaL = RK4_solver(x_omegaL,dt,f_omegaL)
        x_omegaR = RK4_solver(x_omegaR,dt,f_omegaR)
        x_s = RK4_solver(x_s,dt,f_s)
        x_psi = RK4_solver(x_psi,dt,f_psi)
        x_psi = wrap_pi(x_psi) 

        xhat_omegaL.write(x_omegaL)
        xhat_omegaR.write(x_omegaR)
        xhat_s.write(x_s)
        xhat_psi.write(x_psi)
        psidot_est = (r/L) * (x_omegaR - x_omegaL)
        xhat_psidot.write(psidot_est)

        yield 0




def map_task(): # cut into segments to analyze
    _print("task created")
    NAV_START = 0
    NAV_LINE_MAIN = 1
    NAV_FORK1 = 2
    NAV_DIAMOND = 3
    NAV_BIG_ARC = 4
    NAV_DOT_LINE = 5
    NAV_FORK2  = 6
    NAV_TOP_CURVE = 7
    NAV_BRIDGE = 8
    NAV_BRIDGE_TURN = 9
    NAV_LINE_FOLLOW = 10
    NAV_WALL = 11
    NAV_FINAL_LINE = 12
    NAV_FINISH = 13
    NAV_TEST_STRAIGHT = 100
    NAV_TEST_TURN = 101


   


    
    BASE_SP = 20
    RIGHT_BRANCH_TURN_SPEED = 50
    RIGHT_BRANCH_DURACTION = 0.40 #meters to commit

    BRIDGE_ENTRY_S = 3.70
    BRIDGE_LENGTH = 0.4 # meters forward on bridge
    BRIDGE_TURN_ANGLE = math.radians(90)
    BASE_BRIDGE = 25

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
            line_cal_done.write(0)
            state.write(3) # starts linesenosr task
            _print("[NAV] -> LINE_CAL")
            st = NAV_LINE_MAIN
            nav_state.write(st)

        elif st == NAV_LINE_MAIN :
            state.write(4)
            if right_branch_detected.read():
                #spL.write(BASE_SP - 10)
                #spR.write(BASE_SP + 10)
                segment_s0 = s_hat
                nav_state.write(NAV_FORK1)

        elif st == NAV_FORK1:  
            if s_hat - segment_s0 >= RIGHT_BRANCH_DURACTION:
                st = NAV_DIAMOND
                nav_state.write(NAV_DIAMOND)
                
        elif st == NAV_DIAMOND:

            st = NAV_BIG_ARC
            nav_state.write(NAV_BIG_ARC)


        elif st == NAV_BIG_ARC:
            st = NAV_DOT_LINE
            nav_state.write(st)

        elif st == NAV_DOT_LINE:
            segment_start = s_hat
            st = NAV_FORK2
            nav_state.write(st)

        elif st == NAV_FORK2:
            if s_hat > BRIDGE_ENTRY_S - 0.2:
                st = NAV_TOP_CURVE
                nav_state.write(NAV_TOP_CURVE)

            if (s_hat - s_start) > 0.25:
                line_base_sp_sh(BASE_SP)
                st = NAV_TOP_CURVE
                nav_state.write(st)
        
        elif st == NAV_TEST_STRAIGHT:
            target_dist = 1.0
            speed = 20
            
            straight_s0 = s_hat

            spL.write(speed)
            spR.write(speed)
            go_flag.write(1)
            pid_mode.write(1)
            state.write(2)

            if(s_hat - straight_s0) >= target_dist:
                spL.write(0)
                spR.write(0)
                pid_mode.write(1)
                state.write(0)
               

        elif st == NAV_TEST_TURN:

            target_angle = math.radians(90)
            turn_rate_speed = 20

            if 'tt_init' not in globals():
                tt_init = True 
                psi0 = psi_hat

            spL.write(-turn_rate_speed)
            spR.write(turn_rate_speed)

            pid_mode.write(1)
            state.write(2)

            dpsi = wrap_pi(psi_hat - psi0)

            if abs(dpsi) >= abs(target_angle):
                spL.write(0)
                spR.write(0)
                pid_mode.write(1)
                state.write(0)
                tt_init = False
                



        yield 0 


def main():
    # Create tasks and add to scheduler
    state.write(0)
    go_flag.write(0)
    line_cal_done.write(0)
    nav_state.write(0)

    #cotask.task_list.append(cotask.Task(Motion_task,name="Motion",priority=2, period=20, profile=False))
    cotask.task_list.append(cotask.Task(PID_task,  name="PID",    priority=5, period=20, profile=False))
    cotask.task_list.append(cotask.Task(Encoder_task,name="Encoder",priority=5,period=10, profile=False))

    cotask.task_list.append(cotask.Task(LineSensor_task, name ="LineSens", priority = 3, period =30, profile = False))
    cotask.task_list.append(cotask.Task(Follow_task, name="Follow", priority=3, period=5, profile=False))
    cotask.task_list.append(cotask.Task(task_imu, name="IMU", priority=2, period=80, profile=False, trace=False))
    cotask.task_list.append(cotask.Task(observer_task, name="Observer", priority=3, period = 20, profile =False))
    cotask.task_list.append(cotask.Task(map_task, name="Map", priority = 2, period = 70, profile = False))
   
    
    #_print("READY: g <duty%> <ms>  |  v <spL_cps> <spR_cps> <ms>  |  cfg kp ki kd alpha ipd ff  |  vbat <V>")
 
    while True:
        cotask.task_list.pri_sched()

if __name__ == "__main__":
    main()
