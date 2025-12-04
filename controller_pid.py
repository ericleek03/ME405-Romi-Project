class PIDController:
    def __init__(self, kp, ki, kd, out_max, delta_t=None, dt=None,
                 sp=0.0, tau=None, alpha=None, ipd=False):
        # Gains & limits
        self.kp = float(kp); self.ki = float(ki); self.kd = float(kd)
        self.out_max = abs(float(out_max))

        # Accept either delta_t or dt (main.py uses dt)
        if delta_t is None and dt is None:
            raise ValueError("Provide delta_t or dt (controller period, s)")
        self.dt = float(delta_t if delta_t is not None else dt)

        # Setpoint and options (ipd accepted for API compatibility; not required here)
        self.sp = float(sp); self.ipd = bool(ipd)

        # State
        self._integrator = 0.0
        self._prev_error = 0.0
        self._prev_pv = None
        self._d_filt = 0.0
        self._p_action = 0.0; self._i_action = 0.0; self._d_action = 0.0
        self.u = 0.0

        # Derivative low-pass: choose either tau or alpha
        if tau is not None and tau >= 0.0:
            self.alpha = float(tau) / (float(tau) + self.dt) if (float(tau) + self.dt) > 0 else 0.0
        elif alpha is not None:
            self.alpha = max(0.0, min(1.0, float(alpha)))
        else:
            self.alpha = 0.0

    def set_setpoint(self, sp):
        self.sp = float(sp)

    def set_gains(self, kp=None, ki=None, kd=None):
        if kp is not None: self.kp = float(kp)
        if ki is not None: self.ki = float(ki)
        if kd is not None: self.kd = float(kd)

    def reset(self, keep_setpoint=True):
        self._integrator = 0.0
        self._prev_error = 0.0
        self._prev_pv = None
        self._d_filt = 0.0
        self._p_action = self._i_action = self._d_action = 0.0
        self.u = 0.0
        if not keep_setpoint:
            self.sp = 0.0

    def update(self, pv, feedforward=0.0):
        # Error
        pv = float(pv)
        error = self.sp - pv

        # P
        self._p_action = self.kp * error

        # D on measurement (negative sign because d(meas)/dt opposes error)
        if self._prev_pv is None:
            d_meas = 0.0
        else:
            d_meas = -(pv - self._prev_pv) / self.dt
        self._prev_pv = pv

        # Low-pass derivative
        self._d_filt = self.alpha * self._d_filt + (1.0 - self.alpha) * d_meas
        self._d_action = self.kd * self._d_filt

        # Unsaturated output (volts) with optional feedforward (volts)
        u_unsat = self._p_action + self._integrator + self._d_action + float(feedforward)

        # Saturate
        u = max(-self.out_max, min(self.out_max, u_unsat))

        # Anti-windup: integrate unless saturated in the wrong direction
        will_saturate = (u != u_unsat)
        driving_away_hi = (u >= self.out_max and error > 0)
        driving_away_lo = (u <= -self.out_max and error < 0)
        integrate = (not will_saturate) or (driving_away_hi or driving_away_lo)

        if integrate:
            self._integrator += 0.5 * self.ki * self.dt * (error + self._prev_error)
            # Bound integrator so total fits in saturation window
            max_i = self.out_max - (self._p_action + self._d_action + float(feedforward))
            min_i = -self.out_max - (self._p_action + self._d_action + float(feedforward))
            if self._integrator > max_i: self._integrator = max_i
            if self._integrator < min_i: self._integrator = min_i

        self._prev_error = error

        # Final saturated output
        self._i_action = self._integrator
        self.u = max(-self.out_max, min(self.out_max,
                                        self._p_action + self._i_action + self._d_action + float(feedforward)))
        return self.u

    # Optional diagnostics
    def get_PI_components(self):
        return self._p_action, self._i_action

    def get_PID_components(self):
        return self._p_action, self._i_action, self._d_action


def voltage_to_duty(u_volts, vbat_volts):
    """Convert controller output volts to duty fraction in [-1.0, 1.0]."""
    try:
        vbat = float(vbat_volts)
        if vbat <= 0:
            vbat = 7.4
    except Exception:
        vbat = 7.4
    d = float(u_volts) / vbat
    if d > 1.0: d = 1.0
    if d < -1.0: d = -1.0
    return d
