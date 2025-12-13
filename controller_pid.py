class PIDController:
    """General-purpose PID controller with anti-windup and filtered D-term."""

    def __init__(self, kp, ki, kd, out_max, delta_t=None, dt=None,
                 sp=0.0, tau=None, alpha=None, ipd=False):
        """Initialize PID gains, limits, and timing.

        kp, ki, kd – proportional / integral / derivative gains
        out_max    – absolute saturation limit for output
        delta_t/dt – controller period [s]
        sp         – initial setpoint
        tau        – derivative filter time constant (optional)
        alpha      – derivative filter coefficient 0..1 (optional)
        ipd        – accepted for API compatibility (not used directly)
        """
        # Gains & limits
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_max = abs(float(out_max))

        # Choose the dt parameter name used by caller
        if delta_t is None and dt is None:
            raise ValueError("Provide delta_t or dt (controller period, s)")
        self.dt = float(delta_t if delta_t is not None else dt)

        # Setpoint and options
        self.sp = float(sp)
        self.ipd = bool(ipd)     # not used, kept for drop-in compatibility

        # Internal state
        self._integrator = 0.0
        self._prev_error = 0.0
        self._prev_pv = None
        self._d_filt = 0.0
        self._p_action = 0.0
        self._i_action = 0.0
        self._d_action = 0.0
        self.u = 0.0             # last output

        # Derivative low-pass filter setup (choose tau or alpha)
        if tau is not None and tau >= 0.0:
            # First-order low-pass based on tau and dt
            self.alpha = float(tau) / (float(tau) + self.dt) \
                         if (float(tau) + self.dt) > 0 else 0.0
        elif alpha is not None:
            # Clamp alpha into [0,1]
            self.alpha = max(0.0, min(1.0, float(alpha)))
        else:
            # No filtering (pure derivative)
            self.alpha = 0.0

    def set_setpoint(self, sp):
        """Update setpoint."""
        self.sp = float(sp)

    def set_gains(self, kp=None, ki=None, kd=None):
        """Update PID gains selectively."""
        if kp is not None:
            self.kp = float(kp)
        if ki is not None:
            self.ki = float(ki)
        if kd is not None:
            self.kd = float(kd)

    def reset(self, keep_setpoint=True):
        """Clear controller state (integrator, D-term, output)."""
        self._integrator = 0.0
        self._prev_error = 0.0
        self._prev_pv = None
        self._d_filt = 0.0
        self._p_action = self._i_action = self._d_action = 0.0
        self.u = 0.0
        if not keep_setpoint:
            self.sp = 0.0

    def update(self, pv, feedforward=0.0):
        """Run one PID update step.

        pv          – process variable (measurement)
        feedforward – additional output (volts) added before saturation

        Returns: saturated controller output.
        """
        # Error between setpoint and measurement
        pv = float(pv)
        error = self.sp - pv

        # Proportional term
        self._p_action = self.kp * error

        # Derivative on measurement (helps reduce noise on error)
        if self._prev_pv is None:
            d_meas = 0.0
        else:
            # Negative sign: d(meas)/dt opposes error
            d_meas = -(pv - self._prev_pv) / self.dt
        self._prev_pv = pv

        # Low-pass filter the derivative term
        self._d_filt = self.alpha * self._d_filt + (1.0 - self.alpha) * d_meas
        self._d_action = self.kd * self._d_filt

        # Unsaturated output (volts)
        u_unsat = self._p_action + self._integrator + self._d_action + float(feedforward)

        # Apply saturation
        u = max(-self.out_max, min(self.out_max, u_unsat))

        # Decide if integrator should update (simple anti-windup)
        will_saturate = (u != u_unsat)
        driving_away_hi = (u >= self.out_max and error > 0)
        driving_away_lo = (u <= -self.out_max and error < 0)
        integrate = (not will_saturate) or (driving_away_hi or driving_away_lo)

        # Trapezoidal integration of error with clamped range
        if integrate:
            self._integrator += 0.5 * self.ki * self.dt * (error + self._prev_error)
            # Limit integrator so total output still fits in saturation band
            max_i = self.out_max - (self._p_action + self._d_action + float(feedforward))
            min_i = -self.out_max - (self._p_action + self._d_action + float(feedforward))
            if self._integrator > max_i:
                self._integrator = max_i
            if self._integrator < min_i:
                self._integrator = min_i

        self._prev_error = error

        # Final saturated output with updated I-term
        self._i_action = self._integrator
        self.u = max(
            -self.out_max,
            min(self.out_max,
                self._p_action + self._i_action + self._d_action + float(feedforward)),
        )
        return self.u

    # Optional diagnostics below: useful for plotting and tuning
    def get_PI_components(self):
        """Return (P, I) components of last output."""
        return self._p_action, self._i_action

    def get_PID_components(self):
        """Return (P, I, D) components of last output."""
        return self._p_action, self._i_action, self._d_action


def voltage_to_duty(u_volts, vbat_volts):
    """Convert controller output in volts to duty fraction [-1, 1]."""
    try:
        vbat = float(vbat_volts)
        if vbat <= 0:
            # Fallback if battery measurement is invalid
            vbat = 7.4
    except Exception:
        vbat = 7.4

    # Normalize against battery voltage
    d = float(u_volts) / vbat
    if d > 1.0:
        d = 1.0
    if d < -1.0:
        d = -1.0
    return d
