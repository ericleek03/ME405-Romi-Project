from pyb import Pin, Timer


class Motor:
    """Simple DRV8838-style motor driver with PWM + direction."""

    def __init__(self, PWM_Pin: Pin, DIR_Pin: Pin, nSLP_Pin: Pin,
                 tim: Timer, chan: int):
        """Configure PWM channel, direction pin, and sleep pin."""
        # nSLEEP pin: high = enabled, low = sleep
        self.nSLP_pin = Pin(nSLP_Pin, mode=Pin.OUT_PP)
        # Direction pin: high/low selects direction
        self.DIR_pin = Pin(DIR_Pin, mode=Pin.OUT_PP)
        # PWM channel with initial 0% duty
        self.PWM_chan = tim.channel(
            chan, pin=PWM_Pin, mode=Timer.PWM, pulse_width_percent=0
        )

    def set_effort(self, effort):
        """Set signed effort in range [-100, 100] as PWM duty."""
        # Positive effort: one direction
        if effort > 0:
            self.DIR_pin.low()
            self.PWM_chan.pulse_width_percent(effort)
        # Negative effort: opposite direction
        else:
            self.DIR_pin.high()
            self.PWM_chan.pulse_width_percent(-effort)

    def enable(self):
        """Wake the motor driver (allow PWM to move motor)."""
        self.nSLP_pin.high()

    def disable(self):
        """Put motor driver into sleep / coast (no effort)."""
        self.nSLP_pin.low()