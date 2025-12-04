  # encoder.py
from pyb import Pin, Timer
from time import ticks_us, ticks_diff

class Encoder:
    """Quadrature encoder using an STM32 timer in ENC_AB mode."""

    def __init__(self, tim: Timer, chA_pin: Pin, chB_pin: Pin,
                 period: int = 0xFFFF, invert: bool = False):
        self.tim = tim
        self.period = int(period) & 0xFFFF

        # Make sure inputs are not floating; many gearmotor encoders are open-drain.
        try:
            chA_pin.init(mode=Pin.IN, pull=Pin.PULL_UP)
            chB_pin.init(mode=Pin.IN, pull=Pin.PULL_UP)
        except TypeError:
            # Some builds won't accept .init on a Pin created by name; safe to skip
            pass

        # Hardware quadrature decode
        self.tim.init(prescaler=0, period=self.period)
        self.tim.channel(1, Timer.ENC_AB, pin=chA_pin)  # CH1 must be the CH1-capable pin
        self.tim.channel(2, Timer.ENC_AB, pin=chB_pin)  # CH2 must be the CH2-capable pin

        # Direction convention (flip if forward reads negative)
        self.sign = -1 if invert else 1

        # State for integration / velocity
        self.position   = 0
        self.velocity   = 0.0
        self.prev_count = self.tim.counter()
        self.prev_time  = ticks_us()
        self.delta      = 0
        self.dt_us      = 0

    def update(self):
        now   = ticks_us()
        count = self.tim.counter()

        # delta with wrap-around correction
        delta = count - self.prev_count
        half  = (self.period + 1) // 2
        if   delta < -half: delta += (self.period + 1)
        elif delta >  half: delta -= (self.period + 1)

        delta *= self.sign  # <— flip sign if requested

        self.position += delta
        self.delta     = delta

        dt = ticks_diff(now, self.prev_time)
        self.dt_us = dt
        self.velocity = (delta * 1_000_000.0) / dt if dt > 0 else 0.0

        self.prev_count = count
        self.prev_time  = now

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity

    def zero(self):
        self.position   = 0
        self.prev_count = self.tim.counter()
        self.prev_time  = ticks_us()
        self.delta      = 0
        self.dt_us      = 0
        self.velocity   = 0.0
