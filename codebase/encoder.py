# encoder.py
# Quadrature encoder driver using STM32 timer in hardware ENC_AB mode.

from pyb import Pin, Timer
from time import ticks_us, ticks_diff


class Encoder:
    """Quadrature encoder using an STM32 timer in ENC_AB mode."""

    def __init__(self, tim: Timer, chA_pin: Pin, chB_pin: Pin,
                 period: int = 0xFFFF, invert: bool = False):
        """Configure hardware quadrature decoding.

        tim    – Timer configured for encoder mode
        chA/B  – pins connected to encoder A/B outputs
        period – timer auto-reload (wrap) value
        invert – True to flip sign convention
        """
        self.tim = tim
        self.period = int(period) & 0xFFFF

        # Enable pull-ups so channels are never left floating
        try:
            chA_pin.init(mode=Pin.IN, pull=Pin.PULL_UP)
            chB_pin.init(mode=Pin.IN, pull=Pin.PULL_UP)
        except TypeError:
            # Some firmwares don't allow .init on Pin created by name
            pass

        # Configure timer for quadrature decoding on CH1/CH2
        self.tim.init(prescaler=0, period=self.period)
        self.tim.channel(1, Timer.ENC_AB, pin=chA_pin)
        self.tim.channel(2, Timer.ENC_AB, pin=chB_pin)

        # Direction sign (flip if "forward" counts negative)
        self.sign = -1 if invert else 1

        # Integrated position (ticks), velocity (ticks/s), and last samples
        self.position   = 0
        self.velocity   = 0.0
        self.prev_count = self.tim.counter()
        self.prev_time  = ticks_us()
        self.delta      = 0
        self.dt_us      = 0

    def update(self):
        """Read timer, update position and velocity estimates."""
        now   = ticks_us()
        count = self.tim.counter()

        # Raw delta with wrap-around correction
        delta = count - self.prev_count
        half  = (self.period + 1) // 2
        if delta < -half:
            delta += (self.period + 1)
        elif delta > half:
            delta -= (self.period + 1)

        # Apply chosen sign convention
        delta *= self.sign

        # Integrate position and save last delta
        self.position += delta
        self.delta = delta

        # Time difference in microseconds
        dt = ticks_diff(now, self.prev_time)
        self.dt_us = dt

        # Velocity in ticks per second (0 if dt=0)
        self.velocity = (delta * 1_000_000.0) / dt if dt > 0 else 0.0

        # Update last timestamp and count
        self.prev_count = count
        self.prev_time = now

    def get_position(self):
        """Return integrated position in encoder ticks."""
        return self.position

    def get_velocity(self):
        """Return last velocity estimate in ticks/s."""
        return self.velocity

    def zero(self):
        """Reset position and velocity to zero, keeping current count as base."""
        self.position = 0
        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()
        self.delta = 0
        self.dt_us = 0
        self.velocity = 0.0