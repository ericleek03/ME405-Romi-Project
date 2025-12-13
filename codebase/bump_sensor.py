# bump_sensor.py
# Active-low bump switches with simple debounced edge detection.

try:
    import utime as time
except ImportError:
    import time
from pyb import Pin


class BumpSensor:
    """Active-low bump switch with polling-based debouncing."""

    def __init__(self, pin_id, name=None, debounce_ms=30, pullup=True):
        """Configure one bump switch.

        pin_id      – MCU pin name or object
        name        – label for identification / logging
        debounce_ms – minimum time between state changes
        pullup      – enable internal pull-up if True
        """
        # Configure input pin (usually pulled up, active low)
        self.pin = Pin(pin_id, Pin.IN, Pin.PULL_UP if pullup else None)

        # Human-readable name (e.g. "LEFT", "RIGHT")
        self.name = name or str(pin_id)

        # Debounce interval in milliseconds
        self.debounce_ms = debounce_ms

        # Last stable state and time (1 = released, 0 = pressed)
        self._last = self.pin.value()
        self._last_t = time.ticks_ms()

    def pressed(self) -> bool:
        """Return True if switch currently reads as pressed (active low)."""
        return self.pin.value() == 0

    def state(self) -> int:
        """Return raw pin state (0 = pressed, 1 = released)."""
        return self.pin.value()

    def update(self):
        """Check for debounced edges.

        Returns:
            "hit"      – new press detected
            "release"  – new release detected
            None       – no new debounced event
        """
        now = time.ticks_ms()
        v = self.pin.value()

        # Only accept a change if it has been stable long enough
        if v != self._last and time.ticks_diff(now, self._last_t) >= self.debounce_ms:
            self._last_t = now
            self._last = v
            return "hit" if v == 0 else "release"

        return None


class BumpBoard:
    """Group of bump sensors (logical board of multiple switches)."""

    def __init__(self, pairs, debounce_ms=30):
        """Create sensors from (name, pin_id) pairs.

        pairs – list of tuples: (name, pin_id)
        """
        # Build a BumpSensor for each named pin
        self.sensors = [
            BumpSensor(pin, name, debounce_ms=debounce_ms)
            for (name, pin) in pairs
        ]

    def update(self):
        """Poll all sensors and return list of debounced events.

        Returns a list of (name, event, t_ms) tuples.
        """
        now = time.ticks_ms()
        events = []

        # Collect any debounced edges seen this cycle
        for s in self.sensors:
            ev = s.update()
            if ev:
                events.append((s.name, ev, now))

        return events

    def any_pressed(self) -> bool:
        """Return True if at least one bump sensor is pressed."""
        return any(s.pressed() for s in self.sensors)

    def snapshot(self):
        """Return dict mapping name -> raw state (0=pressed, 1=released)."""
        return {s.name: s.state() for s in self.sensors}
