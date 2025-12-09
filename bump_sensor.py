# bump_sensor.py
# Simple active-low bump switches with debounced edge detection.
try:
    import utime as time
except ImportError:
    import time
from pyb import Pin

class BumpSensor:
    """Active-low bump switch with polling-based debouncing."""
    def __init__(self, pin_id, name=None, debounce_ms=30, pullup=True):
        self.pin = Pin(pin_id, Pin.IN, Pin.PULL_UP if pullup else None)
        self.name = name or str(pin_id)
        self.debounce_ms = debounce_ms
        self._last = self.pin.value()          # 1 = released, 0 = pressed
        self._last_t = time.ticks_ms()

    def pressed(self) -> bool:
        return self.pin.value() == 0

    def state(self) -> int:
        return self.pin.value()

    def update(self):
        """Call often. Returns 'hit'/'release'/None (debounced)."""
        now = time.ticks_ms()
        v = self.pin.value()
        if v != self._last and time.ticks_diff(now, self._last_t) >= self.debounce_ms:
            self._last_t = now
            self._last = v
            return "hit" if v == 0 else "release"
        return None


class BumpBoard:
    """Group of bump sensors (e.g., three Romi bumper lines)."""
    def __init__(self, pairs, debounce_ms=30):
        # pairs: list[ (name, pin_id) ]
        self.sensors = [BumpSensor(pin, name, debounce_ms=debounce_ms) for (name, pin) in pairs]

    def update(self):
        """Call often. Yields (name, event, t_ms) for each edge seen."""
        now = time.ticks_ms()
        events = []
        for s in self.sensors:
            ev = s.update()
            if ev:
                events.append((s.name, ev, now))
        return events

    def any_pressed(self) -> bool:
        return any(s.pressed() for s in self.sensors)

    def snapshot(self):
        """Dict name -> raw state (0=pressed, 1=released)."""
        return {s.name: s.state() for s in self.sensors}
