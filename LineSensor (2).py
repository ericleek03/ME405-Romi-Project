from pyb import Pin, ADC
import utime, json


class LineSensorChannel:
    """One reflective sensor channel with simple oversampling and filtering."""
    def __init__(self, pin_name, name=None, oversample=8):
        self.pin        = Pin(pin_name)
        self.adc        = ADC(self.pin)
        self.name       = name or pin_name
        self.oversample = int(oversample)

        # Calibration range (raw ADC)
        self.min_val = 4095
        self.max_val = 0

        # Low-pass filtered normalized value
        self._filt      = 0.0
        self._filt_init = False

    def read_raw(self):
        """Return oversampled raw ADC value (0–4095)."""
        total = 0
        for _ in range(self.oversample):
            total += self.adc.read()
        return total // self.oversample

    def read_norm(self, alpha=0.3):
        """Return low-pass filtered normalized brightness in [0,1].

        0  ~ calibrated BLACK side
        1  ~ calibrated WHITE side
        """
        raw = self.read_raw()

        lo = min(self.min_val, self.max_val)
        hi = max(self.min_val, self.max_val)
        if hi <= lo:
            lo, hi = 0, 4095

        if raw < lo:
            raw = lo
        if raw > hi:
            raw = hi

        norm = (raw - lo) / float(hi - lo)

        if not self._filt_init:
            self._filt = norm
            self._filt_init = True
        else:
            self._filt = (1.0 - alpha) * self._filt + alpha * norm

        return self._filt


class LineSensorArray:
    """Multi-channel line sensor with centroid calculation and calibration.

    Methods expected by main.py:
        - calibrate_white_all()
        - calibrate_black_all()
        - save_calibration(path="line_cal.json")
        - load_calibration(path="line_cal.json")
        - read_all_norm()  -> list of brightness values in [0,1]
        - get_centroid()   -> error in [-1,1] (0 on center), or None if no line
    """
    def __init__(self, pins, oversample=8, filt_alpha=0.3, cal_path="line_cal.json"):
        self.channels = [
            LineSensorChannel(pin_name, name="S{}".format(i), oversample=oversample)
            for i, pin_name in enumerate(reversed(pins))
        ]
        self.filt_alpha = float(filt_alpha)
        self.cal_path   = cal_path
        self.calibrated = False

        # Filtered centroid
        self._centroid_f    = 0.0
        self._centroid_init = False

        # Try to load any existing calibration
        self.load_calibration(self.cal_path)

    # ---------- Internal helpers ----------

    def _read_all_raw(self):
        return [ch.read_raw() for ch in self.channels]

    def _avg_samples(self, nsamples=32, delay_ms=5):
        n = len(self.channels)
        acc = [0] * n
        for _ in range(nsamples):
            raws = self._read_all_raw()
            for i, v in enumerate(raws):
                acc[i] += v
            utime.sleep_ms(delay_ms)
        return [a // nsamples for a in acc]

    # ---------- Public API used by main.py ----------

    def calibrate_white_all(self, nsamples=32, delay_ms=5):
        """Capture background (no line) readings."""
        vals = self._avg_samples(nsamples, delay_ms)
        for ch, raw in zip(self.channels, vals):
            ch.min_val = raw   # background side
        self.calibrated = True

    def calibrate_black_all(self, nsamples=32, delay_ms=5):
        """Capture line (black tape) readings."""
        vals = self._avg_samples(nsamples, delay_ms)
        for ch, raw in zip(self.channels, vals):
            ch.max_val = raw   # line side
        self.calibrated = True

    def save_calibration(self, path=None):
        """Save per-channel min/max to JSON."""
        if path is None:
            path = self.cal_path
        data = []
        for ch in self.channels:
            data.append({"min": int(ch.min_val), "max": int(ch.max_val)})
        try:
            with open(path, "w") as f:
                json.dump(data, f)
            print("[LineSensor] Calibration saved to", path)
        except Exception as e:
            print("[LineSensor] Save failed:", e)

    def load_calibration(self, path=None):
        """Load per-channel min/max from JSON, if available."""
        if path is None:
            path = self.cal_path
        try:
            with open(path, "r") as f:
                data = json.load(f)
            for ch, d in zip(self.channels, data):
                ch.min_val = d.get("min", 0)
                ch.max_val = d.get("max", 4095)
            print("[LineSensor] Calibration loaded from", path)
            self.calibrated = True
            return True
        except Exception as e:
            print("[LineSensor] Load failed (using defaults):", e)
            # keep defaults; not fatal
            self.calibrated = False
            return False

    def read_all_norm(self):
        """List of brightness values in [0,1] (low for line, high for background)."""
        return [ch.read_norm(self.filt_alpha) for ch in self.channels]

    def get_centroid(self):
        """Return filtered centroid error in [-1,1], or None if no line.

        We convert brightness -> 'line strength' by (1 - norm) so that a darker
        line on lighter floor gives larger weights.
        """
        norms = self.read_all_norm()
        # Convert brightness to line strength (dark line => high strength)
        #vals = [1.0 - v for v in norms]
        
        vals = norms
        total = sum(vals)
        if total < 0.05:
            # No line detected
            return None

        n = len(vals)
        # Positions: symmetric about 0, e.g. for 5 sensors -> [-2,-1,0,1,2]
        mid = (n - 1) / 2.0
        positions = [i - mid for i in range(n)]
        num = 0.0
        for p, v in zip(positions, vals):
            num += p * v
        c_raw = num / total  # in roughly [-mid, mid]
        c = c_raw / mid
        # Normalize to [-1,1]
        
        # Low-pass filter centroid to knock down jitter
        if not self._centroid_init:
            self._centroid_f = c
            self._centroid_init = True
        else:
            alpha = 0.4
            self._centroid_f = (1-alpha)* self._centroid_f + alpha*c

        return self._centroid_f
