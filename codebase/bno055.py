# bno055.py
# Minimal BNO055 IMU driver for MicroPython (pyb.*)
# Features:
#   - Initialize with a preconfigured pyb.I2C (CONTROLLER) and optional RST pin
#   - Set/read operating (fusion) modes
#   - Read calibration status (parsed + raw)
#   - Dump/restore calibration coefficients (binary blob)
#   - Read Euler angles (deg) and heading()
#   - Read angular velocity (deg/s) and yaw_rate()
#
# Wiring to NUCLEO-L476RG (default I2C1):
#   3Vo  -> 3V3
#   GND  -> GND
#   SDA  -> PB_9   (I2C1_SDA)
#   SCL  -> PB_8   (I2C1_SCL)
#   RST  -> PA_13  (optional, active LOW)
#
# Example usage:
#   from pyb import I2C, Pin
#   import utime
#   from bno055 import BNO055
#
#   i2c = I2C(1, I2C.CONTROLLER, baudrate=400_000)  # I2C1 on PB8/PB9
#   imu  = BNO055(i2c, address=0x28, rst_pin='PA13')  # ADR low => 0x28 (default)
#   imu.set_mode('NDOF')  # convenient fusion mode
#   while True:
#       h, r, p = imu.read_euler()
#       wz = imu.yaw_rate()
#       print('heading=%.1f roll=%.1f pitch=%.1f  wz=%.2f dps' % (h, r, p, wz))
#       utime.sleep_ms(100)

from pyb import I2C, Pin
import utime, struct

class BNO055:
    """BNO055 IMU driver (I2C). Keeps API small and practical for Romi.
    """

    # I2C addresses (ADR pin low/high)
    ADDR_A = 0x28
    ADDR_B = 0x29

    # Registers (page 0 unless noted)
    _REG_CHIP_ID    = 0x00  # expects 0xA0
    _REG_PAGE_ID    = 0x07
    _REG_ACCEL_DATA = 0x08  # 6 bytes
    _REG_MAG_DATA   = 0x0E  # 6 bytes
    _REG_GYRO_DATA  = 0x14  # 6 bytes (X,Y,Z), 16 LSB/dps
    _REG_EULER_H    = 0x1A  # 6 bytes (H,R,P), 1/16 deg/LSB
    _REG_TEMP       = 0x34
    _REG_CALIB_STAT = 0x35
    _REG_SYS_TRIGGER= 0x3F
    _REG_PWR_MODE   = 0x3E
    _REG_OPR_MODE   = 0x3D
    _REG_UNIT_SEL   = 0x3B

    # Calibration coefficient block (page 0, 0x55..0x6A inclusive -> 22 bytes)
    _REG_CALIB_START = 0x55
    _CALIB_LEN       = 22

    # Operating modes (OPR_MODE)
    MODES = {
        'CONFIG'        : 0x00,
        # Non-fusion:
        'ACCONLY'       : 0x01, 'MAGONLY'  : 0x02, 'GYROONLY' : 0x03,
        'ACCMAG'        : 0x04, 'ACCGYRO'  : 0x05, 'MAGGYRO'  : 0x06,
        'AMG'           : 0x07,
        # Fusion:
        'IMU'           : 0x08,  # accel + gyro
        'COMPASS'       : 0x09,  # accel + mag
        'M4G'           : 0x0A,  # mag + accel (gyro-less)
        'NDOF_FMC_OFF'  : 0x0B,
        'NDOF'          : 0x0C,
    }

    # Power modes
    PWR_NORMAL = 0x00
    PWR_LOW    = 0x01
    PWR_SUSPEND= 0x02

    def __init__(self, i2c: I2C, address: int = ADDR_A, rst_pin=None, boot_time_ms=700):
        """Create an IMU driver using an already-initialized pyb.I2C in CONTROLLER mode.
        Optionally pass a reset pin name (e.g. 'PA13') for hardware reset.
        """
        self.i2c     = i2c
        self.addr    = address
        self._mode   = None
        self._rst    = None

        # Optional hardware reset line
        if rst_pin is not None:
            self._rst = Pin(rst_pin, mode=Pin.OUT_PP)
            self._rst.high()
            utime.sleep_ms(2)
            self._rst.low()
            utime.sleep_ms(2)
            self._rst.high()

        # Give the chip time to boot (datasheet recommends ~650ms after reset)
        utime.sleep_ms(boot_time_ms)

        # Verify CHIP_ID (sometimes returns 0x00/0xFF briefly after power-up)
        for _ in range(10):
            try:
                cid = self._read8(self._REG_CHIP_ID)
                if cid == 0xA0:
                    break
            except OSError:
                pass
            utime.sleep_ms(50)
        else:
            raise RuntimeError("BNO055: CHIP_ID not found (got 0x%02X)" % (cid if 'cid' in locals() else -1))

        # Put in CONFIG to set sane defaults
        self.set_mode('CONFIG')
        self._set_page(0)
        self.set_power_mode(self.PWR_NORMAL)

        # Default unit selection: degrees for Euler, deg/s for gyro
        # UNIT_SEL bits: [0]=Accel(m/s^2), [1]=Gyro(rad/s=1|deg/s=0), [2]=Euler(rad=1|deg=0), [4]=Temp(F=1|C=0)
        # We'll ensure deg/s + deg + C:
        self._write8(self._REG_UNIT_SEL, 0x00)

        # Ready to switch to fusion mode when user calls set_mode('NDOF') or similar

    # --------------------------- Public API ---------------------------

    def set_mode(self, mode):
        """Set operating mode. Accepts string key from MODES or an int."""
        if isinstance(mode, str):
            if mode not in self.MODES:
                raise ValueError("Unknown mode '%s'" % mode)
            mode_val = self.MODES[mode]
        else:
            mode_val = int(mode) & 0xFF

        # If changing to anything other than CONFIG, switch to CONFIG first
        if mode_val != self.MODES['CONFIG']:
            self._write8(self._REG_OPR_MODE, self.MODES['CONFIG'])
            utime.sleep_ms(30)

        self._write8(self._REG_OPR_MODE, mode_val)
        utime.sleep_ms(30)  # allow time to change mode
        self._mode = mode_val

    def get_mode(self):
        """Return current OPR_MODE register value."""
        return self._read8(self._REG_OPR_MODE)

    def set_power_mode(self, pwr_mode=PWR_NORMAL):
        """Set power mode (NORMAL/LOW/SUSPEND). Requires CONFIG mode."""
        was = self.get_mode()
        if was != self.MODES['CONFIG']:
            self.set_mode('CONFIG')
        self._write8(self._REG_PWR_MODE, pwr_mode & 0x03)
        utime.sleep_ms(10)
        if was != self.MODES['CONFIG']:
            self._write8(self._REG_OPR_MODE, was)
            utime.sleep_ms(30)

    def get_calibration_status(self):
        """Return dict with calibration levels (0..3) for sys, gyro, accel, mag, plus raw byte."""
        b = self._read8(self._REG_CALIB_STAT)
        # Bits: SYS[7:6], GYR[5:4], ACC[3:2], MAG[1:0]
        status = {
            'sys'  : (b >> 6) & 0x03,
            'gyro' : (b >> 4) & 0x03,
            'accel': (b >> 2) & 0x03,
            'mag'  : (b >> 0) & 0x03,
            'raw'  : b,
        }
        return status

    def dump_calibration(self):
        """Read and return the 22-byte calibration blob (offsets + radii).
        Call this after the sensor reports fully calibrated (3,3,3,3).
        """
        was = self.get_mode()
        if was != self.MODES['CONFIG']:
            self.set_mode('CONFIG')
        self._set_page(0)
        blob = self._read_bytes(self._REG_CALIB_START, self._CALIB_LEN)
        # Restore previous mode
        if was != self.MODES['CONFIG']:
            self._write8(self._REG_OPR_MODE, was); utime.sleep_ms(30)
        return blob

    def load_calibration(self, blob: bytes):
        """Write a 22-byte calibration blob previously captured by dump_calibration()."""
        if not isinstance(blob, (bytes, bytearray)) or len(blob) != self._CALIB_LEN:
            raise ValueError("Calibration blob must be 22 bytes")
        was = self.get_mode()
        if was != self.MODES['CONFIG']:
            self.set_mode('CONFIG')
        self._set_page(0)
        self._write_bytes(self._REG_CALIB_START, blob)
        utime.sleep_ms(25)
        if was != self.MODES['CONFIG']:
            self._write8(self._REG_OPR_MODE, was); utime.sleep_ms(30)

    def read_euler(self):
        """Return (heading, roll, pitch) in degrees. 1/16 deg per LSB."""
        data = self._read_bytes(self._REG_EULER_H, 6)
        h, r, p = struct.unpack('<hhh', data)
        scale = 1.0 / 16.0
        return (h * scale, r * scale, p * scale)

    def heading(self):
        """Return heading (yaw) in degrees."""
        data = self._read_bytes(self._REG_EULER_H, 2)
        (h,) = struct.unpack('<h', data)
        return h / 16.0

    def read_gyro(self):
        """Return angular rates (gx, gy, gz) in deg/s. 16 LSB per dps."""
        data = self._read_bytes(self._REG_GYRO_DATA, 6)
        gx, gy, gz = struct.unpack('<hhh', data)
        scale = 1.0 / 16.0
        return (gx * scale, gy * scale, gz * scale)

    def yaw_rate(self):
        """Return yaw rate (gz) in deg/s."""
        data = self._read_bytes(self._REG_GYRO_DATA + 4, 2)  # Z only
        (gz,) = struct.unpack('<h', data)
        return gz / 16.0

    # --------------------------- Internals ---------------------------

    def _set_page(self, page: int):
        self._write8(self._REG_PAGE_ID, page & 0x01)
        utime.sleep_ms(1)

    def _read8(self, reg):
        buf = bytearray(1)
        self.i2c.mem_read(buf, self.addr, reg, addr_size=8)
        return buf[0]

    def _write8(self, reg, val):
        self.i2c.mem_write(bytes((val & 0xFF,)), self.addr, reg, addr_size=8)

    def _read_bytes(self, reg, n):
        buf = bytearray(n)
        self.i2c.mem_read(buf, self.addr, reg, addr_size=8)
        return bytes(buf)

    def _write_bytes(self, reg, data):
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("data must be bytes-like")
        # Some ports require chunking; 22 bytes is fine in one go for pyb I2C
        self.i2c.mem_write(data, self.addr, reg, addr_size=8)