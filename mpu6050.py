from machine import I2C, Pin
import math
import time



class MPU6050:
    def __init__(self, i2c_id=0, scl=21, sda=20, addr=0x68):
        self.i2c = I2C(
            i2c_id,
            scl=Pin(scl),
            sda=Pin(sda),
            freq=400_000
        )

        self.addr = addr
        self._wake()

        # Offsets capteurs
        self.gx_offset = 0.0
        self.gy_offset = 0.0
        self.gz_offset = 0.0

        self.ax_offset = 0.0
        self.ay_offset = 0.0

        # Angles calculés
        self.roll = 0.0
        self.pitch = 0.0

        self.last_time = time.ticks_us()

    def _wake(self):
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')

    def _to_int16(self, h, l):
        v = (h << 8) | l
        return v - 65536 if v > 32767 else v

    def read_raw(self):
        d = self.i2c.readfrom_mem(self.addr, 0x3B, 14)

        ax = self._to_int16(d[0], d[1])
        ay = self._to_int16(d[2], d[3])
        az = self._to_int16(d[4], d[5])

        gx = self._to_int16(d[8], d[9])
        gy = self._to_int16(d[10], d[11])
        gz = self._to_int16(d[12], d[13])

        return ax, ay, az, gx, gy, gz

    # -------------------------------
    # CALIBRATION PRO
    # -------------------------------
    def calibrate(self, samples=1000):
        print("IMU calibration... DO NOT MOVE")

        sum_gx = sum_gy = sum_gz = 0
        sum_ax = sum_ay = 0

        for _ in range(samples):
            ax, ay, az, gx, gy, gz = self.read_raw()

            sum_gx += gx
            sum_gy += gy
            sum_gz += gz

            sum_ax += ax
            sum_ay += ay

            time.sleep(0.002)

        self.gx_offset = sum_gx / samples
        self.gy_offset = sum_gy / samples
        self.gz_offset = sum_gz / samples

        self.ax_offset = sum_ax / samples
        self.ay_offset = sum_ay / samples

        print("Calibration done")

    # -------------------------------
    # LECTURE ANGLES
    # -------------------------------
    def read_angles(self):
        ax, ay, az, gx, gy, gz = self.read_raw()

        # Correction offsets
        gx = (gx - self.gx_offset) / 131.0
        gy = (gy - self.gy_offset) / 131.0

        ax -= self.ax_offset
        ay -= self.ay_offset

        # Accéléromètre → angles
        acc_roll = math.atan2(ay, az) * 180 / math.pi
        acc_pitch = math.atan2(-ax, az) * 180 / math.pi

        now = time.ticks_us()
        dt = time.ticks_diff(now, self.last_time) / 1_000_000
        self.last_time = now

        alpha = 0.98
        self.roll = alpha * (self.roll + gx * dt) + (1 - alpha) * acc_roll
        self.pitch = alpha * (self.pitch + gy * dt) + (1 - alpha) * acc_pitch

        return self.roll, self.pitch
