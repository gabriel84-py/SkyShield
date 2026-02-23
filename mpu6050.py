from machine import I2C, Pin
import math
import time

class BiquadFilter:
    """Filtre biquad generique (low-pass, notch)"""
    
    def __init__(self, filter_type, sample_rate, cutoff_freq, q_factor=0.707):
        """
        filter_type: 'lowpass' ou 'notch'
        sample_rate: freq echantillonnage (Hz) - ex 100 pour 100Hz loop
        cutoff_freq: freq coupure (Hz)
        q_factor: qualite filtre (0.707 = Butterworth)
        """
        self.filter_type = filter_type
        self.sample_rate = sample_rate
        self.cutoff_freq = cutoff_freq
        self.q = q_factor
        
        # calcul coefficients
        self._compute_coefficients()
        
        # historique entrees/sorties
        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0
    
    def _compute_coefficients(self):
        """Calcul coefficients biquad"""
        omega = 2.0 * math.pi * self.cutoff_freq / self.sample_rate
        
        # limite omega pour stabilite
        if omega >= math.pi:
            omega = math.pi * 0.99
        
        sin_omega = math.sin(omega)
        cos_omega = math.cos(omega)
        alpha = sin_omega / (2.0 * self.q)
        
        if self.filter_type == 'lowpass':
            # low-pass
            b0 = (1.0 - cos_omega) / 2.0
            b1 = 1.0 - cos_omega
            b2 = (1.0 - cos_omega) / 2.0
            a0 = 1.0 + alpha
            a1 = -2.0 * cos_omega
            a2 = 1.0 - alpha
            
        elif self.filter_type == 'notch':
            # notch (band-stop)
            b0 = 1.0
            b1 = -2.0 * cos_omega
            b2 = 1.0
            a0 = 1.0 + alpha
            a1 = -2.0 * cos_omega
            a2 = 1.0 - alpha
        
        # normalise par a0
        self.b0 = b0 / a0
        self.b1 = b1 / a0
        self.b2 = b2 / a0
        self.a1 = a1 / a0
        self.a2 = a2 / a0
    
    def update(self, x):
        """Applique filtre sur nouvelle valeur"""
        # protection contre valeurs aberrantes
        if abs(x) > 1e6:
            return self.y1 if abs(self.y1) < 1e6 else 0.0
        
        # equation differentielle biquad
        y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2
        
        # protection contre divergence
        if abs(y) > 1e6 or math.isnan(y) or math.isinf(y):
            # reset filtre si divergence
            self.reset()
            return x
        
        # maj historique
        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y
        
        return y
    
    def reset(self):
        """Reset historique"""
        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0


class GyroFilterChain:
    """Chaine filtrage complete gyro: low-pass + notch frame + notch moteur"""
    
    def __init__(self, sample_rate=100):
        """
        sample_rate: freq boucle controle (Hz) - def 100Hz
        """
        # low-pass 90 Hz (coupe vibrations hautes freq)
        self.lpf = BiquadFilter('lowpass', sample_rate, 90, q_factor=0.707)
        
        # notch frame 115 Hz (vibrations chassis)
        self.notch_frame = BiquadFilter('notch', sample_rate, 115, q_factor=5.0)
        
        # notch moteur 700 Hz (vibrations moteurs)
        self.notch_motor = BiquadFilter('notch', sample_rate, 700, q_factor=5.0)
    
    def filter(self, value):
        """Applique filtres en serie"""
        # ordre: low-pass -> notch frame -> notch moteur
        value = self.lpf.update(value)
        value = self.notch_frame.update(value)
        value = self.notch_motor.update(value)
        return value
    
    def reset(self):
        """Reset tous filtres"""
        self.lpf.reset()
        self.notch_frame.reset()
        self.notch_motor.reset()


class LowPassFilter:
    """Filtre passe-bas simple (complementaire au Kalman)"""
    
    def __init__(self, alpha=0.1):
        """
        alpha : facteur lissage (0-1)
        - 0.1 = lissage fort (lent)
        - 0.5 = lissage moyen
        - 0.9 = lissage faible (rapide)
        """
        self.alpha = alpha
        self.value = None
    
    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
    
    def reset(self):
        self.value = None


class KalmanFilter:
    """Filtre Kalman 1D pr lisser data MPU - reduit bruit/vibrations chq axe"""
    
    def __init__(self, process_noise=0.01, measurement_noise=0.1, estimation_error=1.0):
        self.Q = process_noise
        self.R = measurement_noise
        self.P = estimation_error
        self.X = 0.0
        self.K = 0.0
    
    def update(self, measurement):
        self.P = self.P + self.Q
        self.K = self.P / (self.P + self.R)
        self.X = self.X + self.K * (measurement - self.X)
        self.P = (1 - self.K) * self.P
        return self.X
    
    def reset(self, value=0.0):
        self.X = value
        self.P = 1.0


class KalmanFilterIMU:
    """6 filtres Kalman pr filtrer ttes data MPU"""
    
    def __init__(self, accel_noise=0.5, gyro_noise=0.05):
        # filtres pr accel (+ bruite)
        self.kalman_ax = KalmanFilter(process_noise=0.01, measurement_noise=accel_noise)
        self.kalman_ay = KalmanFilter(process_noise=0.01, measurement_noise=accel_noise)
        self.kalman_az = KalmanFilter(process_noise=0.01, measurement_noise=accel_noise)
        
        # filtres pr gyro (- bruite)
        self.kalman_gx = KalmanFilter(process_noise=0.001, measurement_noise=gyro_noise)
        self.kalman_gy = KalmanFilter(process_noise=0.001, measurement_noise=gyro_noise)
        self.kalman_gz = KalmanFilter(process_noise=0.001, measurement_noise=gyro_noise)
    
    def filter_data(self, ax, ay, az, gx, gy, gz):
        ax_filtered = self.kalman_ax.update(ax)
        ay_filtered = self.kalman_ay.update(ay)
        az_filtered = self.kalman_az.update(az)
        
        gx_filtered = self.kalman_gx.update(gx)
        gy_filtered = self.kalman_gy.update(gy)
        gz_filtered = self.kalman_gz.update(gz)
        
        return ax_filtered, ay_filtered, az_filtered, gx_filtered, gy_filtered, gz_filtered


class MPU6050:
    """Classe pr lire data capt MPU6050 avec filtrage anti-vibrations renforce"""
    
    def __init__(self, i2c_id=1, scl=27, sda=26, addr=0x68, lpf_alpha=0.3, sample_rate=100):
        # creation bus I2C
        self.i2c = I2C(
            i2c_id,
            scl=Pin(scl),
            sda=Pin(sda),
            freq=100_000
        )
        
        self.addr = addr
        self._wake()
        
        # vars pr stocker angles calc
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # init filtre Kalman (augmente accel_noise pr vibrations)
        self.kalman = KalmanFilterIMU(accel_noise=0.5, gyro_noise=0.05)
        
        # filtres passe-bas additionnels pr angles finaux
        self.lpf_roll = LowPassFilter(alpha=lpf_alpha)
        self.lpf_pitch = LowPassFilter(alpha=lpf_alpha)
        
        # chaines filtrage gyro (low-pass + notch frame + notch moteur)
        self.gyro_filter_x = GyroFilterChain(sample_rate=sample_rate)
        self.gyro_filter_y = GyroFilterChain(sample_rate=sample_rate)
        self.gyro_filter_z = GyroFilterChain(sample_rate=sample_rate)

        # calibration auto
        self.off_ax, self.off_ay, self.off_az, self.off_gx, self.off_gy, self.off_gz = self.calibrate()
        
        # memo temps pr calc angles
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
        
        return ax - self.off_ax, ay - self.off_ay, az - self.off_az, gx - self.off_gx, gy - self.off_gy, gz - self.off_gz
    
    def read_raw_for_calib(self):
        d = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        
        ax = self._to_int16(d[0], d[1])
        ay = self._to_int16(d[2], d[3])
        az = self._to_int16(d[4], d[5])
        
        gx = self._to_int16(d[8], d[9])
        gy = self._to_int16(d[10], d[11])
        gz = self._to_int16(d[12], d[13])
        
        return ax, ay, az, gx, gy, gz
    
    def read_angles(self):
        # lit val brutes
        ax, ay, az, gx, gy, gz = self.read_raw()
        
        # filtre av Kalman (reduit bruit)
        ax, ay, az, gx, gy, gz = self.kalman.filter_data(ax, ay, az, gx, gy, gz)
        
        # convert gyro en deg/s
        gx = gx / 131.0
        gy = gy / 131.0
        gz = gz / 131.0
        
        # applique filtres frequentiels sur gyro (low-pass + notch)
        gx = self.gyro_filter_x.filter(gx)
        gy = self.gyro_filter_y.filter(gy)
        gz = self.gyro_filter_z.filter(gz)
        
        # calc angles dp accel
        acc_roll = math.atan2(ay, az) * 180 / math.pi
        acc_pitch = math.atan2(-ax, az) * 180 / math.pi
        
        # calc temps ecoule
        now = time.ticks_us()
        dt = time.ticks_diff(now, self.last_time) / 1_000_000
        self.last_time = now
        
        # limite dt (evite valeurs aberrantes si pause longue)
        if dt > 1.0 or dt <= 0:
            dt = 0.01
        
        # filtre complementaire
        alpha = 0.98
        self.roll = alpha * (self.roll + gx * dt) + (1 - alpha) * acc_roll
        self.pitch = alpha * (self.pitch + gy * dt) + (1 - alpha) * acc_pitch
        
        # protection contre divergence
        if abs(self.roll) > 180:
            self.roll = 0.0
            self.lpf_roll.reset()
        if abs(self.pitch) > 180:
            self.pitch = 0.0
            self.lpf_pitch.reset()
        
        # filtre passe-bas additionnel sur angles finaux (reduit vibrations hautes freq)
        self.roll = self.lpf_roll.update(self.roll)
        self.pitch = self.lpf_pitch.update(self.pitch)
        
        # yaw
        self.yaw += gz * dt
        
        # wraparound yaw entre -180 et 180
        if self.yaw > 180:
            self.yaw -= 360
        elif self.yaw < -180:
            self.yaw += 360
        
        return self.pitch, self.roll, self.yaw #inversé car position de l'imu oblige...
    
    def calibrate(self, samples=1000):
        sum_gx = 0
        sum_gy = 0
        sum_gz = 0
        sum_ax = 0
        sum_ay = 0
        sum_az = 0

        for _ in range(samples):
            ax, ay, az, gx, gy, gz = self.read_raw_for_calib()
            sum_gx += gx
            sum_gy += gy
            sum_gz += gz
            sum_ax += ax
            sum_ay += ay
            sum_az += az
        
        offset_gx = sum_gx/samples
        offset_gy = sum_gy/samples
        offset_gz = sum_gz/samples
        offset_ax = sum_ax/samples
        offset_ay = sum_ay/samples
        offset_az = (sum_az/samples) - 16384

        print(f"ax : {offset_ax}")
        print(f"ay : {offset_ay}")
        print(f"az : {offset_az}")
        print(f"gx : {offset_gx}")
        print(f"gy : {offset_gy}")
        print(f"gz : {offset_gz}")

        return offset_ax, offset_ay, offset_az, offset_gx, offset_gy, offset_gz

    def reset_yaw(self):
        self.yaw = 0.0
        print("Yaw reseted")