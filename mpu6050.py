from machine import I2C, Pin
import math
import time

class KalmanFilter:
    """Filtre Kalman 1D pr lisser data MPU - réduit bruit/vibrations chq axe"""
    
    def __init__(self, process_noise=0.01, measurement_noise=0.1, estimation_error=1.0):
        """
        Init filtre Kalman
        
        Params :
        - process_noise (Q) : bruit proc (+ élevé = suit mieux chgmts rapides)
        - measurement_noise (R) : bruit mes (+ élevé = lisse + mais réagit lent)
        - estimation_error (P) : erreur estim init
        
        Règles réglage :
        - Vibrations fortes : ↑ R (0.5 → 1.0)
        - Chgmts rapides : ↑ Q (0.05 → 0.1)
        - Lissage max : ↓ Q (0.001), ↑ R (1.0)
        """
        self.Q = process_noise       # bruit proc
        self.R = measurement_noise   # bruit mes
        self.P = estimation_error    # erreur estim
        self.X = 0.0                 # estim actuelle
        self.K = 0.0                 # gain Kalman
    
    def update(self, measurement):
        """MAJ filtre av new mes"""
        # prédiction (suppose val stable)
        self.P = self.P + self.Q
        
        # calc gain Kalman (K détermine confiance mes vs estim)
        self.K = self.P / (self.P + self.R)
        
        # MAJ estim (corrige av new mes)
        self.X = self.X + self.K * (measurement - self.X)
        
        # MAJ erreur estim
        self.P = (1 - self.K) * self.P
        
        return self.X
    
    def reset(self, value=0.0):
        """Reset filtre av val départ"""
        self.X = value
        self.P = 1.0


class KalmanFilterIMU:
    """6 filtres Kalman pr filtrer ttes data MPU"""
    
    def __init__(self, accel_noise=0.5, gyro_noise=0.05):
        """
        Init 6 filtres (3 accel + 3 gyro)
        
        Params :
        - accel_noise : bruit accel (def 0.1)
        - gyro_noise : bruit gyro (def 0.01)
        """
        # filtres pr accel (+ bruité)
        self.kalman_ax = KalmanFilter(process_noise=0.01, measurement_noise=accel_noise)
        self.kalman_ay = KalmanFilter(process_noise=0.01, measurement_noise=accel_noise)
        self.kalman_az = KalmanFilter(process_noise=0.01, measurement_noise=accel_noise)
        
        # filtres pr gyro (- bruité)
        self.kalman_gx = KalmanFilter(process_noise=0.001, measurement_noise=gyro_noise)
        self.kalman_gy = KalmanFilter(process_noise=0.001, measurement_noise=gyro_noise)
        self.kalman_gz = KalmanFilter(process_noise=0.001, measurement_noise=gyro_noise)
    
    def filter_data(self, ax, ay, az, gx, gy, gz):
        """
        Filtre ttes data MPU6050
        
        Params : ax, ay, az (accel brutes), gx, gy, gz (rot brutes)
        Ret : ax, ay, az, gx, gy, gz filtrés (sans bruit)
        """
        # filtre chq axe indép
        ax_filtered = self.kalman_ax.update(ax)
        ay_filtered = self.kalman_ay.update(ay)
        az_filtered = self.kalman_az.update(az)
        
        gx_filtered = self.kalman_gx.update(gx)
        gy_filtered = self.kalman_gy.update(gy)
        gz_filtered = self.kalman_gz.update(gz)
        
        return ax_filtered, ay_filtered, az_filtered, gx_filtered, gy_filtered, gz_filtered

class MPU6050:
    """Classe pr lire data capt MPU6050"""
    
    def __init__(self, i2c_id=1, scl=27, sda=26, addr=0x70):
        """
        Init capt MPU6050
        - i2c_id: num bus I2C (0/1)
        - scl: pin horloge I2C
        - sda: pin data I2C
        - addr: adresse I2C capt (0x68 def)
        """
        # création bus I2C
        self.i2c = I2C(
            i2c_id,
            scl=Pin(scl),
            sda=Pin(sda),
            freq=100_000
        )
        
        # save addr capt
        self.addr = addr
        
        # réveille capt (démarre en mode veille)
        self._wake()
        
        # vars pr stocker angles calc
        self.roll = 0.0   # rot gauche/droite
        self.pitch = 0.0  # rot avant/arrière
        self.yaw = 0.0    # rot axe vertical

        # init filtre Kalman
        self.kalman = KalmanFilterIMU(accel_noise=0.1, gyro_noise=0.01)

        # calibration auto
        self.off_ax, self.off_ay, self.off_az, self.off_gx, self.off_gy, self.off_gz = self.calibrate()
        
        # mémo temps pr calc angles
        self.last_time = time.ticks_us()
    
    def _wake(self):
        """Réveille capt en écrivant 0 ds registre alim"""
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
    
    def _to_int16(self, h, l):
        """
        Convert 2 octets → nb signé 16 bits
        - h: octet poids fort
        - l: octet poids faible
        """
        # combine 2 octets
        v = (h << 8) | l
        # si nb négatif (> 32767), convert
        return v - 65536 if v > 32767 else v
    
    def read_raw(self):
        """
        Lit val brutes capt
        Ret: ax, ay, az (accel), gx, gy, gz (rot) - offsets calib
        """
        # lit 14 octets dp registre 0x3B (6 accel + 2 temp + 6 gyro)
        d = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        
        # convert octets → val accel (X, Y, Z)
        ax = self._to_int16(d[0], d[1])
        ay = self._to_int16(d[2], d[3])
        az = self._to_int16(d[4], d[5])
        
        # convert octets → val rot (X, Y, Z)
        gx = self._to_int16(d[8], d[9])
        gy = self._to_int16(d[10], d[11])
        gz = self._to_int16(d[12], d[13])
        
        # ret val brutes - offsets
        return ax - self.off_ax, ay - self.off_ay, az - self.off_az, gx - self.off_gx, gy - self.off_gy, gz - self.off_gz
    
    def read_raw_for_calib(self):
        """
        Lit val brutes capt (SANS offsets pr calib)
        Ret: ax, ay, az (accel), gx, gy, gz (rot)
        """
        # lit 14 octets dp registre 0x3B (6 accel + 2 temp + 6 gyro)
        d = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        
        # convert octets → val accel (X, Y, Z)
        ax = self._to_int16(d[0], d[1])
        ay = self._to_int16(d[2], d[3])
        az = self._to_int16(d[4], d[5])  # +16384 pr 1g si capt plat (physique)
        
        # convert octets → val rot (X, Y, Z)
        gx = self._to_int16(d[8], d[9])
        gy = self._to_int16(d[10], d[11])
        gz = self._to_int16(d[12], d[13])
        
        return ax, ay, az, gx, gy, gz
    
    def read_angles(self):
        """
        Calc angles roll (roulis), pitch (tangage), yaw (lacet) en combinant gyro + accel
        Ret: roll, pitch, yaw (en degrés)
        """
        # lit val brutes
        ax, ay, az, gx, gy, gz = self.read_raw()
        
        # filtre av Kalman (réduit bruit)
        ax, ay, az, gx, gy, gz = self.kalman.filter_data(ax, ay, az, gx, gy, gz)
        
        # convert gyro en °/s (131 = facteur conversion MPU6050)
        gx = gx / 131.0
        gy = gy / 131.0
        gz = gz / 131.0
        
        # calc angles dp accel (atan2 gère 4 quadrants)
        acc_roll = math.atan2(ay, az) * 180 / math.pi
        acc_pitch = math.atan2(-ax, az) * 180 / math.pi
        
        # calc temps écoulé dp dern lecture
        now = time.ticks_us()
        dt = time.ticks_diff(now, self.last_time) / 1_000_000  # en sec
        self.last_time = now
        
        # filtre complémentaire (fusion gyro + accel)
        # 98% gyro (précis court terme) + 2% accel (précis long terme)
        alpha = 0.98
        self.roll = alpha * (self.roll + gx * dt) + (1 - alpha) * acc_roll
        self.pitch = alpha * (self.pitch + gy * dt) + (1 - alpha) * acc_pitch
        
        # pr yaw, utilise uniquement gyro (accel peut pas mesurer rot verticale)
        self.yaw += gz * dt
        
        return self.roll, self.pitch, self.yaw
    
    def calibrate(self, samples=1000):
        """
        Calibre capt en calc offsets
        :param samples: nb échantillons pr moyenne
        """
        # sommes pr calc moyennes
        sum_gx = 0
        sum_gy = 0
        sum_gz = 0
        sum_ax = 0
        sum_ay = 0
        sum_az = 0

        # lecture N échantillons
        for _ in range(samples):
            ax, ay, az, gx, gy, gz = self.read_raw_for_calib()
            sum_gx += gx
            sum_gy += gy
            sum_gz += gz
            sum_ax += ax
            sum_ay += ay
            sum_az += az
        
        # calc offsets (moyennes)
        offset_gx = sum_gx/samples
        offset_gy = sum_gy/samples
        offset_gz = sum_gz/samples
        offset_ax = sum_ax/samples
        offset_ay = sum_ay/samples
        offset_az = (sum_az/samples) - 16384  # -16384 car capt plat = 1g

        # affiche offsets
        print(f"ax : {offset_ax}")
        print(f"ay : {offset_ay}")
        print(f"az : {offset_az}")
        print(f"gx : {offset_gx}")
        print(f"gy : {offset_gy}")
        print(f"gz : {offset_gz}")

        return offset_ax, offset_ay, offset_az, offset_gx, offset_gy, offset_gz

    def reset_yaw(self):
        """Reset yaw"""
        self.yaw = 0.0
        print("Yaw reseted")