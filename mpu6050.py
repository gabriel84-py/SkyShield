from machine import I2C, Pin
import math
import time

class MPU6050:
    """Classe lire données capteur MPU6050"""
    
    def __init__(self, i2c_id=1, scl=27, sda=26, addr=0x70):
        """
        Initialise le capteur MPU6050
        - i2c_id: numéro bus I2C (0 / 1)
        - scl: pin horloge I2C
        - sda: pin données I2C
        - addr: adresse I2C capteur (0x68 défaut)
        """
        # Création bus I2C
        #print('IC2 ID:', i2c_id, 'SCL:', scl, 'SDA:', sda) = pr li debuuuuuuug héhéhé
        self.i2c = I2C(
            i2c_id,
            scl=Pin(scl),
            sda=Pin(sda),
            freq=100_000
        )
        
        # Sauvegarde adresse capteur
        self.addr = addr
        
        # Réveiller capteur (lu démre en mooooode veil)
        self._wake()
        
        # Variables pour stocker les angles calculés (ps : il sert a rien ce commentaire on est d'accord ?)
        self.roll = 0.0   # Rotation gauche/droite
        self.pitch = 0.0  # Rotation avant/arrière
        self.yaw = 0.0    # Rotation autour de l'axe vertical

        self.off_ax, self.off_ay, self.off_az, self.off_gx, self.off_gy, self.off_gz = self.calibrate()
        
        # Mémoriser temps calcule angles
        self.last_time = time.ticks_us()
    
    def _wake(self):
        """Réveille le capteur en écrivant 0 dans registre alimentation"""
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
    
    def _to_int16(self, h, l):
        """
        Convertit 2 octets --> nombre signé 16 bits
        - h: octet de poids fort
        - l: octet de poids faible
        """
        # Combine les 2 octets
        v = (h << 8) | l
        # Si le nombre est négatif (> 32767), le convertir
        return v - 65536 if v > 32767 else v
    
    def read_raw(self):
        """
        Lit les valeurs brutes du capteur
        Retourne: ax, ay, az (accélération), gx, gy, gz (rotation)
        """
        # Lire 14 octets à partir du registre 0x3B
        # (6 pour accéléromètre + 2 température + 6 pour gyroscope)
        d = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        
        # Convertir les octets en valeurs d'accélération (X, Y, Z)
        ax = self._to_int16(d[0], d[1])
        ay = self._to_int16(d[2], d[3])
        az = self._to_int16(d[4], d[5]) # +16384 pour 1g si capteur à plat c'est physique sido....
        
        # Convertir les octets en valeurs de rotation (X, Y, Z)
        gx = self._to_int16(d[8], d[9])
        gy = self._to_int16(d[10], d[11])
        gz = self._to_int16(d[12], d[13])
        
        return ax - self.off_ax, ay - self.off_ay, az - self.off_az, gx - self.off_gx, gy - self.off_gy, gz - self.off_gz
    
    def read_raw_for_calib(self):
        """
        Lit les valeurs brutes du capteur
        Retourne: ax, ay, az (accélération), gx, gy, gz (rotation)
        """
        # Lire 14 octets à partir du registre 0x3B
        # (6 pour accéléromètre + 2 température + 6 pour gyroscope)
        d = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        
        # Convertir les octets en valeurs d'accélération (X, Y, Z)
        ax = self._to_int16(d[0], d[1])
        ay = self._to_int16(d[2], d[3])
        az = self._to_int16(d[4], d[5]) # +16384 pour 1g si capteur à plat c'est physique sido....
        
        # Convertir les octets en valeurs de rotation (X, Y, Z)
        gx = self._to_int16(d[8], d[9])
        gy = self._to_int16(d[10], d[11])
        gz = self._to_int16(d[12], d[13])
        
        return ax , ay, az, gx, gy, gz
    
    def read_angles(self):
        """
        Calcule les angles roll (roulis), pitch (tangage) et yaw (lacet)
        en combinant gyroscope et accéléromètre
        Retourne: roll, pitch, yaw (en degrés)
        """
        # Lire les valeurs brutes
        ax, ay, az, gx, gy, gz = self.read_raw()
        
        # Convertir le gyroscope en degrés/seconde
        # (131 = facteur de conversion du MPU6050)
        gx = gx / 131.0
        gy = gy / 131.0
        gz = gz / 131.0  # Ajout du gz pour le yaw
        
        # Calculer les angles depuis l'accéléromètre
        # atan2 = arctangente qui gère les 4 quadrants
        acc_roll = math.atan2(ay, az) * 180 / math.pi
        acc_pitch = math.atan2(-ax, az) * 180 / math.pi
        
        # Calculer le temps écoulé depuis la dernière lecture
        now = time.ticks_us()
        dt = time.ticks_diff(now, self.last_time) / 1_000_000  # en secondes
        self.last_time = now
        
        # Filtre complémentaire (fusion gyro + accel)
        # 98% gyroscope (précis court terme) + 2% accéléromètre (précis long terme)
        alpha = 0.98
        self.roll = alpha * (self.roll + gx * dt) + (1 - alpha) * acc_roll
        self.pitch = alpha * (self.pitch + gy * dt) + (1 - alpha) * acc_pitch
        
        # Pour le yaw, on utilise uniquement le gyroscope
        # (l'accéléromètre ne peut pas mesurer la rotation autour de la verticale)
        self.yaw += gz * dt
        
        return self.roll, self.pitch, self.yaw
    
    def calibrate(self, samples=1000):
        
        """
        Docstring for calibrate
        
        :param samples: nombre prélèvements effectués
        """

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