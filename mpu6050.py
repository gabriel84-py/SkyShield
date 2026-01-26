from machine import I2C, Pin
import math
import time


class MPU6050:
    """Classe pour lire les données du capteur MPU6050 (gyroscope + accéléromètre)"""

    def __init__(self, i2c_id=0, scl=21, sda=20, addr=0x68):
        """
        Initialise le capteur MPU6050
        - i2c_id: numéro du bus I2C (0 ou 1)
        - scl: pin horloge I2C
        - sda: pin données I2C
        - addr: adresse I2C du capteur (0x68 par défaut)
        """
        # Création du bus I2C
        self.i2c = I2C(
            i2c_id,
            scl=Pin(scl),
            sda=Pin(sda),
            freq=400_000  # Fréquence 400kHz
        )

        # Sauvegarde de l'adresse du capteur
        self.addr = addr

        # Réveiller le capteur (il démarre en mode veille)
        self._wake()

        # Variables pour stocker les angles calculés
        self.roll = 0.0  # Rotation gauche/droite (ps sido je sais très bien que tu sais ce que c'est le roulis :p)
        self.pitch = 0.0  # Rotation avant/arrière (ps sido je sais très bien que tu sais ce que c'est le tangage :p)

        # Mémoriser le temps pour calculer les angles
        self.last_time = time.ticks_us()

    def _wake(self):
        """Réveille le capteur en écrivant 0 dans le registre d'alimentation"""
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')

    def _to_int16(self, h, l):
        """
        Convertit 2 octets en nombre signé 16 bits
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
        az = self._to_int16(d[4], d[5])

        # Convertir les octets en valeurs de rotation (X, Y, Z)
        gx = self._to_int16(d[8], d[9])
        gy = self._to_int16(d[10], d[11])
        gz = self._to_int16(d[12], d[13])

        return ax, ay, az, gx, gy, gz

    def read_angles(self):
        """
        Calcule les angles roll (roulis) et pitch (tangage)
        en combinant gyroscope et accéléromètre
        Retourne: roll, pitch (en degrés)
        """
        # Lire les valeurs brutes
        ax, ay, az, gx, gy, gz = self.read_raw()

        # Convertir le gyroscope en degrés/seconde
        # (131 = facteur de conversion du MPU6050)
        gx = gx / 131.0
        gy = gy / 131.0

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

        return self.roll, self.pitch