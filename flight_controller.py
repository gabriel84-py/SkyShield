from pid import PID
from esc_motor import ESC
from mpu6050 import MPU6050
import time

class FlightController:
    """Contrôleur vol drone - gère PID Roll/Pitch + mixage moteurs"""
    
    def __init__(self):

        # === CAPTEURS ===
        print("Init MPU6050...")
        self.mpu = MPU6050(scl=27, sda=26, addr=0x68)

        # === MOTEURS (config X) ===
        print("Init moteurs...")
        self.m1 = ESC(pin=2)   # front right (CW)
        self.m2 = ESC(pin=3)   # rear  right (CCW)
        self.m3 = ESC(pin=4)   # rear  left  (CW)
        self.m4 = ESC(pin=5)   # front left  (CCW)

        # === PID CONTROLLERS ===
        print("Init PID...")
        self.pid_roll = PID(
            kp=0.8,
            ki=0.00,   # Ki=0 pour le banc test, à augmenter après
            kd=0.25,
            output_limits=(-25, 25),
            integral_limit=10
        )
        self.pid_pitch = PID(
            kp=0.8,
            ki=0.00,
            kd=0.25,
            output_limits=(-25, 25),
            integral_limit=10
        )

        # === ALTITUDE VL53L0X (désactivé par défaut) ===
        self.use_altitude = False
        try:
            from vl53l0x import VL53L0X
            self.altitude_sensor = VL53L0X(i2c_id=0, scl=21, sda=20, addr=0x29)
            print("VL53L0X OK (désactivé — mettre use_altitude=True pour l'activer)")
        except Exception as e:
            self.altitude_sensor = None
            if self.use_altitude:
                print(f"VL53L0X non dispo ({e}) → mode throttle manuel forcé")
                self.use_altitude = False

        self.pid_altitude = PID(kp=50, ki=5, kd=20, output_limits=(-30, 30))
        self.target_altitude = 1.0

        # === CIBLES PID ===
        # Initialisées à 0 puis remplacées par l'auto-calibration dans arm()
        self.target_roll  = 0.0
        self.target_pitch = 0.0

        # === THROTTLE ===
        self.base_throttle = 0.0

        # FIX 4 : Seuil physique de démarrage moteur
        # Les moteurs ne tournent qu'à partir de 15% → on ne leur envoie
        # jamais de valeur entre 0 et MIN_MOTOR_THROTTLE
        # Si la valeur calculée est en dessous → on coupe (0), pas on réduit
        self.MIN_MOTOR_THROTTLE = 15.0   # % en dessous duquel le moteur est silencieux
        self.max_throttle       = 30.0   # limite sécurité banc test

        # FIX 1 : Deadzone radio
        # La radio envoie 3.5-11% même à la position repos (observé en CSV)
        # Tout signal sous ce seuil est traité comme 0 → aucun moteur ne tourne
        self.RADIO_DEADZONE = 12.0       # % → tout ce qui est < est traité comme 0

        # === SÉCURITÉS ===
        self.max_angle      = 30.0
        self.armed          = False
        self.emergency_stop = False

        # === STATS ===
        self.loop_count     = 0
        self.last_loop_time = time.ticks_us()
        self.loop_rate      = 0.0

        # Mémo throttle pour reset PID à la remise des gaz
        self._prev_throttle = 0.0

        print("FlightController prêt !")

    # ─────────────────────────────────────────────────────────────
    def _auto_calibrate_imu(self, samples=150, delay_ms=10):
        """
        FIX 3 : Mesure les angles IMU au repos pendant ~1.5s et les
        utilise comme cibles PID. Compense l'offset de montage du MPU6050.
        
        Sans ça : le PID voit roll=+7.6° pitch=-8.3° comme une erreur
        permanente → corrections constantes même au repos → M3 tourne seul.
        """
        print("Auto-calibration IMU (ne pas bouger le drone)...")
        sum_roll  = 0.0
        sum_pitch = 0.0
        for i in range(samples):
            r, p, _ = self.mpu.read_angles()
            sum_roll  += r
            sum_pitch += p
            time.sleep_ms(delay_ms)
        self.target_roll  = sum_roll  / samples
        self.target_pitch = sum_pitch / samples
        print(f"Offset IMU mesuré → target_roll={self.target_roll:.2f}°  target_pitch={self.target_pitch:.2f}°")

    # ─────────────────────────────────────────────────────────────
    def arm(self):
        """Arme les ESC puis auto-calibre l'IMU"""
        print("\n=== ARMEMENT ESC ===")
        self.m1.arm()
        self.m2.arm()
        self.m3.arm()
        self.m4.arm()

        # reset PID avant calibration
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_altitude.reset()
        self._prev_throttle = 0.0

        # FIX 3 : auto-calibration offset IMU
        self._auto_calibrate_imu()

        self.armed = True
        print("ESC armés !\n")

    def disarm(self):
        """Désarme — arrêt moteurs"""
        self.m1.stop()
        self.m2.stop()
        self.m3.stop()
        self.m4.stop()
        self.armed = False
        print("ESC désarmés")

    def emergency(self):
        """Arrêt d'urgence"""
        self.emergency_stop = True
        self.disarm()
        print("\n!!! EMERGENCY STOP !!!")

    # ─────────────────────────────────────────────────────────────
    def set_throttle(self, throttle_raw):
        """
        Reçoit le throttle brut depuis la radio (0-100%)
        
        FIX 1 : applique la deadzone radio
        Si le signal est sous RADIO_DEADZONE → traité comme 0
        Évite que les 3.5-11% parasites de la radio allument les moteurs
        
        FIX 4 (partiel) : reset PID quand on remet les gaz après avoir coupé
        Évite l'accumulation d'intégrale au sol qui crée un départ asymétrique
        """
        # Deadzone radio
        if throttle_raw < self.RADIO_DEADZONE:
            new_throttle = 0.0
        else:
            new_throttle = max(0.0, min(self.max_throttle, throttle_raw))

        # Reset PID à la remise des gaz (était à 0, repart)
        if new_throttle > 0 and self._prev_throttle == 0:
            self.pid_roll.reset()
            self.pid_pitch.reset()
            print(f"PID reset — remise des gaz ({new_throttle:.1f}%)")

        # Reset PID à la coupure gaz (évite intégrale fantôme au sol)
        if new_throttle == 0 and self._prev_throttle > 0:
            self.pid_roll.reset()
            self.pid_pitch.reset()

        self._prev_throttle = new_throttle
        self.base_throttle  = new_throttle

    # ─────────────────────────────────────────────────────────────
    def set_pid_gains(self, axis, kp=None, ki=None, kd=None):
        """Change gains PID en direct (tuning live)"""
        if axis == 'roll':
            self.pid_roll.set_gains(kp, ki, kd)
        elif axis == 'pitch':
            self.pid_pitch.set_gains(kp, ki, kd)
        elif axis == 'altitude':
            self.pid_altitude.set_gains(kp, ki, kd)

    def check_safety(self, roll, pitch):
        """Coupe si angle trop grand"""
        if abs(roll - self.target_roll) > self.max_angle or abs(pitch - self.target_pitch) > self.max_angle:
            print(f"ANGLE LIMITE ! Roll:{roll:.1f}° Pitch:{pitch:.1f}° → EMERGENCY")
            self.emergency()
            return False
        return True

    # ─────────────────────────────────────────────────────────────
    def mix_motors(self, throttle, corr_roll, corr_pitch):
        """
        Mixage config X standard
        
          M4(CCW) --- M1(CW)
              |   X   |
          M3(CW)  --- M2(CCW)
        
        Roll+  = penche à droite → augmente M3/M4 (gauche), diminue M1/M2
        Pitch+ = penche en avant → augmente M1/M2 (avant), diminue M3/M4
        
        FIX 4 : seuil physique moteur
        On ne retourne JAMAIS une valeur entre 0 et MIN_MOTOR_THROTTLE.
        Si la valeur calculée est dans cette zone morte → on coupe à 0.
        Sinon le PID croit corriger sur des moteurs actifs alors qu'ils
        sont silencieux → déséquilibre impossible à rattraper.
        """
        # throttle nul → tout à 0, même si le PID veut corriger
        if throttle <= 0:
            return 0, 0, 0, 0

        def apply_deadband(val):
            """0 si sous le seuil physique, sinon valeur clampée"""
            v = max(0.0, min(100.0, val))
            if v < self.MIN_MOTOR_THROTTLE:
                return 0.0
            return v

        m1 = apply_deadband(throttle + corr_roll - corr_pitch)  # front right
        m2 = apply_deadband(throttle + corr_roll + corr_pitch)  # rear  right
        m3 = apply_deadband(throttle - corr_roll + corr_pitch)  # rear  left
        m4 = apply_deadband(throttle - corr_roll - corr_pitch)  # front left

        return m1, m2, m3, m4

    # ─────────────────────────────────────────────────────────────
    def update(self):
        """
        Boucle principale de contrôle — appeler à ~100 Hz
        Retourne : (roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch)
        """
        roll, pitch, yaw = self.mpu.read_angles()

        if not self.check_safety(roll, pitch):
            return roll, pitch, 0, 0, 0, 0, 0, 0

        if not self.armed or self.emergency_stop:
            return roll, pitch, 0, 0, 0, 0, 0, 0

        # corrections PID (cibles = offset IMU mesuré au repos)
        corr_roll  = self.pid_roll.compute(self.target_roll,  roll)
        corr_pitch = self.pid_pitch.compute(self.target_pitch, pitch)

        # throttle effectif
        if self.use_altitude and self.altitude_sensor is not None:
            altitude = self.altitude_sensor.distance_avg(samples=2)
            if altitude is not None and altitude < 2.5:
                corr_alt = self.pid_altitude.compute(self.target_altitude, altitude)
                throttle = self.base_throttle + corr_alt
            else:
                throttle = self.base_throttle
        else:
            throttle = self.base_throttle

        throttle = max(0.0, min(self.max_throttle, throttle))

        # mixage avec seuil physique moteur
        m1, m2, m3, m4 = self.mix_motors(throttle, corr_roll, corr_pitch)

        # applique aux ESC
        self.m1.throttle(m1)
        self.m2.throttle(m2)
        self.m3.throttle(m3)
        self.m4.throttle(m4)

        # stats boucle
        self.loop_count += 1
        if self.loop_count % 100 == 0:
            now     = time.ticks_us()
            elapsed = time.ticks_diff(now, self.last_loop_time) / 1_000_000
            self.loop_rate      = 100 / elapsed if elapsed > 0 else 0
            self.last_loop_time = now

        return roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch

    def get_pid_terms(self, axis):
        """Retourne termes P, I, D séparés pour debug/logging"""
        if axis == 'roll':
            return self.pid_roll.get_terms()
        elif axis == 'pitch':
            return self.pid_pitch.get_terms()
        return 0.0, 0.0, 0.0