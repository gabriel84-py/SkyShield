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
        # RP2040: GPIO 2 et 3 partagent le slice PWM 1 → moteur 2 sur GPIO 6 (slice dédié)
        print("Init moteurs...")
        self.m1 = ESC(pin=2)   # front right (CW)  - slice 1A
        self.m2 = ESC(pin=6)   # rear  right (CCW) - slice 3A (évite conflit pin 3)
        self.m3 = ESC(pin=4)   # rear  left  (CW)   - slice 2A
        self.m4 = ESC(pin=8)   # front left  (CCW) - slice 2B

        # === PID ===
        print("Init PID...")
        self.pid_roll = PID(
            kp=0.8, ki=0.00, kd=0.25,
            output_limits=(-25, 25), integral_limit=10
        )
        self.pid_pitch = PID(
            kp=0.8, ki=0.00, kd=0.25,
            output_limits=(-25, 25), integral_limit=10
        )

        # === ALTITUDE VL53L0X (désactivé) ===
        self.use_altitude = False
        try:
            from vl53l0x import VL53L0X
            self.altitude_sensor = VL53L0X(i2c_id=0, scl=21, sda=20, addr=0x29)
            print("VL53L0X OK (désactivé)")
        except Exception as e:
            self.altitude_sensor = None
        self.pid_altitude  = PID(kp=50, ki=5, kd=20, output_limits=(-30, 30))
        self.target_altitude = 1.0

        # === CIBLES PID (seront remplacées par auto-calib) ===
        self.target_roll  = 0.0
        self.target_pitch = 0.0

        # ── THROTTLE ─────────────────────────────────────────────
        self.base_throttle = 0.0
        self.max_throttle  = 30.0   # limite banc test

        # Seuil physique moteur : en dessous les moteurs sont silencieux.
        # On ne COUPE PAS (apply_deadband) mais on SNAP au minimum
        # pour que dès que le throttle est > 0, les moteurs tournent vraiment.
        self.MIN_MOTOR = 10.0       # % minimum pour qu'un moteur tourne

        # ── DEADZONE RADIO avec HYSTÉRÉSIS ───────────────────────
        # Problème : radio envoie 3.5-11% au repos.
        # Si seuil unique = 12% : au moindre bruit la radio oscille
        # entre 11% (→0) et 12% (→actif) → bips ESC en boucle.
        #
        # Solution : deux seuils différents
        #   DEAD_ON  : seuil pour ACTIVER  (montée de gaz)
        #   DEAD_OFF : seuil pour DÉSACTIVER (descente de gaz)
        #   DEAD_OFF < DEAD_ON  → zone tampon entre les deux
        self.DEAD_ON  = 14.0   # radio doit dépasser 14% pour activer
        self.DEAD_OFF = 11.5    # radio doit descendre sous X% pour couper
        self._throttle_active = False   # état hystérésis

        # === SÉCURITÉS ===
        self.max_angle      = 45.0
        self.armed          = False
        self.emergency_stop = False

        # === STATS ===
        self.loop_count     = 0
        self.last_loop_time = time.ticks_us()
        self.loop_rate      = 0.0
        self._prev_throttle = 0.0

        # pour eviter les throtles bizares
        self.last_valid_throttle = 0.0
        self.last_radio_time = time.ticks_ms()
        self.radio_timeout_ms = 300


        print("FlightController prêt !")

    # ─────────────────────────────────────────────────────────────
    def _auto_calibrate_imu(self, samples=150, delay_ms=10):
        """
        Mesure les angles IMU au repos → devient la cible du PID.
        Compense l'offset de montage (roll/pitch non nul quand posé à plat).
        """
        print("Auto-calibration IMU — ne pas bouger le drone...")
        sum_roll = 0.0
        sum_pitch = 0.0
        for _ in range(samples):
            r, p, _ = self.mpu.read_angles()
            sum_roll  += r
            sum_pitch += p
            time.sleep_ms(delay_ms)
        self.target_roll  = sum_roll  / samples
        self.target_pitch = sum_pitch / samples
        print(f"  → target_roll={self.target_roll:.2f}°  target_pitch={self.target_pitch:.2f}°")

    # ─────────────────────────────────────────────────────────────
    def arm(self):
        print("\n=== ARMEMENT ESC ===")
        self.m1.arm()
        self.m2.arm()
        self.m3.arm()
        self.m4.arm()

        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_altitude.reset()
        self._prev_throttle   = 0.0
        self._throttle_active = False

        self._auto_calibrate_imu()
        self.armed = True
        print("Armement OK — prêt à voler !\n")

    def disarm(self):
        self.m1.stop()
        self.m2.stop()
        self.m3.stop()
        self.m4.stop()
        self.armed = False
        print("ESC désarmés")

    def emergency(self):
        self.emergency_stop = True
        self.disarm()
        print("\n!!! EMERGENCY STOP !!!")

    # ─────────────────────────────────────────────────────────────
    def set_throttle(self, throttle_raw):
        """
        Applique la deadzone radio avec hystérésis puis stocke le throttle.

        Hystérésis :
          - inactif → actif  : seulement si radio > DEAD_ON  (14%)
          - actif   → inactif: seulement si radio < DEAD_OFF ( 9%)
          → zone tampon 9-14% : l'état ne change pas → plus de bips ESC

        Reset PID aux transitions pour éviter l'accumulation d'intégrale.
        """
        # ── hystérésis deadzone ───────────────────────────────────
        if not self._throttle_active:
            # on était arrêté : on active seulement si on dépasse DEAD_ON
            if throttle_raw >= self.DEAD_ON:
                self._throttle_active = True
        else:
            # on était actif : on coupe seulement si on descend sous DEAD_OFF
            if throttle_raw < self.DEAD_OFF:
                self._throttle_active = False

        # ── calcul throttle effectif ─────────────────────────────
        if not self._throttle_active:
            new_throttle = 0.0
        else:
            new_throttle = max(0.0, min(self.max_throttle, throttle_raw))

        # ── reset PID aux transitions ─────────────────────────────
        if new_throttle > 0 and self._prev_throttle == 0:
            # remise des gaz : reset pour partir propre
            self.pid_roll.reset()
            self.pid_pitch.reset()

        if new_throttle == 0 and self._prev_throttle > 0:
            # coupure gaz : reset pour éviter intégrale fantôme
            self.pid_roll.reset()
            self.pid_pitch.reset()
        
        filtered = self.filter_throttle(new_throttle)
        self._prev_throttle = filtered
        self.base_throttle  = filtered

    # ─────────────────────────────────────────────────────────────
    def set_pid_gains(self, axis, kp=None, ki=None, kd=None):
        if axis == 'roll':
            self.pid_roll.set_gains(kp, ki, kd)
        elif axis == 'pitch':
            self.pid_pitch.set_gains(kp, ki, kd)
        elif axis == 'altitude':
            self.pid_altitude.set_gains(kp, ki, kd)

    def check_safety(self, roll, pitch):
        if abs(roll - self.target_roll) > self.max_angle or \
           abs(pitch - self.target_pitch) > self.max_angle:
            print(f"ANGLE LIMITE ! Roll:{roll:.1f}° Pitch:{pitch:.1f}° → EMERGENCY")
            self.emergency()
            return False
        return True

    # ─────────────────────────────────────────────────────────────
    def _snap_to_min(self, val):
        """
        Applique le seuil physique moteur en mode SNAP (pas CLAMP).

        CLAMP (ancien) : si val < MIN_MOTOR → retourne 0
          Problème : les moteurs ne démarrent jamais entre 12-15% de radio

        SNAP (nouveau) : si val > 0 et val < MIN_MOTOR → booste à MIN_MOTOR
          Résultat : dès que throttle > deadzone radio, tous les moteurs tournent
          Les corrections PID ont toujours un effet réel même à bas régime
        """
        v = max(0.0, min(100.0, val))
        if v <= 0.0:
            return 0.0
        if v < self.MIN_MOTOR:
            return self.MIN_MOTOR   # snap au minimum physique
        return v

    def mix_motors(self, throttle, corr_roll, corr_pitch):
        """
        Mixage config X :

          M4(CCW) ── M1(CW)
              |  X  |
          M3(CW) ── M2(CCW)

        Roll+  (penche droite) → augmente M3/M4 (gauche), diminue M1/M2
        Pitch+ (penche avant)  → augmente M1/M2 (avant), diminue M3/M4
        """
        if throttle <= 0:
            return 0, 0, 0, 0

        m1 = self._snap_to_min(throttle + corr_roll - corr_pitch)  # front right
        m2 = self._snap_to_min(throttle + corr_roll + corr_pitch)  # rear  right
        m3 = self._snap_to_min(throttle - corr_roll + corr_pitch)  # rear  left
        m4 = self._snap_to_min(throttle - corr_roll - corr_pitch)  # front left

        return m1, m2, m3, m4

    # ─────────────────────────────────────────────────────────────
    def update(self):
        """
        Boucle principale — appeler à ~100 Hz.
        Retourne : (roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch)
        """
        roll, pitch, yaw = self.mpu.read_angles()

        if not self.check_safety(roll, pitch):
            return roll, pitch, 0, 0, 0, 0, 0, 0

        if not self.armed or self.emergency_stop:
            return roll, pitch, 0, 0, 0, 0, 0, 0

        # corrections PID
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

        m1, m2, m3, m4 = self.mix_motors(throttle, corr_roll, corr_pitch)

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

    def filter_throttle(self, raw_throttle):
        now = time.ticks_ms()

        # Si signal non nul --> valide
        if raw_throttle > 5:
            self.last_valid_throttle = raw_throttle
            self.last_radio_time = now
            return raw_throttle

        # Si 0 mais trop court --> glitch
        if raw_throttle <= 5:
            dt = time.ticks_diff(now, self.last_radio_time)

            if dt < self.radio_timeout_ms:
                # Glitch radio --> garde ancien throttle
                return self.last_valid_throttle
            else:
                # vrai arrêt volontaire
                self.last_valid_throttle = 0
                return 0


    def get_pid_terms(self, axis):
        if axis == 'roll':
            return self.pid_roll.get_terms()
        elif axis == 'pitch':
            return self.pid_pitch.get_terms()
        return 0.0, 0.0, 0.0