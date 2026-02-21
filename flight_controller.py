from pid import PID
from esc_motor import ESC
from mpu6050 import MPU6050
import time

class FlightController:
    """Contrôleur vol drone - gère PID Roll/Pitch + mixage moteurs"""
    
    def __init__(self):
        """Init flight controller"""
        
        # === CAPTEURS ===
        print("Init MPU6050...")
        self.mpu = MPU6050(scl=27, sda=26, addr=0x68)
        
        # === MOTEURS (config X) ===
        print("Init moteurs...")
        self.m1 = ESC(pin=2)  # front right (CW)
        self.m2 = ESC(pin=3)  # rear right (CCW)
        self.m3 = ESC(pin=4)  # rear left (CW)
        self.m4 = ESC(pin=5)  # front left (CCW)
        
        # === PID CONTROLLERS ===
        print("Init PID...")
        # Roll
        self.pid_roll = PID(
            kp=0.8,
            ki=0.00,
            kd=0.25,
            output_limits=(-25, 25),
            integral_limit=10
        )
        
        # Pitch
        self.pid_pitch = PID(
            kp=0.8,
            ki=0.00,
            kd=0.25,
            output_limits=(-25, 25),
            integral_limit=10
        )

        # === CAPTEUR ALTITUDE VL53L0X ===
        # Mettre à True pour activer le contrôle d'altitude automatique
        self.use_altitude = False  # <-- passer à True quand le VL53L0X sera utilisé

        try:
            from vl53l0x import VL53L0X
            self.altitude_sensor = VL53L0X(
                i2c_id=0,
                scl=21,
                sda=20,
                addr=0x29
            )
            print("VL53L0X OK (désactivé — mettre use_altitude=True pour l'activer)")
        except Exception as e:
            self.altitude_sensor = None
            if self.use_altitude:
                print(f"VL53L0X non dispo ({e}) → mode throttle manuel forcé")
                self.use_altitude = False

        # PID altitude
        self.pid_altitude = PID(
            kp=50,
            ki=5,
            kd=20,
            output_limits=(-30, 30)
        )
        self.target_altitude = 1.0  # 1m cible
        
        # === CIBLES ===
        self.target_roll  = 0.0
        self.target_pitch = 3.0
        
        # === THROTTLE ===
        self.base_throttle = 0
        self.min_throttle  = 15
        self.max_throttle  = 40   # limite banc test
        
        # === SÉCURITÉS ===
        self.max_angle      = 30   # réduit à 30° pour banc test (était 45°)
        self.armed          = False
        self.emergency_stop = False
        
        # === STATS ===
        self.loop_count     = 0
        self.last_loop_time = time.ticks_us()
        self.loop_rate      = 0.0
        
        print("FlightController prêt !")
    
    def arm(self):
        """Arme ESC"""
        print("\n=== ARMEMENT ESC ===")
        self.m1.arm()
        self.m2.arm()
        self.m3.arm()
        self.m4.arm()
        
        # reset PID
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_altitude.reset()
        
        self.armed = True
        print("ESC armés !\n")
    
    def disarm(self):
        """Désarme (arrêt moteurs)"""
        self.m1.stop()
        self.m2.stop()
        self.m3.stop()
        self.m4.stop()
        self.armed = False
        print("ESC désarmés")
    
    def emergency(self):
        """Arrêt urgence"""
        self.emergency_stop = True
        self.disarm()
        print("\n!!! EMERGENCY STOP !!!")
    
    def set_throttle(self, throttle):
        """Change throttle base (0-100%)"""
        new_throttle = max(0, min(self.max_throttle, throttle))

        # reset PID quand on coupe les gaz → évite accumulation intégrale au sol
        if new_throttle == 0 and self.base_throttle > 0:
            self.pid_roll.reset()
            self.pid_pitch.reset()

        self.base_throttle = new_throttle
    
    def set_pid_gains(self, axis, kp=None, ki=None, kd=None):
        """Change gains PID en direct (tuning live)"""
        if axis == 'roll':
            self.pid_roll.set_gains(kp, ki, kd)
        elif axis == 'pitch':
            self.pid_pitch.set_gains(kp, ki, kd)
        elif axis == 'altitude':
            self.pid_altitude.set_gains(kp, ki, kd)
    
    def check_safety(self, roll, pitch):
        """Vérifie sécurités — coupe si angle trop grand"""
        if abs(roll) > self.max_angle or abs(pitch) > self.max_angle:
            print(f"ANGLE LIMITE ! Roll:{roll:.1f} Pitch:{pitch:.1f} → EMERGENCY")
            self.emergency()
            return False
        return True
    
    def mix_motors(self, throttle, corr_roll, corr_pitch):
        """
        Mixage config X standard
        
        Sens de rotation (vu du dessus) :
          M4(CCW) --- M1(CW)
             |   X   |
          M3(CW)  --- M2(CCW)
        
        - Roll+  = drone penche à droite → augmente M3/M4 (gauche), diminue M1/M2 (droite)
        - Pitch+ = drone penche en avant → augmente M1/M2 (avant), diminue M3/M4 (arrière)
        """
        # si throttle nul → tous les moteurs à 0, même si le PID veut corriger
        # (on ne laisse pas les corrections PID faire tourner les moteurs sans intention de vol)
        if throttle <= 0:
            return 0, 0, 0, 0

        m1 = throttle + corr_roll - corr_pitch  # front right (CW)
        m2 = throttle + corr_roll + corr_pitch  # rear  right (CCW)
        m3 = throttle - corr_roll + corr_pitch  # rear  left  (CW)
        m4 = throttle - corr_roll - corr_pitch  # front left  (CCW)
        
        # clamp 0-100
        m1 = max(0, min(100, m1))
        m2 = max(0, min(100, m2))
        m3 = max(0, min(100, m3))
        m4 = max(0, min(100, m4))
        
        return m1, m2, m3, m4
    
    def update(self):
        """
        Boucle principale contrôle (1 itération)
        À appeler en boucle à ~100 Hz
        
        Ret : (roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch)
        """
        # ── lecture IMU ──
        roll, pitch, yaw = self.mpu.read_angles()
        
        # ── sécurités ──
        if not self.check_safety(roll, pitch):
            return roll, pitch, 0, 0, 0, 0, 0, 0
        
        if not self.armed or self.emergency_stop:
            return roll, pitch, 0, 0, 0, 0, 0, 0
        
        # ── corrections PID Roll/Pitch ──
        corr_roll  = self.pid_roll.compute(self.target_roll, roll)
        corr_pitch = self.pid_pitch.compute(self.target_pitch, pitch)
        
        # ── throttle effectif (avec ou sans altitude) ──
        if self.use_altitude and self.altitude_sensor is not None:
            altitude = self.altitude_sensor.distance_avg(samples=2)
            if altitude is not None and altitude < 2.5:
                corr_alt = self.pid_altitude.compute(self.target_altitude, altitude)
                throttle = self.base_throttle + corr_alt
            else:
                throttle = self.base_throttle
        else:
            throttle = self.base_throttle
        
        # clamp throttle final
        throttle = max(0, min(self.max_throttle, throttle))
        
        # ── mixage moteurs ──  (BUG CORRIGÉ : on passe throttle et pas self.base_throttle)
        m1, m2, m3, m4 = self.mix_motors(throttle, corr_roll, corr_pitch)
        
        # ── applique moteurs ──
        self.m1.throttle(m1)
        self.m2.throttle(m2)
        self.m3.throttle(m3)
        self.m4.throttle(m4)
        
        # ── stats loop rate ──
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