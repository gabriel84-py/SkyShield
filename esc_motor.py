from machine import Pin, PWM
import time

class ESC:
    """Classe pr contrôler ESC/moteur brushless via PWM"""
    
    def __init__(self, pin, freq=50, min_us=1000, max_us=2000):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(freq)
        
        self.min_us = min_us
        self.max_us = max_us
        self.period_us = 1_000_000 / freq

        # ── FIX : signal coupé dès l'init ────────────────────────
        # duty_u16(0) = aucune impulsion = moteur garanti à l'arrêt
        # L'ancien code appelait throttle(0) qui envoyait 1000µs,
        # ce qui peut faire tourner certains ESC légèrement
        self.pwm.duty_u16(0)
    
    def _us_to_duty(self, us):
        """Convert µs → duty cycle 16 bits"""
        return int((us / self.period_us) * 65535)
    
    def throttle(self, percent):
        """
        Définit vitesse moteur
        percent = 0 à 100 (0 = arrêt total, 100 = plein gaz)
        """
        percent = max(0, min(100, percent))

        # ── FIX : si 0% → coupe le signal PWM complètement ──────
        # duty_u16(0) = silence total sur le pin
        # ≠ impulsion 1000µs qui peut faire tourner certains ESC
        if percent == 0:
            self.pwm.duty_u16(0)
            return

        pulse_us = self.min_us + (percent / 100.0) * (self.max_us - self.min_us)
        self.pwm.duty_u16(self._us_to_duty(pulse_us))
    
    def arm(self):
        """
        Arme l'ESC :
        1. Envoie 1000µs pendant 2s → l'ESC apprend son point bas
        2. Coupe le signal → moteur reste à l'arrêt
        """
        print("Armement ESC...")
        # envoie impulsion 1000µs (nécessaire pour calibration ESC)
        self.pwm.duty_u16(self._us_to_duty(self.min_us))
        time.sleep(2)
        # coupe → moteur silencieux
        self.pwm.duty_u16(0)
        print("ESC armé")
    
    def stop(self):
        """Arrêt immédiat — coupe le signal PWM"""
        self.pwm.duty_u16(0)