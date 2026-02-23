from machine import Pin, PWM
import time

# RP2040 (Pico): chaque slice PWM a 2 canaux (A/B). GPIO 2+3 = slice 1, GPIO 4+5 = slice 2.
# Éviter 2 moteurs sur le même slice (comportement erratique). Moteur 2 → GPIO 6 (slice 3).

class ESC:
    """Classe pr contrôler ESC/moteur brushless via PWM"""
    
    def __init__(self, pin, freq=50, min_us=1000, max_us=2000):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(freq)
        
        self.min_us = min_us
        self.max_us = max_us
        self.period_us = 1_000_000 / freq

        self.pwm.duty_u16(self._us_to_duty(self.min_us))

    
    def _us_to_duty(self, us):
        """Convert µs → duty cycle 16 bits"""
        return int((us / self.period_us) * 65535)
    
    def throttle(self, percent):
        percent = max(0, min(100, percent))

        # Toujours envoyer au minimum 1000µs
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
        self.pwm.duty_u16(self._us_to_duty(self.min_us))
