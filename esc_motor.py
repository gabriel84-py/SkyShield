from machine import Pin, PWM
import time

class ESC:
    """Classe pr contrôler ESC/moteur brushless via PWM (ALLER SIDO TU VAS Y ARRIVER)"""
    
    def __init__(self, pin, freq=50, min_us=1000, max_us=2000):
        """
        Init ESC
        
        Params :
        - pin: num GPIO (ex: 0, 1, 2, 3)
        - freq: fréq PWM en Hz (def 50 = standard servo)
        - min_us: impulsion min en µs (def 1000 = arrêt)
        - max_us: impulsion max en µs (def 2000 = plein gaz)
        """
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(freq)
        
        # save limites
        self.min_us = min_us
        self.max_us = max_us
        
        # calc duty cycle pr 16 bits (0-65535)
        # duty = (temps_us / période_us) × 65535
        self.period_us = 1_000_000 / freq  # période en µs
        
        # init à 0 (sécur)
        self.throttle(0)
    
    def _us_to_duty(self, us):
        """Convert µs → duty cycle 16 bits"""
        return int((us / self.period_us) * 65535)
    
    def throttle(self, percent):
        """
        Définit vitesse moteur
        
        Params : percent = 0 à 100 (0=arrêt, 100=max)
        """
        # limite 0-100
        percent = max(0, min(100, percent))
        
        # calc impulsion en µs
        pulse_us = self.min_us + (percent / 100.0) * (self.max_us - self.min_us)
        
        # convert → duty cycle
        duty = self._us_to_duty(pulse_us)
        
        # applique PWM
        self.pwm.duty_u16(duty)
    
    def arm(self):
        """Arme ESC"""
        print("Armement ESC...")
        self.throttle(0)
        time.sleep(2)
        print("ESC armé")
    
    def stop(self):
        """Arrêt moteur"""
        self.throttle(0)