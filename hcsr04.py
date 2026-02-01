from machine import Pin
import time


class HCSR04:
    """Classe pr mesurer distance av capteur ultrason HC-SR04"""

    def __init__(self, trigger_pin=14, echo_pin=15, timeout_us=30000):
        """
        Init capteur ultrason

        Params :
        - trigger_pin: pin pr déclencher mesure (def 14)
        - echo_pin: pin pr recevoir écho (def 15)
        - timeout_us: timeout max en µs (def 30ms = 5m max)
        """
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.timeout = timeout_us

        # init trigger à 0
        self.trigger.value(0)
        time.sleep(0.1)

    def _pulse_us(self):
        """
        Mesure durée impulsion echo en µs
        Ret : durée en µs, ou -1 si timeout
        """
        # déclenche mesure (impulsion 10µs sur trigger)
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)

        # attend début echo (passage à HIGH)
        start = time.ticks_us()
        while self.echo.value() == 0:
            if time.ticks_diff(time.ticks_us(), start) > self.timeout:
                return -1  # timeout

        # mémorise début impulsion
        pulse_start = time.ticks_us()

        # attend fin echo (passage à LOW)
        while self.echo.value() == 1:
            if time.ticks_diff(time.ticks_us(), pulse_start) > self.timeout:
                return -1  # timeout

        # mémorise fin impulsion
        pulse_end = time.ticks_us()

        # calc durée impulsion
        return time.ticks_diff(pulse_end, pulse_start)

    def distance_cm(self):
        """
        Mesure distance en cm
        Ret : distance en cm, ou None si erreur
        """
        # mesure durée impulsion
        pulse = self._pulse_us()

        if pulse < 0:
            return None  # timeout / erreur

        # calc distance (vitesse son = 340 m/s = 0.034 cm/µs)
        # distance = (durée × vitesse) / 2  (aller-retour)
        distance = (pulse * 0.034) / 2

        return distance

    def distance_m(self):
        """
        Mesure distance en mètres
        Ret : distance en m, ou None si erreur
        """
        cm = self.distance_cm()

        if cm is None:
            return None

        return cm / 100.0

    def distance_avg(self, samples=5):
        """
        Mesure distance av moyenne sur N échantillons (+ stable)

        Params : samples = nb mesures pr moyenne (def 5)
        Ret : distance en m, ou None si erreur
        """
        measures = []

        for _ in range(samples):
            dist = self.distance_m()
            if dist is not None:
                measures.append(dist)
            time.sleep(0.01)  # 10ms entre mesures

        if len(measures) == 0:
            return None

        # ret moyenne
        return sum(measures) / len(measures)