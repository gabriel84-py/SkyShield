from hcsr04 import HCSR04
import time

# init capteur ultrason (trigger=GP14, echo=GP15)
sensor = HCSR04(trigger_pin=14, echo_pin=15)

print("=== Capteur Ultrason HC-SR04 ===")
print("Trigger: GPIO 14")
print("Echo: GPIO 15")
print("Portée: 2cm → 4m")
print("-" * 40)

# boucle infinie pr mesurer distance
try:
    while True:
        # mesure distance av moyenne (+ stable)
        distance = sensor.distance_avg(samples=3)

        if distance is not None:
            # affiche distance en m (3 déc)
            print(f"Distance: {distance:5.3f} m  |  {distance * 100:6.1f} cm")
        else:
            # hors portée ou obstacle absent
            print("Distance: --.--- m  |  Hors portée")

        # pause 200ms entre mesures
        time.sleep(0.2)

except KeyboardInterrupt:
    # arrêt propre (Ctrl+C)
    print("\n✓ Programme arrêté")