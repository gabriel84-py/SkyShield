from vl53l0x import VL53L0X
import time

print("="*50)
print("   TEST CAPTEUR VL53L0X LASER TOF")
print("="*50)

# init capteur 
# Ajuste selon ton câblage !
try:
    sensor = VL53L0X(
        i2c_id=0,
        scl=13,
        sda=12,
        addr=0x29
    )
except Exception as e:
    print(f"\nErreur init capteur: {e}")
    import sys
    sys.exit(1)

print("Portée : 3cm - 2m")
print("Fréquence : 50 Hz")
print("-"*50)

# boucle mesure temps réel
try:
    while True:
        # mesure simple
        dist_mm = sensor.distance_mm()
        
        if dist_mm is not None:
            dist_cm = dist_mm / 10.0
            dist_m = dist_mm / 1000.0
            
            print(f"Distance: {dist_m:5.3f} m  |  {dist_cm:6.1f} cm  |  {dist_mm:4.0f} mm", end='')
            
            print()  # newline
        else:
            print("Distance: --.--- m  |  Hors portée ou erreur")
        
        # pause 50ms (20 Hz max pr VL53L0X)
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\n finitoooooo !!")