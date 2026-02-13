from mpu6050 import MPU6050
from machine import Pin, time_pulse_us
from esc_motor import ESC
import time

# init capt MPU6050 (pins 27 et 26)
mpu = MPU6050(scl=27, sda=26, addr=0x68)

# entree throttle radio
rx = Pin(10, Pin.IN)

# 4 ESC
motors = [
    ESC(pin=2),
    ESC(pin=3),
    ESC(pin=4),
    ESC(pin=5)
]

MAX_THROTTLE = 50
last_valid_throttle = 0

# armement initial ESC
for m in motors:
    m.arm()

# ouvre fichier csv en ecriture
csv_file = open("imu_data.csv", "w")
csv_file.write("time,roll,pitch,throttle\n")

start_time = time.ticks_ms()

print("zéééépartitr")

try:
    while True:
        # lecture throttle radio
        try:
            pulse = time_pulse_us(rx, 1, 30000)  # timeout augmente a 30ms
            
            # validation plage normale radio (900-2100us)
            if 900 <= pulse <= 2100:
                # conversion 1000-2000 us -> %
                percent = (pulse - 1000) / 10
                percent = max(0, min(MAX_THROTTLE, percent))
                last_valid_throttle = percent
            else:
                # valeur aberrante -> garde derniere valeur valide
                percent = last_valid_throttle
                
        except:
            # timeout -> garde derniere valeur valide (pas arret brutal)
            percent = last_valid_throttle
        
        # applique aux 4 moteurs
        for m in motors:
            m.throttle(percent)
        
        # lit angles roll, pitch, yaw
        roll, pitch, yaw = mpu.read_angles()
        
        # temps ecoule en secondes
        current_time = time.ticks_diff(time.ticks_ms(), start_time) / 1000.0
        
        # ecrit dans fichier
        csv_file.write(f"{current_time},{roll},{pitch},{percent}\n")
        csv_file.flush()  # force ecriture immediate
        
        # affiche donnees
        print(f"t:{current_time:6.2f}s | Roll:{roll:6.2f} | Pitch:{pitch:6.2f} | Throttle:{percent:4.0f}%")
        
        time.sleep_ms(10)  # 100Hz au lieu de 20ms pour 50Hz

except KeyboardInterrupt:
    # arret moteurs
    for m in motors:
        m.stop()
    csv_file.close()
    print("\nMoteurs arretes")
    print("Donnees sauvegardees dans imu_data.csv")
    print("Programme termine")
except:
    # arret moteurs en cas erreur
    for m in motors:
        m.stop()
    csv_file.close()
    print("\nErreur - moteurs arretes - fichier ferme")