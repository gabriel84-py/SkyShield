from mpu6050 import MPU6050
from sd_logger import SDLogger
import time

# init capt MPU6050 (pins 27 et 26)
mpu = MPU6050(scl=27, sda=26, addr=0x68)

try:
    while True:
        # lit angles roll, pitch, yaw
        roll, pitch, yaw = mpu.read_angles()
        
        # affiche angles formatés (2 déc, 6 car largeur) + nb logs
        print(f"Roll: {roll:6.2f}°  |  Pitch: {pitch:6.2f}°  |  Yaw: {yaw:6.2f}°")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    # arrêt propre (Ctrl+C)
    print("Programme terminé")