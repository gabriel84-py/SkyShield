from mpu6050 import MPU6050
from sd_logger import SDLogger
import time

# init capt MPU6050 (pins 27 et 26)
mpu = MPU6050(scl=27, sda=26, addr=0x68)

# init logger SD (pins SPI0 par défaut)
logger = SDLogger(cs_pin=17, sck_pin=18, mosi_pin=19, miso_pin=16)

# crée nouveau fichier pr ce vol
logger.new_flight()

# compteur pr afficher stats
log_count = 0

try:
    while True:
        # lit angles roll, pitch, yaw
        roll, pitch, yaw = mpu.read_angles()
        
        # log sur SD
        if logger.log(roll, pitch, yaw):
            log_count += 1
        
        # affiche angles formatés (2 déc, 6 car largeur) + nb logs
        print(f"Roll: {roll:6.2f}°  |  Pitch: {pitch:6.2f}°  |  Yaw: {yaw:6.2f}°  |  Logs: {log_count}")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    # arrêt propre (Ctrl+C)
    print("\n=== Arrêt logging ===")
    print(f"Total logs enregistrés: {log_count}")
    logger.unmount()
    print("Programme terminé")