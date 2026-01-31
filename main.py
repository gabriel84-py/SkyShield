from mpu6050 import MPU6050
import time

# Initialiser le capteur MPU6050 (pins 21 et 20)
mpu = MPU6050(scl=27, sda=26, addr=0x68)

# Message de démarrage
print("=== Lecture MPU6050 ===")
print("Roll = rotation gauche/droite")
print("Pitch = rotation avant/arrière")
print("Yaw = rotation autour de l'axe vertical")
print("-" * 50)

# Boucle infinie pour lire et afficher
while True:
    # Lire les angles roll, pitch et yaw
    roll, pitch, yaw = mpu.read_angles()
    
    # Afficher dans la console
    print(f"Roll: {roll:6.2f}°  |  Pitch: {pitch:6.2f}°  |  Yaw: {yaw:6.2f}°")
    
    # Pause de 100ms entre chaque lecture
    time.sleep(0.1)