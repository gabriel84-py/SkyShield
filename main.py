from mpu6050 import MPU6050
import time

# Initialiser le capteur MPU6050 (pins 21 et 20)
mpu = MPU6050(scl=27, sda=26, addr=0x68)

print("=== Lecture MPU6050 ===")
print("Roll = rotation gauche/droite")
print("Pitch = rotation avant/arrière")
print("Yaw = rotation autour de l'axe vertical")
print("-" * 50)


# Boucle infinie pour lire et afficher
while True:
    # Lire angles roll, pitch et yaw
    roll, pitch, yaw = mpu.read_angles()
    
    # Afficher angles formatés 2 décimales 6 caractères largeur f pour float (ps sido tu connaissait ou pas ?)
    print(f"Roll: {roll:6.2f}°  |  Pitch: {pitch:6.2f}°  |  Yaw: {yaw:6.2f}°")

    time.sleep(0.1)

"rappels : "
"""
| Paramètre    | Valeur     |
| ------------ | ---------- |
| Orientation  | à plat     |
| Infill       | Gyroid     |
| Densité      | 25–35 %    |
| Parois       | 4–5        |
| Layer height | 0.2 mm     |
| Temp buse    | 235–245 °C |
| Temp plateau | 75–85 °C   |
"""