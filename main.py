from mpu6050 import MPU6050
from oled import OLED
import time

# Initialiser l'écran OLED (pins 9 et 8)
oled = OLED(scl=9, sda=8)

# Initialiser le capteur MPU6050 (pins 21 et 20)
mpu = MPU6050(scl=21, sda=20)

# Message de démarrage
print("=== Lecture MPU6050 ===")
print("Affichage sur OLED...")

# Boucle infinie pour lire et afficher
while True:
    # Lire les angles roll et pitch
    roll, pitch = mpu.read_angles()

    # Afficher sur l'écran OLED
    oled.show_angles(roll, pitch)

    # Afficher aussi dans le terminal
    print(f"Roll: {roll:6.2f}°  |  Pitch: {pitch:6.2f}°")

    # Pause de 100ms entre chaque lecture
    time.sleep(0.1)


"raiseppels : "
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