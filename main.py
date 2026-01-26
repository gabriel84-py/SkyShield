from mpu6050 import MPU6050
import time


# Initialiser le capteur MPU6050 (pins 21 et 20)
mpu = MPU6050(scl=21, sda=20)

# Message de démarrage
print("=== Lecture MPU6050 ===")
print("Roll = rotation gauche/droite je sais que tu sais sido tkt :p")
print("Pitch = rotation avant/arrière")
print("-" * 40)

# Boucle infinie pour lire et afficher
while True:
    # Lire les angles roll et pitch
    roll, pitch = mpu.read_angles()
    
    # Afficher dans la console
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