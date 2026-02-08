import matplotlib.pyplot as plt
import csv

# lecture donnees
time_data = []
roll_data = []
pitch_data = []
throttle_data = []

with open("imu_data.csv", "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        time_data.append(float(row['time']))
        roll_data.append(float(row['roll']))
        pitch_data.append(float(row['pitch']))
        throttle_data.append(float(row['throttle']))

# creation graphique avec 2 subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# subplot 1: roll et pitch
ax1.plot(time_data, roll_data, 'b-', label='Roll', linewidth=1.5)
ax1.plot(time_data, pitch_data, 'r-', label='Pitch', linewidth=1.5)
ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)
ax1.set_ylabel('Angle (degres)')
ax1.set_title('Evolution Roll et Pitch MPU6050')
ax1.legend()
ax1.grid(True, alpha=0.3)

# subplot 2: throttle
ax2.plot(time_data, throttle_data, 'g-', label='Throttle', linewidth=1.5)
ax2.set_xlabel('Temps (s)')
ax2.set_ylabel('Throttle (%)')
ax2.set_title('Commande Throttle Radio')
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()

# sauvegarde et affichage
plt.savefig('imu_plot_sans_motor_noise+_4.png', dpi=150, bbox_inches='tight')
print("Graphique sauvegarde dans imu_plot.png")
plt.show()