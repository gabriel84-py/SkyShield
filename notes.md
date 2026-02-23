| Filtre        | Fréquence  |
| ------------- | -----------|
| Low-pass gyro | 80–100 Hz  |
| Notch frame   | 100–130 Hz |
| Notch moteur  | 600–800 Hz |


FRAME : True X 250mm
POIDS : 444g (léger !)
BATTERIE : 3S 2300mAh 45C @ 11.45V (50% charge)
RATIO POUSSÉE/POIDS : 1600g / 444g = 3.6:1 

MOTEURS :
- M1 (front right CW) : GP2
- M2 (rear right CCW) : GP3  
- M3 (rear left CW) : GP4
- M4 (front left CCW) : GP5
Throttle min stable : 15%

IMU : MPU6050
- I2C : GP27 (SCL), GP26 (SDA)
- Position : Centre drone
- Isolation : Standoffs silicone 20mm 
- Kalman actif : accel=0.1, gyro=0.01
- Pas INT (pr l'instant)

CONTRÔLE : PC → USB → Pico (ou radio)
OBJECTIF : Tuning PID Roll/Pitch sur banc test