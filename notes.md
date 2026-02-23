| Filtre        | Fréquence  |
| ------------- | -----------|
| Low-pass gyro | 80–100 Hz  |
| Notch frame   | 100–130 Hz |
| Notch moteur  | 600–800 Hz |


FRAME : True X 250mm
POIDS : 444g (léger !)
BATTERIE : 3S 2300mAh 45C @ 11.45V (50% charge)
RATIO POUSSÉE/POIDS : 1600g / 444g = 3.6:1 

MOTEURS — disposition physique réelle (vue de dessus) :

  M1(CW) front-right  ──  M2(CCW) front-left
          |       X       |
  M4(CCW) rear-right  ──  M3(CW)  rear-left

- M1 (front-right CW)  : GP2
- M2 (front-left  CCW) : GP6
- M3 (rear-left   CW)  : GP4
- M4 (rear-right  CCW) : GP8

Throttle min stable : 15%

MIXAGE PID (corr_roll > 0 = correction droite, corr_pitch > 0 = correction avant) :
  M1 = throttle - corr_roll - corr_pitch
  M2 = throttle + corr_roll - corr_pitch
  M3 = throttle + corr_roll + corr_pitch
  M4 = throttle - corr_roll + corr_pitch

IMU : MPU6050
- I2C : GP27 (SCL), GP26 (SDA)
- Position : Centre drone
- Isolation : Standoffs silicone 20mm 
- Kalman actif : accel=0.1, gyro=0.01
- Pas INT (pr l'instant)

CONTRÔLE : PC → USB → Pico (ou radio)
OBJECTIF : Tuning PID Roll/Pitch sur banc de test