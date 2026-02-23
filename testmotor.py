"""from esc_motor import ESC
import time

# init moteur GPIO 2
motor1 = ESC(pin=2)
motor2 = ESC(pin=6)
motor3 = ESC(pin=4)
motor4 = ESC(pin=5)

# armement
print("Armement ESC...")
motor1.arm()
motor2.arm()
motor3.arm()
motor4.arm()

# test 100%
print("Test 100%")
motor1.throttle(40)
#motor2.throttle(45)
#motor3.throttle(45)
#motor4.throttle(40)
time.sleep(4)

# arrêt
motor1.stop()
motor2.stop()
motor3.stop()
motor4.stop()"""

"""from machine import Pin, time_pulse_us
import time
from esc_motor import ESC

# Entrée throttle radio
rx = Pin(10, Pin.IN)

# 4 ESC — RP2040: pin 3 partage le slice avec pin 2 → moteur 2 sur pin 6
motors = [
    ESC(pin=2),
    ESC(pin=6),
    ESC(pin=4),
    ESC(pin=5)
]

MAX_THROTTLE = 40

# Armement initial ESC
for m in motors:
    m.arm()

while True:
    try:
        # lecture throttle radio
        pulse = time_pulse_us(rx, 1, 25000)

        # conversion 1000–2000 µs → %
        percent = (pulse - 1000) / 10
        percent = max(0, min(MAX_THROTTLE, percent))

        # appliquer aux 4 moteurs
        for m in motors:
            m.throttle(percent)

    except:
        for m in motors:
            m.stop()

    time.sleep_ms(20)
"""
from machine import Pin, PWM
import time

# ===== CONFIGURATION =====
ESC_PINS = [2, 6, 4, 8]
PWM_FREQ = 50          # 50 Hz (standard ESC)
MIN_US = 1000          # Minimum throttle (1 ms)
MAX_US = 2000          # Maximum throttle (2 ms)
STEP = 25              # Incrément progression
DELAY = 0.1            # Temps entre chaque step

# ==========================

# Création des 4 PWM
escs = []
for pin in ESC_PINS:
    pwm = PWM(Pin(pin))
    pwm.freq(PWM_FREQ)
    escs.append(pwm)

# Conversion microsecondes -> duty 16 bits
def us_to_duty(us):
    period_us = 1000000 // PWM_FREQ
    duty = int((us / period_us) * 65535)
    return duty

# Fonction pour envoyer la même valeur aux 4 moteurs
def set_all_motors(us):
    duty = us_to_duty(us)
    for esc in escs:
        esc.duty_u16(duty)

# ==========================
# ARMEMENT
# ==========================
print("Armement des ESC...")
set_all_motors(MIN_US)
time.sleep(3)

# ==========================
# MONTEE PROGRESSIVE
# ==========================
print("Acceleration...")
for pulse in range(MIN_US, MAX_US, STEP):
    set_all_motors(pulse)
    time.sleep(DELAY)

# ==========================
# DESCENTE PROGRESSIVE
# ==========================
print("Deceleration...")
for pulse in range(MAX_US, MIN_US, -STEP):
    set_all_motors(pulse)
    time.sleep(DELAY)

# ==========================
# STOP
# ==========================
print("Arret moteurs")
set_all_motors(MIN_US)
