"""from esc_motor import ESC
import time

# init moteur GPIO 2
motor1 = ESC(pin=2)
#motor2 = ESC(pin=3)
#motor3 = ESC(pin=4)
#motor4 = ESC(pin=5)

# armement
print("Armement ESC...")
motor1.arm()
#motor2.arm()
#motor3.arm()
#motor4.arm()

# test 100%
print("Test 100%")
motor1.throttle(50)
#motor2.throttle(10)
#motor3.throttle(10)
#motor4.throttle(10)
time.sleep(4)

# arrêt
motor1.stop()
#motor2.stop()
#motor3.stop()
#motor4.stop()"""

from machine import Pin, time_pulse_us
import time
from esc_motor import ESC

# Entrée throttle radio
rx = Pin(10, Pin.IN)

# 4 ESC
motors = [
    ESC(pin=2),
    ESC(pin=3),
    ESC(pin=4),
    ESC(pin=5)
]

MAX_THROTTLE = 50

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
