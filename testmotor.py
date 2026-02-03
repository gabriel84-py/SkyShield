from esc_motor import ESC
import time

# init moteur GPIO 2
motor = ESC(pin=2)

# armement
print("Armement ESC...")
motor.arm()

# test 100%
print("Test 100%")
motor.throttle(100)
time.sleep(5)

# arrêt
motor.stop()