from esc_motor import ESC
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
motor1.throttle(30)
#motor2.throttle(10)
#motor3.throttle(10)
#motor4.throttle(10)
time.sleep(2)

# arrêt
motor1.stop()
#motor2.stop()
#motor3.stop()
#motor4.stop()