"""https://github.com/micropython/micropython-lib : code lib for the sensors"""

from flight_controller import FlightController
from machine import Pin, time_pulse_us
import time

print("="*70)
print("   DRONE FLIGHT CONTROLLER")
print("="*70)

# init flight controller
print("\nInitialisation...")
fc = FlightController()

# config radio rx
rx_pin = Pin(10, Pin.IN)
MAX_THROTTLE = 50

# variables etat
armed = False
last_rx_time = time.ticks_ms()
RX_TIMEOUT = 500

# stats affichage
loop_count = 0
display_interval = 10

input("\nBranche batterie et appuie ENTER...")

# armement esc
fc.arm()
armed = True

print("\n" + "="*70)
print("   VOL EN COURS")
print("="*70)
print("Throttle | Roll/Pitch/Alt | Corrections PID | Moteurs | Loop")
print("="*70)

# boucle principale
try:
    while True:
        loop_start = time.ticks_us()
        
        # lecture radio
        try:
            pulse = time_pulse_us(rx_pin, 1, 25000)
            last_rx_time = time.ticks_ms()
            
            # conversion 1000-2000us -> 0-100%
            throttle_rx = (pulse - 1000) / 10.0
            throttle_rx = max(0, min(MAX_THROTTLE, throttle_rx))
            
            radio_ok = True
        except:
            radio_ok = False
            throttle_rx = 0
        
        # failsafe radio
        if time.ticks_diff(time.ticks_ms(), last_rx_time) > RX_TIMEOUT:
            if armed:
                print("\nPERTE SIGNAL RADIO - ARRET URGENCE")
                fc.emergency()
                armed = False
            throttle_rx = 0
            radio_ok = False
        
        # throttle base
        fc.set_throttle(throttle_rx)
        
        # update controleur (PID actif)
        roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch = fc.update()
        
        # lecture altitude
        try:
            altitude = fc.altitude_sensor.distance_avg(samples=1)
            if altitude is not None and altitude < 2.5:
                corr_alt = fc.pid_altitude.compute(fc.target_altitude, altitude)
            else:
                corr_alt = 0.0
        except:
            altitude = None
            corr_alt = 0.0
        
        # affichage periodique
        loop_count += 1
        if loop_count % display_interval == 0:
            alt_str = f"{altitude:.2f}m" if altitude else "-.--m"
            radio_str = "OK" if radio_ok else "XX"
            
            print(f"{radio_str} | T:{throttle_rx:4.0f}% | R:{roll:+6.1f} P:{pitch:+6.1f} A:{alt_str} | cR:{corr_roll:+5.1f} cP:{corr_pitch:+5.1f} cA:{corr_alt:+5.1f} | M1:{m1:3.0f} M2:{m2:3.0f} M3:{m3:3.0f} M4:{m4:3.0f} | {fc.loop_rate:.0f}Hz")
        
        # timing loop (100 Hz)
        elapsed = time.ticks_diff(time.ticks_us(), loop_start)
        sleep_time = 10000 - elapsed
        if sleep_time > 0:
            time.sleep_us(sleep_time)

except KeyboardInterrupt:
    print("\n\n" + "="*70)
    print("   ARRET PROGRAMME")
    print("="*70)
    fc.disarm()
    armed = False
    print("Moteurs arretes")
    print("Debranche batterie")

except Exception as e:
    print(f"\n\nERREUR CRITIQUE: {e}")
    fc.emergency()
    armed = False
    print("DEBRANCHE BATTERIE IMMEDIATEMENT")


# notes utilisation
"""
AFFICHAGE :
- OK/XX : Signal radio
- T : Throttle radio (%)
- R/P/A : Roll/Pitch/Altitude
- cR/cP/cA : Corrections PID
- M1-M4 : Throttle moteurs (%)
- Hz : Frequence boucle

SECURITES :
- Timeout radio 500ms -> arret moteurs
- Angle max 45 -> emergency stop
- Throttle limite 50% max
- Anti-windup PID

PINS :
- GP2,3,4,5 : Moteurs ESC
- GP26,27 : MPU6050 I2C
- GP20,21 : VL53L0X I2C
- GP10 : Recepteur radio (PWM)
"""