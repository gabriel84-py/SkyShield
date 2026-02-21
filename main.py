from flight_controller import FlightController
from machine import Pin, time_pulse_us
import time

# ============================================================
#  CONFIGURATION
# ============================================================
RX_PIN          = 10       # pin signal radio throttle
MAX_THROTTLE    = 30       # limite sécurité banc test (%)
MIN_ARM_PULSE   = 900      # µs min signal radio valide
MAX_ARM_PULSE   = 2100     # µs max signal radio valide
LOG_FILE        = "imu_data.csv"
PRINT_EVERY     = 10       # affiche debug toutes les N boucles
# ============================================================

# init pin radio
rx = Pin(RX_PIN, Pin.IN)

# init flight controller (ESC + IMU + PID + VL53L0X)
print("=== SkyShield Flight Controller ===")
fc = FlightController()

# armement ESC
fc.arm()

# init logging CSV
csv_file = open(LOG_FILE, "w")
csv_file.write("time,roll,pitch,throttle,m1,m2,m3,m4,corr_roll,corr_pitch,p_roll,i_roll,d_roll,p_pitch,i_pitch,d_pitch\n")

start_time   = time.ticks_ms()
last_throttle = 0
loop_count   = 0

print("Boucle principale lancée — bonne chance pour le tuning !")
print(f"PID Roll  → Kp={fc.pid_roll.kp}  Ki={fc.pid_roll.ki}  Kd={fc.pid_roll.kd}")
print(f"PID Pitch → Kp={fc.pid_pitch.kp}  Ki={fc.pid_pitch.ki}  Kd={fc.pid_pitch.kd}")

try:
    while True:
        # ── 1. LECTURE THROTTLE RADIO ──────────────────────────
        try:
            pulse = time_pulse_us(rx, 1, 30000)
            if MIN_ARM_PULSE <= pulse <= MAX_ARM_PULSE:
                percent = (pulse - 1000) / 10.0
                percent = max(0.0, min(MAX_THROTTLE, percent))
                last_throttle = percent
            else:
                percent = last_throttle   # valeur aberrante → garde la dernière
        except:
            percent = last_throttle       # timeout → garde la dernière

        # ── 2. TRANSMET THROTTLE AU FLIGHT CONTROLLER ─────────
        fc.set_throttle(percent)

        # ── 3. BOUCLE DE STABILISATION PID ────────────────────
        roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch = fc.update()

        # ── 4. LOGGING CSV ────────────────────────────────────
        t = time.ticks_diff(time.ticks_ms(), start_time) / 1000.0
        p_roll,  i_roll,  d_roll  = fc.get_pid_terms('roll')
        p_pitch, i_pitch, d_pitch = fc.get_pid_terms('pitch')

        csv_file.write(
            f"{t:.3f},{roll:.4f},{pitch:.4f},{percent:.1f},"
            f"{m1:.1f},{m2:.1f},{m3:.1f},{m4:.1f},"
            f"{corr_roll:.3f},{corr_pitch:.3f},"
            f"{p_roll:.3f},{i_roll:.3f},{d_roll:.3f},"
            f"{p_pitch:.3f},{i_pitch:.3f},{d_pitch:.3f}\n"
        )
        csv_file.flush()

        # ── 5. DEBUG CONSOLE ──────────────────────────────────
        loop_count += 1
        if loop_count % PRINT_EVERY == 0:
            print(
                f"t:{t:6.2f}s | "
                f"Roll:{roll:6.2f}° Pitch:{pitch:6.2f}° | "
                f"Thr:{percent:4.0f}% | "
                f"M1:{m1:4.0f} M2:{m2:4.0f} M3:{m3:4.0f} M4:{m4:4.0f} | "
                f"cR:{corr_roll:+.2f} cP:{corr_pitch:+.2f} | "
                f"Hz:{fc.loop_rate:.0f}"
            )

        # ── 6. TIMING 100 Hz ──────────────────────────────────
        time.sleep_ms(10)

except KeyboardInterrupt:
    fc.disarm()
    csv_file.close()
    print("\nMoteurs arrêtés — données sauvées dans", LOG_FILE)

except Exception as e:
    fc.emergency()
    csv_file.close()
    print(f"\nErreur critique : {e}")
    print("EMERGENCY STOP — fichier fermé")