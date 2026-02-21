from flight_controller import FlightController
from machine import Pin, time_pulse_us
import time

# ============================================================
#  CONFIGURATION
# ============================================================
RX_PIN        = 10    # pin signal radio throttle
MAX_THROTTLE  = 30    # limite sécurité banc test (%)
MIN_ARM_PULSE = 900   # µs min signal radio valide
MAX_ARM_PULSE = 2100  # µs max signal radio valide
LOG_FILE      = "imu_data.csv"
PRINT_EVERY   = 20    # affiche debug toutes les N boucles
LOOP_MS       = 10    # cible 100 Hz = 10ms par itération
# ============================================================

rx = Pin(RX_PIN, Pin.IN)

print("=== SkyShield Flight Controller ===")
fc = FlightController()

# arm() inclut l'auto-calibration IMU → ne pas bouger le drone pendant 1.5s !
fc.arm()

csv_file = open(LOG_FILE, "w")
csv_file.write("time,roll,pitch,throttle,m1,m2,m3,m4,corr_roll,corr_pitch,p_roll,i_roll,d_roll,p_pitch,i_pitch,d_pitch\n")

start_time    = time.ticks_ms()
last_throttle = 0.0
loop_count    = 0

print(f"Boucle lancée — cible {1000//LOOP_MS} Hz")
print(f"Deadzone radio  : {fc.RADIO_DEADZONE}%  (tout < = traité comme 0)")
print(f"Seuil moteur    : {fc.MIN_MOTOR_THROTTLE}%  (entre 0 et ce seuil = coupé)")
print(f"PID cibles      : roll={fc.target_roll:.2f}°  pitch={fc.target_pitch:.2f}°")
print(f"PID Roll  → Kp={fc.pid_roll.kp}  Ki={fc.pid_roll.ki}  Kd={fc.pid_roll.kd}")
print(f"PID Pitch → Kp={fc.pid_pitch.kp}  Ki={fc.pid_pitch.ki}  Kd={fc.pid_pitch.kd}")

try:
    while True:
        loop_start = time.ticks_ms()

        # ── 1. LECTURE THROTTLE RADIO ──────────────────────────
        # timeout 20ms (pas 30ms) pour ne pas bloquer la boucle
        try:
            pulse = time_pulse_us(rx, 1, 20000)
            if MIN_ARM_PULSE <= pulse <= MAX_ARM_PULSE:
                raw = (pulse - 1000) / 10.0
                raw = max(0.0, min(MAX_THROTTLE, raw))
                last_throttle = raw
                percent = raw
            else:
                percent = last_throttle  # hors plage → garde la dernière
        except:
            percent = last_throttle      # timeout → garde la dernière

        # ── 2. THROTTLE → FLIGHT CONTROLLER ───────────────────
        # set_throttle applique la deadzone radio en interne
        fc.set_throttle(percent)

        # ── 3. PID + MOTEURS ───────────────────────────────────
        roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch = fc.update()

        # ── 4. LOGGING CSV ────────────────────────────────────
        t = time.ticks_diff(time.ticks_ms(), start_time) / 1000.0
        p_roll,  i_roll,  d_roll  = fc.get_pid_terms('roll')
        p_pitch, i_pitch, d_pitch = fc.get_pid_terms('pitch')

        # throttle loggé = celui effectivement utilisé (après deadzone)
        thr_effective = fc.base_throttle

        csv_file.write(
            f"{t:.3f},{roll:.4f},{pitch:.4f},{thr_effective:.1f},"
            f"{m1:.1f},{m2:.1f},{m3:.1f},{m4:.1f},"
            f"{corr_roll:.3f},{corr_pitch:.3f},"
            f"{p_roll:.3f},{i_roll:.3f},{d_roll:.3f},"
            f"{p_pitch:.3f},{i_pitch:.3f},{d_pitch:.3f}\n"
        )
        csv_file.flush()

        # ── 5. DEBUG CONSOLE ──────────────────────────────────
        loop_count += 1
        if loop_count % PRINT_EVERY == 0:
            thr_label = f"{thr_effective:4.1f}%" if thr_effective > 0 else "DEAD"
            print(
                f"t:{t:6.2f}s | "
                f"Roll:{roll:6.2f}° Pitch:{pitch:6.2f}° | "
                f"Thr:{thr_label} | "
                f"M1:{m1:4.1f} M2:{m2:4.1f} M3:{m3:4.1f} M4:{m4:4.1f} | "
                f"cR:{corr_roll:+.2f} cP:{corr_pitch:+.2f} | "
                f"Hz:{fc.loop_rate:.0f}"
            )

        # ── 6. TIMING PRÉCIS ──────────────────────────────────
        elapsed   = time.ticks_diff(time.ticks_ms(), loop_start)
        remaining = LOOP_MS - elapsed
        if remaining > 1:
            time.sleep_ms(remaining)

except KeyboardInterrupt:
    fc.disarm()
    csv_file.close()
    print("\nMoteurs arrêtés — données sauvées dans", LOG_FILE)

except Exception as e:
    fc.emergency()
    csv_file.close()
    print(f"\nErreur critique : {e}")
    print("EMERGENCY STOP — fichier fermé")