from flight_controller import FlightController
from machine import Pin, time_pulse_us
import time

# ============================================================
#  CONFIGURATION
# ============================================================""
RX_PIN        = 10    # pin signal radio throttle
MAX_THROTTLE  = 30    # limite sécurité banc test (%)
MIN_ARM_PULSE = 900   # µs min signal radio valide
MAX_ARM_PULSE = 2100  # µs max signal radio valide
LOG_FILE      = "imu_data.csv"
PRINT_EVERY   = 20    # affiche debug toutes les N boucles
LOOP_MS       = 10    # cible 100 Hz
# ============================================================

rx = Pin(RX_PIN, Pin.IN)

print("=== SkyShield Flight Controller ===")
fc = FlightController()

# arm() = armement ESC + auto-calibration IMU
# NE PAS BOUGER LE DRONE pendant l'armement (~10s total)
fc.arm()

csv_file = open(LOG_FILE, "w")
csv_file.write("time,roll,pitch,throttle,m1,m2,m3,m4,corr_roll,corr_pitch,p_roll,i_roll,d_roll,p_pitch,i_pitch,d_pitch\n")

start_time    = time.ticks_ms()
last_throttle = 0.0
loop_count    = 0
fc.max_throttle = MAX_THROTTLE 

print(f"Boucle lancée — cible {1000 // LOOP_MS} Hz")
print(f"Deadzone radio  : active sous {fc.DEAD_OFF:.0f}%, active au-dessus de {fc.DEAD_ON:.0f}%")
print(f"Seuil moteur    : {fc.MIN_MOTOR:.0f}% (snap-to-min actif)")
print(f"PID cibles      : roll={fc.target_roll:.2f}°  pitch={fc.target_pitch:.2f}°")
print(f"PID Roll  Kp={fc.pid_roll.kp}  Ki={fc.pid_roll.ki}  Kd={fc.pid_roll.kd}")
print(f"PID Pitch Kp={fc.pid_pitch.kp}  Ki={fc.pid_pitch.ki}  Kd={fc.pid_pitch.kd}")
print("─" * 70)

try:
    while True:
        loop_start = time.ticks_ms()

        # ── 1. LECTURE RADIO ───────────────────────────────────
        # timeout 20ms max (pas 30ms) pour ne pas bloquer la boucle
        try:
            pulse = time_pulse_us(rx, 1, 20000)
            if MIN_ARM_PULSE <= pulse <= MAX_ARM_PULSE:
                raw = (pulse - 1000) / 10.0
                raw = max(0.0, min(MAX_THROTTLE, raw))
                last_throttle = raw
            # si hors plage → garde last_throttle (pas de mise à jour)
        except:
            pass   # timeout → garde last_throttle

        percent = last_throttle

        # ── 2. THROTTLE → FC (deadzone hystérésis appliquée ici)
        fc.set_throttle(percent)

        # ── 3. PID + MOTEURS ───────────────────────────────────
        roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch = fc.update()

        # ── 4. LOGGING CSV ────────────────────────────────────
        t = time.ticks_diff(time.ticks_ms(), start_time) / 1000.0
        p_roll,  i_roll,  d_roll  = fc.get_pid_terms('roll')
        p_pitch, i_pitch, d_pitch = fc.get_pid_terms('pitch')
        thr_eff = fc.base_throttle

        csv_file.write(
            f"{t:.3f},{roll:.4f},{pitch:.4f},{thr_eff:.1f},"
            f"{m1:.1f},{m2:.1f},{m3:.1f},{m4:.1f},"
            f"{corr_roll:.3f},{corr_pitch:.3f},"
            f"{p_roll:.3f},{i_roll:.3f},{d_roll:.3f},"
            f"{p_pitch:.3f},{i_pitch:.3f},{d_pitch:.3f}\n"
        )
        csv_file.flush()

        # ── 5. DEBUG CONSOLE ──────────────────────────────────
        loop_count += 1
        if loop_count % PRINT_EVERY == 0:
            # état deadzone
            if thr_eff == 0:
                state = f"DEAD(radio={percent:.1f}%)"
            else:
                state = f"{thr_eff:.1f}%"
            print(
                f"t:{t:6.2f}s | "
                f"R:{roll:6.2f}° P:{pitch:6.2f}° | "
                f"Thr:{state:12s} | "
                f"M1:{m1:4.1f} M2:{m2:4.1f} M3:{m3:4.1f} M4:{m4:4.1f} | "
                f"Hz:{fc.loop_rate:.0f}"
            )

        # ── 6. TIMING ADAPTATIF 100 Hz ─────────────────────────
        # sleep seulement le temps restant → compense le temps déjà écoulé
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
    print("EMERGENCY STOP")