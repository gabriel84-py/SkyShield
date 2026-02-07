from flight_controller import FlightController
import time
import sys
import select

# === INIT FLIGHT CONTROLLER ===
print("="*60)
print("   BENCH TEST PID - ROLL & PITCH")
print("="*60)

fc = FlightController()

# === ARMEMENT ===
input("\n⚠️  Branche batterie et appuie ENTER...")
fc.arm()

# === CONFIG INITIALE ===
fc.set_throttle(20)  # throttle base 20% (ajustable)

print("\n" + "="*60)
print("   COMMANDES")
print("="*60)
print("  t+/t-  : Throttle ±5%")
print("  p+/p-  : Kp Roll ±0.1")
print("  d+/d-  : Kd Roll ±0.05")
print("  i+/i-  : Ki Roll ±0.01")
print("  r      : Reset PID")
print("  s      : STOP urgence")
print("  q      : Quitter")
print("="*60)

# === AFFICHAGE INITIAL ===
def print_status(roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch):
    """Affiche status drone"""
    kp_r = fc.pid_roll.kp
    ki_r = fc.pid_roll.ki
    kd_r = fc.pid_roll.kd
    
    print(f"\rRoll:{roll:6.2f}° Pitch:{pitch:6.2f}° | "
          f"T:{fc.base_throttle:3.0f}% | "
          f"M1:{m1:3.0f} M2:{m2:3.0f} M3:{m3:3.0f} M4:{m4:3.0f} | "
          f"cR:{corr_roll:+5.1f} cP:{corr_pitch:+5.1f} | "
          f"Kp:{kp_r:.2f} Ki:{ki_r:.3f} Kd:{kd_r:.2f} | "
          f"{fc.loop_rate:.0f}Hz", end='')

# === LOOP PRINCIPALE ===
try:
    last_time = time.ticks_us()
    
    while True:
        # timing loop (vise 100 Hz = 10ms)
        loop_start = time.ticks_us()
        
        # update contrôleur
        roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch = fc.update()
        
        # affiche status
        print_status(roll, pitch, m1, m2, m3, m4, corr_roll, corr_pitch)
        
        # lecture commande clavier (non-bloquant)
        # Note : sur MicroPython, sys.stdin pas toujours dispo
        # Alternative : utilise REPL ou modifie gains direct dans code
        
        # attente fin cycle (garantit 10ms)
        elapsed = time.ticks_diff(time.ticks_us(), loop_start)
        sleep_time = 10000 - elapsed  # µs
        if sleep_time > 0:
            time.sleep_us(sleep_time)

except KeyboardInterrupt:
    # Ctrl+C → arrêt propre
    print("\n\n=== ARRÊT PROGRAMME ===")
    fc.disarm()
    print("Moteurs arrêtés - Débranche batterie")
    print("✓ Programme terminé")

except Exception as e:
    # erreur → emergency stop
    print(f"\n\n!!! ERREUR : {e} !!!")
    fc.emergency()
    print("Débranche batterie immédiatement !")


# === ALTERNATIVE : MODE INTERACTIF SIMPLIFIÉ ===
"""
Si input() clavier marche pas en loop, utilise ce mode :

1. Lance programme
2. Observe comportement
3. Ctrl+C pr arrêter
4. Modifie gains directement dans code :
   
   fc.set_pid_gains('roll', kp=1.0, kd=0.3)
   
5. Relance programme
6. Répète jusqu'à stable

Ou utilise REPL :
1. Importe : from flight_controller import FlightController
2. Crée : fc = FlightController()
3. Arme : fc.arm()
4. Change : fc.set_throttle(25)
5. Change gains : fc.set_pid_gains('roll', kp=1.2)
6. Loop manuelle :
   while True:
       fc.update()
       time.sleep(0.01)
"""