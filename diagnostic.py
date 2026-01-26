from machine import I2C, Pin
import time

print("=== Diagnostic I2C ===")

# Tester avec les pins actuelles
print("\nTest I2C0 avec SCL=17, SDA=16:")
try:
    i2c_test = I2C(0, scl=Pin(17), sda=Pin(16), freq=100_000)
    devices = i2c_test.scan()
    print(f"Appareils trouvés: {devices}")
    if devices:
        for addr in devices:
            print(f"  - Adresse: 0x{addr:02X} (décimal: {addr})")
    else:
        print("  AUCUN appareil détecté!")
except Exception as e:
    print(f"  Erreur I2C: {e}")

# Essayer aussi les pins par défaut possibles
print("\nTest I2C0 avec SCL=9, SDA=8:")
try:
    i2c_test2 = I2C(0, scl=Pin(9), sda=Pin(8), freq=100_000)
    devices = i2c_test2.scan()
    print(f"Appareils trouvés: {devices}")
    if devices:
        for addr in devices:
            print(f"  - Adresse: 0x{addr:02X} (décimal: {addr})")
except Exception as e:
    print(f"  Erreur I2C: {e}")

print("\nVérifications à faire:")
print("1. Vérifier les câbles de connexion SCL/SDA")
print("2. Vérifier les résistances pull-up sur le bus I2C")
print("3. Vérifier l'adresse de l'écran OLED (doit être 0x3C)")
print("4. Vérifier l'alimentation de l'écran")
