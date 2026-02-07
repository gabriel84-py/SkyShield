from machine import Pin, I2C
import time

# essayer les deux bus possibles
i2c0 = I2C(0, scl=Pin(21), sda=Pin(20), freq=100000)
i2c1 = I2C(1, scl=Pin(27), sda=Pin(26), freq=100000)

print("Scan I2C bus 0 (GP20/21):")
devices0 = i2c0.scan()
print([hex(d) for d in devices0])

print("\nScan I2C bus 1 (GP26/27):")
devices1 = i2c1.scan()
print([hex(d) for d in devices1])