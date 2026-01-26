from machine import I2C, Pin

# BUS I2C UNIQUE DU PROJET
i2c = I2C(
    0,
    scl=Pin(17),
    sda=Pin(16),
    freq=100_000   # volontairement lent et stable
)
