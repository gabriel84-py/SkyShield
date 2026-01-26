from machine import I2C, Pin
import ssd1306


class OLED:
    def __init__(self, width=128, height=64, i2c_id=0, scl=9, sda=8):
        self.i2c = I2C(
            i2c_id,
            scl=Pin(scl),
            sda=Pin(sda),
            freq=400_000
        )

        self.oled = ssd1306.SSD1306_I2C(
            width,
            height,
            self.i2c
        )

    def show_angles(self, roll, pitch):
        self.oled.fill(0)
        self.oled.text("IMU DATA", 0, 0)
        self.oled.text("Roll : %.1f" % roll, 0, 20)
        self.oled.text("Pitch: %.1f" % pitch, 0, 35)
        self.oled.show()
