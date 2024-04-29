import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

def detectVictim():
        green = (bus.read_byte_data(0x44, 0x09) + bus.read_byte_data(0x44, 0x0A)*255) / 255
        red = (bus.read_byte_data(0x44, 0x0B) + bus.read_byte_data(0x44, 0x0C)*255) / 255
        blue = (bus.read_byte_data(0x44, 0x0D) + bus.read_byte_data(0x44, 0x0E)*255) / 255 * 1.75

        if green > 150 or red > 150 or blue > 150:
                return True