import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")

def getAndUpdateColour():
    while True:
	# Read the data from the sensor
        # Insert code here
        green = (bus.read_byte_data(0x44, 0x09) + bus.read_byte_data(0x44, 0x0A)*255) / 255
        red = (bus.read_byte_data(0x44, 0x0B) + bus.read_byte_data(0x44, 0x0C)*255) / 255
        blue = (bus.read_byte_data(0x44, 0x0D) + bus.read_byte_data(0x44, 0x0E)*255) / 255

        # Convert the data to green, red and blue int values
        # Insert code here
        
        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
        print("RGB(%d %d %d)" % (red, green, blue))
        
        time.sleep(2) 

getAndUpdateColour()
