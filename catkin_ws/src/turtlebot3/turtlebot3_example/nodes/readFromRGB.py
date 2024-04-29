import smbus
import time
import RPi.GPIO as GPIO

bus = smbus.SMBus(1)
bus.write_byte_data(0x44, 0x01, 0x05)

led = 18 # Se slides week 2 slide 4 for GPIO nr.
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(led,GPIO.OUT)

def mean(a, b, c):
        return (a + b + c) / 3

def detectVictim():
        green = (bus.read_byte_data(0x44, 0x09) + bus.read_byte_data(0x44, 0x0A)*255) / 255
        red = (bus.read_byte_data(0x44, 0x0B) + bus.read_byte_data(0x44, 0x0C)*255) / 255
        blue = (bus.read_byte_data(0x44, 0x0D) + bus.read_byte_data(0x44, 0x0E)*255) / 255 * 1.75

        if mean(red, green, blue) < 3:
                return True
        return False

def blinkLED():
        GPIO.output(led,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(led,GPIO.LOW)

