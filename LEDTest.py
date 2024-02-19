import RPi.GPIO as GPIO
import time

led = 16

while True:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(led,GPIO.OUT)
    print("LED on")
    GPIO.output(led,GPIO.HIGH)
    time.sleep(1)
    print("LED off")
    GPIO.output(led,GPIO.LOW)
    time.sleep(1)