import smbus
import time
import RPi.GPIO as GPIO

bus = smbus.SMBus(1)
bus.write_byte_data(0x44, 0x01, 0x05)

led = 18 # Se slides week 2 slide 4 for GPIO nr.
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(led,GPIO.OUT)

def mean_list(a):
    return sum(a) / len(a)

def mean_lists(a, b, c):
    liste = []
    for i in range(len(a)):
        liste.append((a[i] + b[i] + c[i]) / 3)
    return sum(liste) / len(liste)

def detectVictim():
    reds = []
    greens = []
    blues = []

    for i in range(60):
        #time.sleep(0.001)
        green = (bus.read_byte_data(0x44, 0x09) + bus.read_byte_data(0x44, 0x0A)*255) / 255
        red = (bus.read_byte_data(0x44, 0x0B) + bus.read_byte_data(0x44, 0x0C)*255) / 255
        blue = (bus.read_byte_data(0x44, 0x0D) + bus.read_byte_data(0x44, 0x0E)*255) / 255 * 1.75
        greens.append(green)
        reds.append(red)
        blues.append(blue)

    red_mean, green_mean, blue_mean = mean_list(reds), mean_list(greens), mean_list(blues)

    #print(f'Red: {red_mean}, Green: {green_mean}, Blue: {blue_mean}')

    if red_mean > green_mean + blue_mean - ((red_mean + green_mean + blue_mean) / 6):
        return True

    return False

def LEDon():
    GPIO.output(led,GPIO.LOW)

def LEDoff():
    GPIO.output(led,GPIO.HIGH)

if __name__ == "__main__":
    try:
        victims = 0
        while True:
            LEDoff()
                    
            if detectVictim():
                victims += 1
                print("Victim detected")
                LEDon()

    except KeyboardInterrupt:
        pass
        print("Victims: ", victims)
            