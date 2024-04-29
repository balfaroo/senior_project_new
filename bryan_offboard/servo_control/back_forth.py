'''script for moving claw between open and closed positions'''
from gpiozero import Servo
from time import sleep

servo = Servo(27)

try:
    while True:
        servo.min()
        sleep(0.5)
        servo.value = 0.25
        sleep(0.5)
except KeyboardInterrupt:
    print("Program stopped")