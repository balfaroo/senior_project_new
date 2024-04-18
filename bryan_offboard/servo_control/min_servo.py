from gpiozero import Servo
from time import sleep

servo = Servo(26)

try:
    while True:
        servo.min()
        sleep(2)
        #servo.mid()
    print('servo in mid position')
    #sleep(0.5)
except KeyboardInterrupt:
    print("Program stopped")