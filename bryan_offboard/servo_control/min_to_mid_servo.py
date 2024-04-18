from gpiozero import Servo
from time import sleep

servo = Servo(26)

try:
    while True:
        servo.min()
        sleep(0.5)
        servo.mid()
        sleep(0.5)
        #servo.max()
    print('servo in min position')
    #sleep(0.5)
except KeyboardInterrupt:
    print("Program stopped")