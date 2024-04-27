from gpiozero import Servo
from time import sleep

servo = Servo(27)

try:
    #while True:
    servo.min()
    sleep(0.5)
        # servo.max()
        # sleep(1)
        
        #servo.mid()
    print('servo in closed position')
    #sleep(0.5)
except KeyboardInterrupt:
    print("Program stopped")