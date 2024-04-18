from gpiozero import Servo
from time import sleep

servo = Servo(26)

try:
    while True:
    # 	servo.min()
    # 	sleep(0.5)
        servo.value = 0.2
        sleep(2)
        # servo.max()
    print('servo in max position')
    #sleep(0.5)
except KeyboardInterrupt:
    print("Program stopped")