from gpiozero import Servo
from time import sleep

servo = Servo(27)

try:
    #while True:
    # 	servo.min()
    # 	sleep(0.5)
    servo.value = 0.35
    sleep(0.5)
        # break
        # servo.max()
    print('servo in open position')
    #sleep(0.5)
except KeyboardInterrupt:
    print("Program stopped")