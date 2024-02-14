from gpiozero import Servo
import math
from time import sleep

from gpiozero.pins.pigpio import PiGPIOFactory
//make sure you input "sudo pigpiod" into terminal to have PiGPIOFactory running in the background
factory = PiGPIOFactory()

servo = Servo(18, min_pulse_width=0.6/1000, max_pulse_width=2.24/1000, pin_factory=factory)

while True:
    for i in range(0,360):
        servo.value = math.sin(math.radians(i))
        sleep(0.01)
