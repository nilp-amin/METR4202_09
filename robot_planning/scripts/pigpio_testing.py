#!/usr/bin/python3

import pigpio
import time

pi = pigpio.pi()

try:
    while True:
        pi.set_servo_pulsewidth(17, 700)
        time.sleep(1)
        pi.set_servo_pulsewidth(17, 1500)
        time.sleep(1)
except KeyboardInterrupt:
    pi.stop()


"""
from gpiozero import Servo
from time import sleep

servo = Servo(17)

while True:
    servo.min()
    sleep(2)
    servo.mid()
    sleep(2)
    servo.max()
    sleep(2)
"""