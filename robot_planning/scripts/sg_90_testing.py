#!/usr/bin/python3

import rospy
from gpiozero import Servo
#from time import sleep
from gpiozero import AngularServo
import time

import RPi.GPIO as GPIO


servoPIN = 12
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(5) # Initialization

try:
  while True:
    val = 5
    p.ChangeDutyCycle(val)
    print(val)
    time.sleep(2)
    val = 6.5
    p.ChangeDutyCycle(val)
    print(val)
    time.sleep(2)
except KeyboardInterrupt:
  p.stop()
  GPIO.cleanup()

if __name__ == "__main__":
  """
  s = AngularServo(12)
  s.angle = -90
  time.sleep(2)
  s.angle = 20
  time.sleep(2)
  s.angle = -10
  """
  pass
