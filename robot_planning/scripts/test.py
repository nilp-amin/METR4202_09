import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

center = 7.5
left = 12
right = 1
delay = 2
initdelay = 0.05
servo_pin = 12
freq = 50 

GPIO.setup(servo_pin, GPIO.OUT)
time.sleep(initdelay)
mypwm = GPIO.PWM(servo_pin,freq)

print('Initialized PWM pin 12 at 50 Hertz')
mypwm.start(center)
print('Started duty cycle at center postion')
time.sleep(delay)
mypwm.ChangeDutyCycle(left)
print('change duty cycle to left position')
time.sleep(delay)
mypwm.ChangeDutyCycle(right)
print('change duty cycle to right postion')
time.sleep(delay)

GPIO.cleanup()

