#!/usr/bin/env python3

#
# Spin both motors at varying speeds and in both directions.
#
#  State  A  B
#  FWD    0  1
#  REV    1  0
#  STOP   0  0
#  short  1  1
#  
#

import time
import RPi.GPIO as gpio


# GPIO pins
motor1A = 18   # Left track
motor1B = 12
motor2A = 13   # Right track
motor2B = 19

# Frequency
# TODO what is the optimal value here?
FREQ = 50

# Setup
print("Setup")
gpio.setmode(gpio.BCM)
gpio.setup(motor1A, gpio.OUT)
gpio.setup(motor1B, gpio.OUT)
gpio.setup(motor2A, gpio.OUT)
gpio.setup(motor2B, gpio.OUT)

p1a = gpio.PWM(motor1A, FREQ)
p1b = gpio.PWM(motor1B, FREQ)
p2a = gpio.PWM(motor2A, FREQ)
p2b = gpio.PWM(motor2B, FREQ)


print("Start")

p1a.start(0)
p1b.start(0)
p2a.start(0)
p2b.start(0)


def control_motor(motor_name: str, direction: str, percent_speed: int = 0):
    if motor_name == 'left':
        pin_a = p1a
        pin_b = p1b
    elif motor_name == 'right':
        pin_a = p2a
        pin_b = p2b
    else:
        raise ValueError(f"Unknown motor name: {motor_name}")

    if percent_speed > 100 or percent_speed < 0:
        raise ValueError(f"Invalid speed: {percent_speed}")

    if direction == 'stop':
        pin_a.ChangeDutyCycle(0)
        pin_b.ChangeDutyCycle(0)
    elif direction == 'forward':
        pin_a.ChangeDutyCycle(0)
        pin_b.ChangeDutyCycle(percent_speed)
    elif direction == 'reverse':
        pin_a.ChangeDutyCycle(percent_speed)
        pin_b.ChangeDutyCycle(0)
    else:
        raise ValueError(f"Unknown direction name: {direction}")


control_motor('left', 'stop')
control_motor('right', 'stop')
time.sleep(0.5)

try:
    for dir in ['forward', 'reverse']:
        print(f"\t{dir}")
        for p in range(0, 101, 10):
            print(f"\t\t{p}%")
            control_motor('left', dir, p)
            control_motor('right', dir, p)
            time.sleep(0.5)

except KeyboardInterrupt:
    pass

print("Stop")
control_motor('left', 'stop')
control_motor('right', 'stop')
time.sleep(0.1)

p1a.stop(0)
p1b.stop(0)
p2a.stop(0)
p2b.stop(0)

gpio.cleanup()

print("Done")

