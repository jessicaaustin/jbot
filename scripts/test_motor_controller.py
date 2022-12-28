#!/usr/bin/env python3

#
# Spin both motors at full speed and in both directions.
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


# Setup
print("Setup")
gpio.setmode(gpio.BCM)
gpio.setup(motor1A, gpio.OUT)
gpio.setup(motor1B, gpio.OUT)
gpio.setup(motor2A, gpio.OUT)
gpio.setup(motor2B, gpio.OUT)


def control_motor(motor_name, direction):
    if motor_name == 'left':
        pin_a = motor1A
        pin_b = motor1B
    elif motor_name == 'right':
        pin_a = motor2A
        pin_b = motor2B
    else:
        raise ValueError(f"Unknown motor name: {motor_name}")

    if direction == 'stop':
        gpio.output(pin_a, 0)
        gpio.output(pin_b, 0)
    elif direction == 'forward':
        gpio.output(pin_a, 0)
        gpio.output(pin_b, 1)
    elif direction == 'reverse':
        gpio.output(pin_a, 1)
        gpio.output(pin_b, 0)
    else:
        raise ValueError(f"Unknown direction name: {direction}")


print("Start")
control_motor('left', 'stop')
control_motor('right', 'stop')
time.sleep(0.5)

try:
    print("\tForward")
    control_motor('left', 'forward')
    control_motor('right', 'forward')
    time.sleep(2)

    print("\tReverse")
    control_motor('left', 'reverse')
    control_motor('right', 'reverse')
    time.sleep(2)

    print("\tTurn Left")
    control_motor('left', 'reverse')
    control_motor('right', 'forward')
    time.sleep(2)

    print("\tTurn Right")
    control_motor('left', 'forward')
    control_motor('right', 'reverse')
    time.sleep(2)

except KeyboardInterrupt:
    pass

print("Stop")
control_motor('left', 'stop')
control_motor('right', 'stop')

gpio.cleanup()

print("Done")

