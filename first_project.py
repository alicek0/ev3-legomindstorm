#!/usr/bin/env pybricks-micropython

import socket
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait

# # Initialize the EV3 brick.
ev3 = EV3Brick()

# Play a sound.
# ev3.speaker.beep ()

# Initialize two motors at ports B and C.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize color sensors and gyro sensor.
left_color_sensor = ColorSensor(Port.S1)
right_color_sensor = ColorSensor(Port.S4)

# Set start boolean.
start = False

# Set speed variables.
forward_speed = 200
turn_speed = 100
left_motor_speed = 100
right_motor_speed = 100

# Set color sensor thresholds.
# black < green < red < white
white_thres = 25
red_thres = 18
green_thres = 9
black_thres = 5  # black if less than black_thres



def calibrate_sensor(sensor: ColorSensor, name: str):
    # Step 1: Black calibration
    ev3.speaker.say("Place " + name + " on black")
    wait(3000)  # wait 3s
    black = sensor.reflection()
    ev3.speaker.beep()

    # Step 2: White calibration
    ev3.speaker.say("Place " + name + " on white")
    wait(3000)
    white = sensor.reflection()
    ev3.speaker.beep()

    threshold = (black + white) / 2

    # Show results on screen
    ev3.screen.print(name, "B:", black, "W:", white, "T:", threshold)
    print(name, "B:", black, "W:", white, "T:", threshold)

    # Return everything
    return {
        "black": black,
        "white": white,
        "threshold": threshold
    }
  
  
# Calibrate both sensors

# left_data = {"black": 5, "white": 43, "red": 39, "threshold": 24.0}
# right_data = {"black": 7, "white": 51, "red": 36, "threshold": 29.0}




# Function to move forward for n milliseconds
def move_forward(duration_ms):
  left_motor.run(forward_speed)
  right_motor.run(forward_speed)
  wait(duration_ms)
  left_motor.stop()
  right_motor.stop()
  
def turn_left(angle):
  left_motor.run()
  
def difference(a, b):
  return a-b

    
while not start:
  
  # Display reflection values on the EV3 screen.
  l_reflection = left_color_sensor.reflection()
  r_reflection = right_color_sensor.reflection()
  
  # l_color = left_color_sensor.color()
  # r_color = right_color_sensor.color()
  
  ev3.screen.clear()
  reflection = "L:" + str(l_reflection) + " R:" + str(r_reflection)
  ev3.screen.print (reflection)
  print(reflection)
  wait(100)
  
  # If detect red start.
  
  # if difference(l_reflection, r_reflection) > 5:
  #   print("Turning right")
  #   left_motor.run(turn_speed)
  #   right_motor.run(turn_speed)
  
  
if start:
  l_reflection = left_color_sensor.reflection()
  r_reflection = right_color_sensor.reflection()
  l_color = left_color_sensor.color()
  r_color = right_color_sensor.reflection()
  
  # # Go forward if black.
  
  # # Turn left if left sensor doesn't detect black and right sensor doesn't detect black

  
  # # Stop if green.

  
  # # If color sensors detect no colors stop.
  if l_reflection <= 0 and r_reflection <= 0:
    start = False