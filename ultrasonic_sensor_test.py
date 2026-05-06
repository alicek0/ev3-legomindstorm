#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import wait

ev3 = EV3Brick()
left_motor  = Motor(Port.B)
right_motor = Motor(Port.C)
L = ColorSensor(Port.S1)   # 왼쪽 센서
R = ColorSensor(Port.S4)   # 오른쪽 센서
ultra_sensor = UltrasonicSensor(Port.S2)  # 장애물 감지용
ultra_sensor.distance()  # 초기화용
right_ultra_sensor = UltrasonicSensor(Port.S3)  # 후진용 
right_ultra_sensor.distance()  # 초기화용

while True:
  distance = ultra_sensor.distance() // 10
  
  
  r_distance = right_ultra_sensor.distance() // 10
  print("FD:", distance,"RD:", r_distance)
  
  wait(200)
  