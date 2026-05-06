#!/usr/bin/env pybricks-micropython
import socket
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

forward_speed = 200
turn_speed = 100

def move_forward():
    left_motor.run(forward_speed)
    right_motor.run(forward_speed)

def move_backward():
    left_motor.run(-forward_speed)
    right_motor.run(-forward_speed)

def turn_left():
    left_motor.run(-turn_speed)
    right_motor.run(turn_speed)

def turn_right():
    left_motor.run(turn_speed)
    right_motor.run(-turn_speed)

def stop():
    left_motor.stop()
    right_motor.stop()

# Start server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0.0.0.0', 12345))   # listen on port 12345
server.listen(1)
print("Waiting for connection...")
conn, addr = server.accept()
print("Connected:", addr)

while True:
    data = conn.recv(1024).decode().strip()
    if not data:
        break

    if data == "forward":
        move_forward()
    elif data == "backward":
        move_backward()
    elif data == "left":
        turn_left()
    elif data == "right":
        turn_right()
    elif data == "stop":
        stop()
    elif data == "quit":
        break

stop()
conn.close()
server.close()