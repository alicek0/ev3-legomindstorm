#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxClient, TextMailbox

ev3 = EV3Brick()
DB_BASE_SPEED = 100
left_motor  = Motor(Port.B)
right_motor = Motor(Port.C)
L = ColorSensor(Port.S1)   # 왼쪽 센서
R = ColorSensor(Port.S4)   # 오른쪽 센서
ultra_sensor = UltrasonicSensor(Port.S2)  # 장애물 감지용
ultra_sensor.distance()  # 초기화용
right_ultra_sensor = UltrasonicSensor(Port.S3)  # 후진용 
right_ultra_sensor.distance()  # 초기화용
ev3 = EV3Brick()
left_motor  = Motor(Port.B)
right_motor = Motor(Port.C)
L = ColorSensor(Port.S1) # [중요] 이 센서가 PID를 전담
R = ColorSensor(Port.S4) # (색상 감지용으로만 사용)
ultra_sensor = UltrasonicSensor(Port.S2)

# --- 2. DriveBase 설정 ---
WHEEL_DIAMETER = 56
AXLE_TRACK = 114
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
def park():
    print("Parking maneuver started (90deg + 2sec)...")
    robot.run_time(-DB_BASE_SPEED*0.5, 0)
    wait(1200)
    robot.turn(90)
    robot.drive(DB_BASE_SPEED*0.5, 0)
    print("Parking maneuver completed.")
    wait(500)
    
    
# 버튼 누르면 시작
ev3.screen.clear()
ev3.screen.print("Press center button")
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
park()