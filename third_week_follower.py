#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxClient, TextMailbox

# --- 장치 설정 ---
ev3 = EV3Brick()
left_motor  = Motor(Port.B)
right_motor = Motor(Port.C)
L = ColorSensor(Port.S1) 
R = ColorSensor(Port.S4) 
ultra_sensor = UltrasonicSensor(Port.S2)

# --- DriveBase 설정 ---
WHEEL_DIAMETER = 56
AXLE_TRACK = 114
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)

# --- 블루투스 클라이언트 설정 ---
client = BluetoothMailboxClient()
LEADER_BRICK_NAME = "ev3-dongjun"
# LEADER_BRICK_NAME = "ev3-qhy"
mbox = TextMailbox('CMD', client)

print("'{}'에 연결 시도 중...".format(LEADER_BRICK_NAME))
ev3.screen.print("Connecting...")

while True:
    try:
        client.connect(LEADER_BRICK_NAME)
        print("Car 1 연결됨!")
        ev3.screen.clear()
        ev3.screen.print("Connected!")
        ev3.speaker.beep()
        break
    except Exception as e:
        print("연결 실패: {}. 3초 후 재시도...".format(e))
        wait(3000)

# --- 상수 및 변수 설정 ---
# DriveBase PID
DB_BASE_SPEED = 100 
DIST_Kp = 0.5         
MAX_SPEED = 150      
VALID_DIST_MAX = 2000 

# --- PID Constants ---
LINE_Kp = 2.0      
LINE_Ki = 0.01     
LINE_Kd = 0.5      
LINE_I_CLAMP = 200 

# Calibration targets 
TARGET_L = 20 
TARGET_R = 22  

# 상태 변수
integral = 0
last_err = 0
right_lane = False

# 보정 및 상수
BLACK = 6
WHITE = 15
TARGET = (BLACK + WHITE) / 2 
# RANGE = max(1, WHITE - BLACK) # (참고용)

current_command = "STOP"

# --- 헬퍼 함수 ---

def line_pid_control(error, last_err, integral):
    integral = max(-LINE_I_CLAMP, min(LINE_I_CLAMP, integral + error))

    deriv = error - last_err

    turn_rate = LINE_Kp * error + LINE_Ki * integral + LINE_Kd * deriv
    return turn_rate, integral

def get_color_rgb(sensor):
    r, g, b = sensor.rgb()
    total = r + g + b
    refl = sensor.reflection()
    if total == 0: return "BLACK"
    r_ratio, g_ratio, b_ratio = r / total, g / total, b / total
    if refl < 15 and total < 60: return "BLACK"
    else: return "UNKNOWN"
def get_color_rgb_right(sensor):
    return get_color_rgb(sensor)

def avoid_obstacle():
    global right_lane, integral, last_err
    direction = -1 if right_lane else 1
    if direction == -1: print("Obstacle: Turning LEFT 45 deg...")
    else: print("Obstacle: Turning RIGHT 45 deg...")
    robot.turn(45 * direction)
    robot.drive(DB_BASE_SPEED, 0)
    wait(2000)
    if direction == -1:
        while get_color_rgb(L) == "BLACK": wait(10)
    else:
        while get_color_rgb_right(R) == "BLACK": wait(10)
    robot.stop(Stop.BRAKE)
    robot.turn(-45 * direction)
    right_lane = not right_lane
    integral = last_err = 0
    print("Obstacle avoided.")

def park():
    global integral, last_err, current_command
    print("Parking maneuver started (90deg + 2sec)...")
    # go back a little to fit into park space
    robot.drive(-DB_BASE_SPEED*0.6, 0)
    wait(1500)
    # turn right 90 degrees
    robot.turn(90)
    
    # go forward for 1 second
    robot.drive(DB_BASE_SPEED*0.6, 0)
    wait(1000) 

    # go forward until recieving STOP command
    while current_command != "STOP":
        current_command = mbox.read()
        robot.drive(DB_BASE_SPEED*0.5, 0)
        wait(20) 
    robot.stop()

# --- 메인 루프 시작 ---
ev3.screen.clear()
ev3.screen.print("Starting Follower...")

try:
    while True:
        # 명령 수신 
        new_cmd = mbox.read()
        
        # Only update current_command if it's new
        if new_cmd and new_cmd != current_command:
            current_command = new_cmd
            print("New Command:", current_command)
            # Reset processing flag for lane changes when a new command arrives
            if "CHANGE_LANE" in current_command:
                processed_lane_change = False
        
        # 명령에 따라 행동 결정
        if current_command == "STOP":
            robot.stop()
            wait(100)
            continue
        
        # Lane Change Logic
        elif current_command == "CHANGE_LANE_LEFT":
            # Only execute if we haven't processed it yet AND we are actually in the right lane
            if not processed_lane_change and right_lane == True:
                print("Executing Lane Change LEFT...")
                avoid_obstacle() # This toggles right_lane to False
                processed_lane_change = True # Mark as done
                # Explicitly switch to RESUME to avoid loop re-entry
                current_command = "RESUME" 

        elif current_command == "CHANGE_LANE_RIGHT":
             # Only execute if we haven't processed it yet AND we are actually in the left lane
            if not processed_lane_change and right_lane == False:
                print("Executing Lane Change RIGHT...")
                avoid_obstacle() # This toggles right_lane to True
                processed_lane_change = True
                current_command = "RESUME"
        
        elif current_command == "PARK":
            print("주차 명령 수신. 주차 실행.")
            try:
                print("Send park_ready")
                mbox.send("PARK_READY")
            except OSError as e:
                print("통신 오류(무시함):", e)
                pass
            robot.stop()
            park()
            robot.stop()
            print("Parking maneuver completed.")
            break
        
        elif current_command == "RESUME" or current_command == "SLOW":
            
            # 속도 결정 (20-30cm 구간 정속 주행 로직 적용)
            raw_distance = ultra_sensor.distance()
            
            # 오류 값이거나 너무 가까우면(6cm 미만) 정지
            if raw_distance < 60:
                drive_speed = 0
            else:
                print("distance:", raw_distance)
                
                if 200 <= raw_distance <= 300:
                    drive_speed = DB_BASE_SPEED
                elif raw_distance < 200:
                    error = raw_distance - 200 # 음수 값
                    drive_speed = DB_BASE_SPEED + (error * DIST_Kp * 2)
                else: 
                    error = raw_distance - 300 # 양수 값
                    drive_speed = DB_BASE_SPEED + (error * DIST_Kp)
                
                # SLOW 모드 확인
                if current_command == "SLOW":
                    current_max_speed = DB_BASE_SPEED//2
                else:
                    current_max_speed = MAX_SPEED
                
                # 속도 범위 제한 (음수 방지 및 최대 속도 제한)
                drive_speed = max(0, min(current_max_speed, drive_speed))
            
            # PID 주행
            if not right_lane:
                error = L.reflection() - TARGET_L
                turn_rate, integral = line_pid_control(error, last_err, integral)
                robot.drive(drive_speed, turn_rate)
            else:
                error = R.reflection() - TARGET_R
                turn_rate, integral = line_pid_control(error, last_err, integral)
                robot.drive(drive_speed, -turn_rate)

            last_err = error

        wait(20)

except Exception as e:
    print("오류 발생: {}".format(e))
    ev3.screen.print("ERROR:\n{}".format(e))

finally:
    robot.stop()
    print("팔로워 프로그램 종료.")