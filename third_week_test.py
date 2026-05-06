#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.messaging import BluetoothMailboxServer, TextMailbox
from pybricks.robotics import DriveBase
import time

just_test_obstacle = False
right_lane = True # [시작 차선]

# --- 1. 블루투스 서버 설정 ---
if not just_test_obstacle:
    server = BluetoothMailboxServer()
    mbox = TextMailbox('CMD', server)
    print("블루투스 서버 시작. 연결 대기 중...")
    server.wait_for_connection()
    print("Car 2 연결됨!")

# --- 2. 명령 전송 헬퍼 함수 ---
last_command_sent = ""
def send_command(command):
    if not just_test_obstacle:
        global last_command_sent
        if command != last_command_sent:
            try:
                mbox.send(command)
                print("명령 전송: {}".format(command))
                last_command_sent = command
            except OSError as e:
                print("통신 오류(무시함):", e)
                pass

# --- 3. 장치 및 DriveBase 설정 ---
ev3 = EV3Brick()
left_motor  = Motor(Port.B)
right_motor = Motor(Port.C)
L = ColorSensor(Port.S1) 
R = ColorSensor(Port.S4) 
ultra_sensor = UltrasonicSensor(Port.S2)
right_ultra_sensor = UltrasonicSensor(Port.S3)

WHEEL_DIAMETER = 56
AXLE_TRACK = 114
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)

# --- 4. 상수 및 변수 설정 ---
DB_BASE_SPEED = 150
DB_KP = 2.0     
DB_KI = 0.01
DB_KD = 0.5
DB_I_CLAMP = 200

MANEUVER_POWER = 280

# 상태 변수
integral = 0
last_err = 0
obstacle_detected = False
obstacle_detected_count = 0
in_crosswalk = False
right_ultra_sensor_detected = 0

# [수정됨] 센서별 목표값 분리 변수 선언
TARGET_L = 0
TARGET_R = 0

# 쿨타임
COOLDOWN_MS = 1000
last_detection_time = 0
met_red = False

# --- 5. 헬퍼 함수 ---
def get_color_rgb(sensor):
    r, g, b = sensor.rgb()
    total = r + g + b
    refl = sensor.reflection()
    print("LeftRGB", r, g, b, "rfl", refl)
    if total == 0: return "BLACK"
    r_ratio, g_ratio, b_ratio = r / total, g / total, b / total
    if refl < 15 and total < 60: return "BLACK"
    # elif (b_ratio > 0.60 and g_ratio < 0.30 and r_ratio < 0.30 and refl < 18 and total < 120): return "BLUE"
    elif (r<=14 and g>=19 and g<= 50 and b >= 90 and refl < 11): return "BLUE"
    elif r_ratio > 0.35 and g_ratio > 0.35 and b_ratio < 0.25 and refl > 40: return "YELLOW"
    elif total > 180 and abs(r_ratio - g_ratio) < 0.12 and abs(g_ratio - b_ratio) < 0.12: return "WHITE"
    elif g_ratio > 0.35 and refl > 30: return "GREEN"
    elif (r > 80 and g < 40 and b < 40 and refl > 25): return "RED"
    else: return "UNKNOWN"

def get_color_rgb_right(sensor):
    r, g, b = sensor.rgb()
    refl = sensor.reflection()
    total = r + g + b
    if total == 0: return "BLACK"
    r_ratio, g_ratio, b_ratio = r / total, g / total, b / total
    if refl < 15 and total < 40: return "BLACK"
    elif b_ratio > 0.45 and g_ratio > 0.3 and r_ratio < 0.25 and refl < 20: return "BLUE"
    elif r_ratio > 0.38 and g_ratio > 0.38 and b_ratio < 0.2 and refl > 45: return "YELLOW"
    elif total > 160 and abs(r_ratio - g_ratio) < 0.1 and abs(g_ratio - b_ratio) < 0.1 and refl > 45: return "WHITE"
    elif g_ratio > 0.4 and g > r and g > b and refl > 25: return "GREEN"
    elif (r > 80 and g < 40 and b < 40 and refl > 25): return "RED"
    else: return "UNKNOWN"

def pid_control_db(error, last_err, integral):
    integral = max(-DB_I_CLAMP, min(DB_I_CLAMP, integral + error))
    deriv = error - last_err
    turn_rate = DB_KP * error + DB_KI * integral + DB_KD * deriv
    return turn_rate, integral

def avoid_obstacle():
    global right_lane, integral, last_err
    
    direction = -1 if right_lane else 1
    print("Obstacle Avoid: Changing Lane. Direction:", direction)
    
    robot.stop()
    robot.turn(45 * direction)
    
    # Blind Drive (중앙선 통과)
    robot.drive(DB_BASE_SPEED, 0)
    wait(2000) # 중앙선 넘을 때까지 충분히 직진
    
    # Green 찾기 (반대편 엣지 도달)
    print("Searching for Edge (Green/Black)...")
    while True:
        lc = get_color_rgb(L)
        rc = get_color_rgb_right(R) 
        
        # 도로 밖(초록)이나 검정(도로)을 만나면 정지
        if direction == -1: # Going Left
            if lc == "GREEN" or lc == "BLACK": break
        else: # Going Right
            if rc == "GREEN" or rc == "BLACK": break
        
        wait(10)

    # 자세 정렬
    robot.stop()             
    robot.turn(-45 * direction) 
    
    # [수정됨] 문법 에러 수정 및 정밀 정렬
    # print("Fine aligning...")
    # 살짝만 더 꺾어서 확실히 도로 안쪽을 보게 함
    # robot.turn(-10 * direction) 
    
    right_lane = not right_lane
    integral = last_err = 0
    print("Lane changed completely.")    
    
def park():
    global integral, last_err
    print("Parking...")
    robot.turn(90)
    robot.run_time(DB_BASE_SPEED, 2000) 
    integral = last_err = 0
    wait(500)

# --- 6. 시작 및 보정 (수정됨) ---
ev3.screen.clear()
ev3.screen.print("Ready to Start")
ev3.screen.print("Place on BLACK road")
ev3.screen.print("Press CENTER")

# 버튼 누를 때까지 대기
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)

ev3.screen.clear()
ev3.screen.print("Calibrating...")
wait(500)

# 1. 검은색 값 측정 (양쪽 다)
l_black = L.reflection()
r_black = R.reflection()
print("Black - L: {}, R: {}".format(l_black, r_black))

# (옵션) 여기서 흰색/초록색 위에 놓고 측정하면 더 좋지만, 
# 편의상 '검은색 + 범위'로 타겟 설정
# 보통 검은색(5~10) + 15 정도 하면 엣지 값이 됨
TARGET_L = l_black + 12 
TARGET_R = r_black + 12 

print("TARGET - L: {}, R: {}".format(TARGET_L, TARGET_R))
ev3.screen.print("GO!")
wait(1000)

# --- 7. 메인 루프 ---
try:
    while True:
        # --- 주차 및 장애물 로직 (생략없이 기존 로직 유지) ---
        if met_red:
            now = time.ticks_ms()
            distance = right_ultra_sensor.distance()
            if distance < 300:
                if now - last_detection_time > COOLDOWN_MS:
                    right_ultra_sensor_detected += 1
                    last_detection_time = now
            if right_ultra_sensor_detected >= 2:
                send_command("STOP"); robot.stop(); send_command("PARK"); park(); break 


        # 두번 이상 25cm 이하 장애물 인식시 작동 (오작동 방지))
        obstacle_dist = ultra_sensor.distance()
        if obstacle_dist < 250:
            print("Obstacle found:", obstacle_dist)
            obstacle_detected_count += 1
            if obstacle_detected_count >= 2:
                send_command("STOP"); robot.stop(); 
                if right_lane: send_command("CHANGE_LANE_LEFT")
                else: send_command("CHANGE_LANE_RIGHT")
                avoid_obstacle(); 
                obstacle_detected = True; obstacle_detected_count = 0; 
                send_command("RESUME"); integral = last_err = 0; continue
        elif obstacle_dist > 250: 
            obstacle_detected = False; obstacle_detected_count = 0

        # --- 색상 및 PID ---
        lc = get_color_rgb(L)
        rc = get_color_rgb_right(R) 
        
        if lc == "RED" or rc == "RED":
            if not met_red: print("RED LINE."); met_red = True
        elif lc == "BLUE" or rc == "BLUE":
            print("BLUE detected."); send_command("STOP"); robot.stop(); wait(3000)
            while get_color_rgb(L) == "BLUE" or get_color_rgb_right(R) == "BLUE":
                robot.drive(DB_BASE_SPEED, 0); wait(20) # 직진 통과
            send_command("RESUME"); integral = last_err = 0; continue
        elif lc == "YELLOW" or rc == "YELLOW":
            print("YELLOW Crosswalk."); send_command("SLOW")
            exit_counter = 0; EXIT_THRESHOLD = 25; elapsed = 0; robot.stop()
            while True:
                robot.drive(DB_BASE_SPEED // 2, 0) # 직진
                # 흰색, 노랑이 안 보이면(검정, 초록이면) 카운트 증가
                curr_l, curr_r = get_color_rgb(L), get_color_rgb_right(R)
                if (curr_l not in ["WHITE", "YELLOW"]) and (curr_r not in ["WHITE", "YELLOW"]):
                    exit_counter += 1
                else: exit_counter = 0
                
                if exit_counter >= EXIT_THRESHOLD: break
                elapsed += 20; wait(20)
                if elapsed > 5000: break
            print("Crosswalk Done."); send_command("RESUME"); integral = last_err = 0; continue

        # --- [PID 제어 핵심 수정] ---
        send_command("RESUME")
        
        if not right_lane:
            # 왼쪽 차선 (L 센서 사용)
            error = L.reflection() - TARGET_L
            turn_rate, integral = pid_control_db(error, last_err, integral)
            robot.drive(DB_BASE_SPEED, turn_rate) # 정상 방향
        else:
            # 오른쪽 차선 (R 센서 사용)
            error = R.reflection() - TARGET_R
            turn_rate, integral = pid_control_db(error, last_err, integral)
            robot.drive(DB_BASE_SPEED, -turn_rate) # 반대 방향
        
        last_err = error
        wait(20)

except Exception as e:
    print("Error:", e)
finally:
    robot.stop()
    send_command("STOP")