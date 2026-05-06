#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait
from pybricks.messaging import BluetoothMailboxServer, TextMailbox
from pybricks.robotics import DriveBase
import time

# --- 테스트용 변수 ---
# 단일 주행 시 속도를 높게 설정해둠 
# 차선에 따라 기본 주행 달라짐 
# 빨간색을 만난 후에 주차장 탐지
test_without_bluetooth = False  # just test without bluetooth
right_lane = False # 시작 차선
two_connections = True  # check if there will be two bluetooth connections
met_red = False  # park detection only happens after meeting red

# --- 장치 및 DriveBase 설정 ---
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

# --- 상수 및 변수 설정 ---
DB_BASE_SPEED = 150 -60
DB_KP = 2.0     
DB_KI = 0.01
DB_KD = 0.5
DB_I_CLAMP = 200
MANEUVER_POWER = 280
if test_without_bluetooth:
    DB_BASE_SPEED = 200
    DB_KP = 8.0     
    DB_KI = 0
    DB_KD = 0

# 상태 변수
integral = 0
last_err = 0
obstacle_detected = False
obstacle_detected_count = 0
in_crosswalk = False
right_ultra_sensor_detected = 0

# 쿨타임
COOLDOWN_MS = 3000
last_detection_time = 0

# --- 헬퍼 함수 ---

# --- 명령 전송 헬퍼 함수 ---
last_command_sent = ""
def send_command(command):
    if not test_without_bluetooth:
        global last_command_sent
        if command != last_command_sent:
            try:
                mbox.send(command)
                if two_connections:
                    if command == "PARK":
                        print("Waiting for car1 ready")
                        while mbox.read() != "PARK_READY":
                            wait(20)
                    mbox2.send(command)
                print("명령 전송: {}".format(command))
                last_command_sent = command
            except OSError as e:
                print("통신 오류(무시함):", e)
                pass

# 왼쪽 센서 색상 반환
def get_color_rgb(sensor):
    r, g, b = sensor.rgb()
    total = r + g + b
    refl = sensor.reflection()
    print("LeftRGB", r, g, b, "rfl", refl)  # debug line
    if total == 0: return "BLACK"
    r_ratio, g_ratio, b_ratio = r / total, g / total, b / total
    if refl < 15 and total < 60: return "BLACK"
    elif (r<=14 and g>=19 and g<= 50 and b >= 90 and refl < 11): return "BLUE"
    elif r_ratio > 0.35 and g_ratio > 0.35 and b_ratio < 0.25 and refl > 40: return "YELLOW"
    elif total > 180 and abs(r_ratio - g_ratio) < 0.12 and abs(g_ratio - b_ratio) < 0.12: return "WHITE"
    elif g_ratio > 0.35 and refl > 30: return "GREEN"
    elif (r > 35 and g < 40 and b < 40 and refl > 25): return "RED"
    else: return "UNKNOWN"

# 오른쪽 센서 색상 반환
def get_color_rgb_right(sensor):
    r, g, b = sensor.rgb()
    refl = sensor.reflection()
    total = r + g + b
    print("RightRGB", r, g, b, "rfl", refl)  # debug line
    if total == 0: return "BLACK"
    r_ratio, g_ratio, b_ratio = r / total, g / total, b / total
    if refl < 15 and total < 40: return "BLACK"
    elif b_ratio > 0.45 and g_ratio > 0.3 and r_ratio < 0.25 and refl < 20: return "BLUE"
    elif r_ratio > 0.38 and g_ratio > 0.38 and b_ratio < 0.2 and refl > 45: return "YELLOW"
    elif total > 160 and abs(r_ratio - g_ratio) < 0.1 and abs(g_ratio - b_ratio) < 0.1 and refl > 45: return "WHITE"
    elif g_ratio > 0.4 and g > r and g > b and refl > 25: return "GREEN"
    elif (r > 35 and g < 40 and b < 40 and refl > 25): return "RED"
    else: return "UNKNOWN"

# PID 함수
def pid_control_db(error, last_err, integral):
    integral = max(-DB_I_CLAMP, min(DB_I_CLAMP, integral + error))
    deriv = error - last_err
    turn_rate = DB_KP * error + DB_KI * integral + DB_KD * deriv
    return turn_rate, integral

# 장애물 회피 함수 
def avoid_obstacle():
    global right_lane, integral, last_err
    
    direction = -1 if right_lane else 1
    print("Obstacle Avoid: Changing Lane. Direction:", direction)
    
    # 반대 차선으로 45도 방향 전환
    robot.stop()
    robot.turn(45 * direction)
    
    # 중앙선 통과
    robot.drive(DB_BASE_SPEED, 0)
    if test_without_bluetooth: # 속도가 빠르니 더 짧은 시간동안 전진
        wait(1600)
    else:
        wait(2400) # 중앙선 넘을 때까지 충분히 직진
    
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
    
    right_lane = not right_lane
    integral = last_err = 0
    print("Lane changed completely.")    
    
def park():
    global integral, last_err, current_command
    print("Parking maneuver started (90deg + 2sec)...")
    
    # 팔로워들의 주차 완료 신호 대기
    if not test_without_bluetooth:
        print("Waiting for Followers...")
        
        # 상태 플래그
        car1_ready = False
        if not two_connections:
            while not car1_ready:
                msg1 = mbox.read()
                if msg1 == "PARK_READY" and not car1_ready:
                    print("Car 1 Ready!")
                    car1_ready = True
        
        car2_ready = False if two_connections else True # 2대 연결 아니면 이미 된 걸로 침
            
        while not car2_ready:
            msg2 = mbox2.read()
            if msg2 == "PARK_READY" and not car2_ready:
                print("Car 2 Ready!")
                car2_ready = True
            
        print("All Followers Ready. Leader Parking...")
    
        send_command("ALL_PARK_READY")
    
    # go back to match start of parking space
    robot.drive(-DB_BASE_SPEED*0.5, 0)
    wait(2300)
    if test_without_bluetooth:
        wait(1000)
    
    # turn right to face parking space
    robot.turn(97)
    
    # go front for 1 second
    robot.drive(DB_BASE_SPEED*1.3, 0)
    wait(1000) 
    
    # go back until front sensor distance is less than 13cm
    obstacle_dist = ultra_sensor.distance()
    while obstacle_dist > 130:
        robot.drive(DB_BASE_SPEED*1.3, 0)
        obstacle_dist = ultra_sensor.distance()
        wait(20) 
        
    print("distance to obstacle:", obstacle_dist)
    send_command("STOP")
    robot.stop()

# --- 블루투스 서버 설정 ---
if not test_without_bluetooth:
    server = BluetoothMailboxServer()
    mbox = TextMailbox('CMD', server)
    print("Waiting for connection...")
    if two_connections: 
        mbox2 = TextMailbox('CMD2', server)
        server.wait_for_connection(2)
        print("Car 2, 3 연결됨!")
    else:
        server.wait_for_connection()
        print("Car 2 연결됨!")

# --- 시작 및 보정 ---
ev3.screen.clear()
ev3.screen.print("Ready to Start")
ev3.screen.print("Place on BLACK road")
ev3.screen.print("Press CENTER")
ev3.speaker.beep()

# 버튼 누를 때까지 대기
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)

ev3.screen.clear()
ev3.screen.print("Calibrating...")
wait(500)

# 검은색 값 측정 (양쪽 다)
l_black = L.reflection()
r_black = R.reflection()
print("Black - L: {}, R: {}".format(l_black, r_black))

TARGET_L = l_black + 12 
TARGET_R = r_black + 12 

print("TARGET - L: {}, R: {}".format(TARGET_L, TARGET_R))
ev3.screen.print("GO!")
wait(1000)

print("BASE SPEED:",DB_BASE_SPEED)  # debug line

# --- 메인 루프 ---
try:
    while True:
        # --- 주차 ---
        if met_red:
            now = time.ticks_ms()
            distance = right_ultra_sensor.distance()
            if distance < 200:
                if now - last_detection_time > COOLDOWN_MS:
                    right_ultra_sensor_detected += 1
                    last_detection_time = now
            if right_ultra_sensor_detected >= 2:
                send_command("STOP"); robot.stop(); send_command("PARK"); park(); break 

        # --- 장애물 회피 및 차선 변경 ---
        # 두번 이상 25cm 이하 장애물 인식시 작동 (오작동 방지)
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
                elapsed += 20; 
                if test_without_bluetooth: wait(10)
                else: wait(20)
                if elapsed > 5000: break
            print("Crosswalk Done."); send_command("RESUME"); integral = last_err = 0; continue

        # --- PID 제어 ---
        send_command("RESUME")
        
        if not right_lane:
            # 왼쪽 차선 (L 센서 사용)
            error = L.reflection() - TARGET_L
            turn_rate, integral = pid_control_db(error, last_err, integral)
            robot.drive(DB_BASE_SPEED, turn_rate) # 정상 방향, 오른쪽으로 꺾어야 함 
        else:
            # 오른쪽 차선 (R 센서 사용)
            error = R.reflection() - TARGET_R
            turn_rate, integral = pid_control_db(error, last_err, integral)
            robot.drive(DB_BASE_SPEED, -turn_rate) # 반대 방향, 왼쪽으로 꺾어야 함 
        
        last_err = error
        if test_without_bluetooth: # 빠른 속도로 주행 시 더 빠른 반응 속도
            wait(10)
        else: 
            wait(20)
        

except Exception as e:
    print("Error:", e)
finally:
    robot.stop()