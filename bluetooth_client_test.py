#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.messaging import BluetoothMailboxClient, TextMailbox

# --- 1. 장치 설정 ---
ev3 = EV3Brick()
left_motor  = Motor(Port.B)
right_motor = Motor(Port.C)
L = ColorSensor(Port.S1)   # 왼쪽 센서
R = ColorSensor(Port.S4)   # 오른쪽 센서
ultra_sensor = UltrasonicSensor(Port.S2)  # 앞차(Car 1)와의 거리 감지용

# --- 2. DriveBase 설정 ---
# [필수] 바퀴 지름과 두 바퀴 사이의 거리를 mm 단위로 정확히 측정하세요!
WHEEL_DIAMETER = 56  # mm (예시 값)
AXLE_TRACK = 114   # mm (예시 값)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)

# --- 3. 블루투스 클라이언트 설정 ---
client = BluetoothMailboxClient()
SERVER_NAME = 'ev3-dongjun' 
mbox = TextMailbox('greeting', client)

print("'" + SERVER_NAME + "'에 연결 시도 중...")
ev3.screen.print("Connecting...")

# 연결될 때까지 재시도하는 루프
while True: 
    try:
        client.connect(SERVER_NAME)
        print("Car 1 연결됨!")
        ev3.screen.clear()
        ev3.screen.print("Connected!")
        ev3.speaker.beep()
        break # 연결 성공 시 루프 탈출
    except Exception as e:
        print("연결 실패:,",e,"3초 후 재시도...")
        wait(3000)

# --- 4. 상수 및 변수 설정 ---
# 거리 유지 (P-제어)
DIST_Kp = 1.0
TARGET_DISTANCE = 200 # mm (20cm)
BASE_SPEED = 280
MAX_SPEED  = 400

# 라인 트래킹 (PID)
LINE_Kp = 8.0
LINE_Ki = 0.01
LINE_Kd = 0.5
LINE_I_CLAMP = 200

# 라인 트래킹 (센서)
BLACK = 6
WHITE = 38
RANGE = max(1, WHITE - BLACK)

# 런타임 변수
line_integral = 0
line_last_err = 0
base_error = 0 # 센서 반사도 보정 값
current_command = "STOP" # 기본값은 정지

# --- 5. 헬퍼 함수 ---

def line_pid_control(error, last_err, integral):
    norm_error = error / RANGE
    integral = max(-LINE_I_CLAMP, min(LINE_I_CLAMP, integral + norm_error))
    deriv = norm_error - last_err
    turn_rate = (LINE_Kp * norm_error) + (LINE_Ki * integral) + (LINE_Kd * deriv)
    return turn_rate, integral, norm_error

def get_color_rgb(sensor):
    r, g, b = sensor.rgb()
    total = r + g + b
    refl = sensor.reflection()
    if total == 0:
        return "BLACK"
    r_ratio, g_ratio, b_ratio = r / total, g / total, b / total
    
    if refl < 15 and total < 60:
        return "BLACK"
    elif (b_ratio > 0.60 and g_ratio < 0.28 and r_ratio < 0.25 and refl < 18 and total < 120):
        return "BLUE"
    elif (r > 100 and g < 30 and b < 30 and refl > 30):
        return "RED"
    elif r_ratio > 0.35 and g_ratio > 0.35 and b_ratio < 0.25 and refl > 40:
        return "YELLOW"
    else:
        return "UNKNOWN"

# [수정됨] 오른쪽 센서 RGB 감지 함수 (Offset 제거)
def get_color_rgb_right(sensor):
    r, g, b = sensor.rgb()
    refl = sensor.reflection()

    total = r + g + b
    if total == 0:
        return "BLACK"
    r_ratio, g_ratio, b_ratio = r / total, g / total, b / total

    # --- BLACK ---
    if refl < 15 and total < 40:
        return "BLACK"
    # --- BLUE ---
    elif b_ratio > 0.45 and g_ratio > 0.3 and r_ratio < 0.25 and refl < 20:
        return "BLUE"
    # --- YELLOW ---
    elif r_ratio > 0.38 and g_ratio > 0.38 and b_ratio < 0.2 and refl > 45:
        return "YELLOW"
    # --- (임시 RED) ---
    elif (r > 100 and g < 30 and b < 30 and refl > 30):
        return "RED"
    else:
        return "UNKNOWN"


# ---------------------
# 메인 루프 시작
# ---------------------
ev3.screen.clear()
ev3.screen.print("Starting Follower...")

try:
    while True:
        # 1. [메시지 수신]
        new_cmd = mbox.read()
        if new_cmd:
            current_command = new_cmd
            print("새 명령 수신:", current_command)

        # 2. [주행] 현재 명령에 따라 행동
        if current_command == "STOP":
            robot.stop()
            wait(100)
            continue
        
        elif current_command == "PARK":
            print("주차 명령 수신. 정지.")
            robot.stop()
            break

        # "RESUME" 또는 "SLOW_DOWN" 명령일 때만 주행 로직 실행
        elif current_command == "RESUME" or current_command == "SLOW_DOWN" or current_command == "hello":
            robot.drive(100, 0)

        wait(20)

except Exception as e:
    print("오류 발생:",{e})
finally:
    robot.stop()
    print("팔로워 프로그램 종료.")