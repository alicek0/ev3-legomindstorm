#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import wait
import time


ev3 = EV3Brick()
left_motor  = Motor(Port.B)
right_motor = Motor(Port.C)
L = ColorSensor(Port.S1)   # 왼쪽 센서
R = ColorSensor(Port.S4)   # 오른쪽 센서
ultra_sensor = UltrasonicSensor(Port.S2)  # 장애물 감지용
ultra_sensor.distance()  # 초기화용
right_ultra_sensor = UltrasonicSensor(Port.S3)  # 후진용 
right_ultra_sensor.distance()  # 초기화용


# RGB 또는 reflection 값 기준으로 보정
l_r, l_g, l_b = L.rgb()
r_r, r_g, r_b = R.rgb()

# 오른쪽 센서 보정치 계산
color_offset = [(l_r - r_r),(l_g - r_g),(l_b - r_b)]

obstacle_detected = False # 장애물 하나만 있게

max_obstacle_distance = 30  # cm
# right obstacle sensor 인식 쿨타임
COOLDOWN_MS = 1000  # 1초 쿨타임
last_detection_time = 0

BASE_SPEED = 280
MAX_SPEED  = 400

Kp, Ki, Kd = 1.0, 0.0, 1.0
I_CLAMP = 200

in_crosswalk = False  

# range에만 사용될 black, white 값
BLACK = 6
WHITE = 38  # 실제 측정 값이랑 비교 필요 42이었는데 더 낮게 설정하면 어떻게 될까?
THRESHOLD = (BLACK + WHITE) / 2   # ≈ 23
RANGE = max(1, WHITE - BLACK)
MARGIN = 5

right_ultra_sensor_detected = 0

# 색상 분류 함수 (RGB 기반)
def get_color_rgb(sensor):
    r, g, b = sensor.rgb()
    
    total = r + g + b
    refl = sensor.reflection()
    if total == 0:
        return "BLACK"  # 

    r_ratio, g_ratio, b_ratio = r / total, g / total, b / total

    # BLACK 밝기/반사도 낮으면 검정
    #ref 8 10
    #RGB (10-11, 17-18, 24-25)  (9-10, 12-13, 7-8)
    if refl < 15 and total < 60:
        return "BLACK"

    # BLUE (b가 압도적으로 크고 g/r은 작아야)
    # ref 9 12
    # RGB (11-12, 44-45, 100)  (11, 30-31, 43)
    # L: 8 UNKNOWN (13, 23, 16) | R: 12 UNKNOWN (11, 16, 1)
    # L: 14 UNKNOWN (11, 22, 16) | R: 12 BLACK (11, 16, 2)
    # bluee 10 36 100 | 10 27 49
    # blue  10 35 100 | 10 27 50    10 37 83  10 28 43   10 27 47
    # 18 30 44 <- was actually green?? whey
    # L: 42 UNKNOWN (55, 84, 100) | R: 11 UNKNOWN (10, 15, 5)
    # L: 10 UNKNOWN (9, 19, 17) | R: 11 BLACK (10, 15, 4)
    elif (
        b_ratio > 0.60
        and g_ratio < 0.28
        and r_ratio < 0.25
        and refl < 18
        and total < 120
    ):
        return "BLUE"

    # YELLOW (r과 g가 높고 b는 낮음, 반사도 중간 이상)
    # ref  52 51
    # rgb  (67, 84, 40)  (48, 51, 14)
    elif r_ratio > 0.35 and g_ratio > 0.35 and b_ratio < 0.25 and refl > 40:
        return "YELLOW"

    # WHITE (총합 높고 r,g,b 비슷)
    # ref  50 53
    # rgb  (65, 100, 100)  (50, 64, 71)
    elif total > 180 and abs(r_ratio - g_ratio) < 0.12 and abs(g_ratio - b_ratio) < 0.12:
        return "WHITE"

    # GREEN (g가 확실히 압도적일 때만)
    # ref  35 36
    # rgb  (45-46, 83, 93)  (34, 55, 37-38)
    elif g_ratio > 0.35 and refl > 30:
        return "GREEN"

    else:
        return "UNKNOWN"

def get_color_rgb_right(sensor, offset=(0, 0, 0)):
    r, g, b = sensor.rgb()
    refl = sensor.reflection()

    # Apply offset
    r += offset[0]
    g += offset[1]
    b += offset[2]

    total = r + g + b
    if total == 0:
        return "BLACK"

    r_ratio, g_ratio, b_ratio = r / total, g / total, b / total

    # --- BLACK ---
    # R_ref ≈10, RGB ≈(9–10, 13–14, 5–6)
    if refl < 15 and total < 40:
        return "BLACK"

    # --- BLUE ---
    # R_ref ≈11, RGB ≈(9–10, 28–29, 44–45)
    elif b_ratio > 0.45 and g_ratio > 0.3 and r_ratio < 0.25 and refl < 20:
        return "BLUE"

    # --- YELLOW ---
    # R_ref ≈56, RGB ≈(53, 55, 13–14)
    elif r_ratio > 0.38 and g_ratio > 0.38 and b_ratio < 0.2 and refl > 45:
        return "YELLOW"

    # --- WHITE ---
    # R_ref ≈55, RGB ≈(52, 65, 70)
    elif total > 160 and abs(r_ratio - g_ratio) < 0.1 and abs(g_ratio - b_ratio) < 0.1 and refl > 45:
        return "WHITE"

    # --- GREEN ---
    # R_ref ≈38, RGB ≈(36, 58, 36)
    elif g_ratio > 0.4 and g > r and g > b and refl > 25:
        return "GREEN"

    else:
        return "UNKNOWN"


def stable_color(sensor, target, stable_count=5):
    for _ in range(stable_count):
        if get_color_rgb(sensor) != target:
            return False
        wait(20)
    return True

# PID 보조 함수
def pid_control(error, last_err, integral):
    # Normalize error by range
    norm_error = error / RANGE
    integral = max(-I_CLAMP, min(I_CLAMP, integral + norm_error))
    deriv = norm_error - last_err
    turn = Kp * norm_error + Ki * integral + Kd * deriv
    return turn, integral

def avoid_obstacle():
    global right_lane, integral, last_err
    print("Avoiding obstacle smoothly... Right lane: " + str(right_lane))

    direction = -1 if not right_lane else 1  # 왼쪽 차선이면 오른쪽으로, 오른쪽이면 왼쪽으로
    MAX_TURN = 90
    STEP = 20
    
    l_white_count = 0
    r_white_count = 0
    l_green_count = 0
    r_green_count = 0

    # --- Phase 1: Turn out (0 → +MAX) ---
    TURN_DURATION = 700
    for t in range(0, TURN_DURATION, STEP):
        l_col = get_color_rgb(L)
        r_col = get_color_rgb_right(R)
            
        # check if both sensors have crossed white midline
        if l_col == "WHITE":
            l_white_count += 1
        if r_col == "WHITE":
            r_white_count += 1  
        if l_white_count >= 2 and r_white_count >= 2:
            print("Both sensors detected WHITE midline → lane change complete")
            break
    
        if l_col == "GREEN":
            l_green_count += 1
        if r_col == "GREEN":
            r_green_count += 1
        if right_lane:
            if l_green_count >= 2:
                print("GREEN on left sensor during turn → lane change complete")
                break
        else:
            if r_green_count >= 2:
                print("GREEN on right sensor during turn → lane change complete")
                break
        
        phase = t / TURN_DURATION          # 0 -> 1
        turn = direction * phase * MAX_TURN
        speed = BASE_SPEED
        left_motor.run(speed - turn)
        right_motor.run(speed + turn)
        print("Turning out: L speed:", int(speed - turn), "R speed:", int(speed + turn))
        wait(STEP)
        
    # debug line
    print("COUNTER TURNING PHASE")
    # --- Phase 2: Counter turn (0 → -MAX/2) ---
    COUNTER_DURATION = 700
    L_green_counter = 0
    R_green_counter = 0
    for t in range(0, COUNTER_DURATION, STEP):
        if l_col != "BLACK":
            L_green_counter += 1
        if r_col != "BLACK":
            R_green_counter += 1
        if t > COUNTER_DURATION//2 and L_green_counter >= 2 or R_green_counter >= 2:
            print("Both sensors detected GREEN → overshoot")  # overshoot
            break
        phase = t / COUNTER_DURATION       # 0 -> 1
        turn = -direction * (MAX_TURN * 1.5) * phase
        speed = BASE_SPEED
        left_motor.run(speed - turn)
        right_motor.run(speed + turn)
        print("Counter turning: L speed:", int(speed - turn), "R speed:", int(speed + turn))
        wait(STEP)

    # 상태 초기화
    right_lane = not right_lane
    integral = last_err = 0
    print("Obstacle avoided → now on", "RIGHT" if right_lane else "LEFT")

right_lane = True  # 왼 차선에서 시작, 임시로 설정
obstacle_detected = True 

def park():
    print("Parking maneuver started...")
    turn = 180
    speed = BASE_SPEED // 2
    backtrack_time = 4000 
    step = 20
    elapsed = 0
    
    backtrack_limit = 150 
    second_backtrack_limit = 100  
    forward_limit = 150 
    second_backtrack_front_limit = 250 
    
    # 일단 후진 4초
    while elapsed < backtrack_time:
        left_motor.run((speed) * -1)
        right_motor.run((speed) * -1)
        elapsed += step
        print("Distance to obstacle:", right_ultra_sensor.distance() // 10, "cm")

        wait(step)
        
    # 벽까지 15 cm 남을 때까지 스무스하게 턴하면서 후진
    print("벽까지 15cm 남을 때까지 스무스하게 턴 후진")
    TURN_DURATION = 10000  # 총 회전 시간 (ms 단위)
    STEP = 20
    for t in range(0, TURN_DURATION, STEP):
        distance = right_ultra_sensor.distance()
        if distance <= backtrack_limit:  # 20cm 이내로 가까워지면 중단
            print(backtrack_limit, "cm 도달 → 턴 종료")
            break
        
        phase = t / TURN_DURATION   # 0 → 1
        turn = (1 - phase) * 80     # 처음엔 크게 돌고 점점 작게
        left_motor.run((-speed - turn))
        right_motor.run((-speed + turn))
        
        print("Distance:", distance // 10, "cm", "Turn:", int(turn))
        wait(STEP)

    # 이후 정렬용 직진 후진 단계로 연결
    # 벽까지 10 cm 남을 때까지 반대 방향으로 살짝 꺾어서 후진 (정렬 단계)
    print("벽까지 10cm 남을 때까지 반대 방향으로 살짝 꺾어서 후진")
    TURN_CORRECT = 40  # 조정 강도 (필요시 조절)
    while right_ultra_sensor.distance() > second_backtrack_limit and ultra_sensor.distance() > second_backtrack_front_limit:
        left_motor.run((-speed + TURN_CORRECT))
        right_motor.run((-speed - TURN_CORRECT))
        print("Distance:", right_ultra_sensor.distance() // 10, "cm")
        wait(20)
        
    # 앞으로 전진
    print("앞으로 전진하여 주차 마무리")
    while ultra_sensor.distance() > forward_limit:
        left_motor.run(speed)
        right_motor.run(speed)
        wait(20)
        
    print("Stopped, Distance to obstacle:", right_ultra_sensor.distance() // 10, "cm")
    # 정지
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    print("Parking maneuver completed.")
    wait(500)


# 버튼 누르면 시작
ev3.screen.clear()
ev3.screen.print("Press center button")
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)


# 오른쪽 왼쪽 센서 반사도 차이 보정
ev3.screen.clear()
ev3.screen.print("Calibrating sensors...")
wait(1000)  # 잠시 대기 후 측정
left_reflection = L.reflection()
right_reflection = R.reflection()
base_error = left_reflection - right_reflection
print("Calibration offset (base_error):", base_error)
if abs(base_error) > 10:
    base_error = -2
ev3.screen.clear()
ev3.screen.print("Calib offset:", base_error)
wait(1000)


print("Right sensor color offset:", color_offset)
wait(1000)

# ---------------------
# 메인 시작
# ---------------------
ev3.screen.clear()
ev3.screen.print("Starting...")

integral = 0
last_err = 0

base_speed = BASE_SPEED

# 차선 변경 함수 (장애물 감지시 호출)

# 색상 이벤트 안정성 카운터 초기화
# 각 센서별로 BLUE, RED, YELLOW 카운터를 저장하는 딕셔너리
color_counters = {
    'L': {'BLUE': 0, 'RED': 0, 'YELLOW': 0, 'BLACK': 0},
    'R': {'BLUE': 0, 'RED': 0, 'YELLOW': 0, 'BLACK': 0},
}

# 색상이 인식되기 위한 최소 안정화 시간 (반복 횟수)
STABLE_THRESHOLD = 5 # 5*20ms = 100ms

def update_color_counters():
    # 현재 각 센서 색상 읽기
    l_color = get_color_rgb(L)
    r_color = get_color_rgb_right(R)
    # 업데이트 함수: 각 색상별로 감지되면 카운터 증가, 아니면 0으로 초기화
    for sensor_label, color in [('L', l_color), ('R', r_color)]:
        for c in ['BLUE', 'RED', 'YELLOW', 'BLACK', 'GREEN', 'WHITE']:
            if color == c:
                color_counters[sensor_label][c] += 1
            else:
                color_counters[sensor_label][c] = 0

# PID용 반사도 보정 함수
def get_pid_reflection(sensor):
    color = get_color_rgb(sensor)
    refl = sensor.reflection()
    if color == "GREEN":
        return refl - 15   # 도로 밖은 더 어둡게 보정
    elif color == "WHITE":
        return refl + 10   # 중앙선은 더 밝게 보정
    else:
        return refl


def is_stably_detected(color):
    # 두 센서 중 하나라도 해당 색상이 안정적으로 감지되었는지 확인
    return (color_counters['L'][color] >= STABLE_THRESHOLD) or (color_counters['R'][color] >= STABLE_THRESHOLD)

obstacle_detected_count = 0

# 메인 루프
while True:
    # lv = L.reflection()
    # rv = R.reflection()
    lv = get_pid_reflection(L)
    rv = get_pid_reflection(R)
    lc = get_color_rgb(L)
    rc = get_color_rgb_right(R)
    
    # 색상 이벤트 안정성 카운터 업데이트
    update_color_counters()
    
    # debug line
    # 디버깅 출력 (반사도 + 컬러)
    print("L:", lv, lc, L.rgb(), "| R:", rv, rc, R.rgb())
    
    # --- 주차장 인식 ---   
    now = time.ticks_ms()  # 현재 시간(ms)
    distance = right_ultra_sensor.distance()
    # debug line
    print("Right ultra sensor distance (cm):", distance // 10)

    if distance < 300:
        # 쿨타임 체크
        if now - last_detection_time > COOLDOWN_MS:
            right_ultra_sensor_detected += 1
            last_detection_time = now
            # debug line
            print("Obstacle on right detected:", right_ultra_sensor_detected)

    # 두 번째 감지 시 주차장 인식
    if right_ultra_sensor_detected >= 2:
        print("주차장 인식")
        left_motor.stop(Stop.BRAKE)
        right_motor.stop(Stop.BRAKE)
        park()
        break

    # --- Initial lane detection using WHITE or GREEN before RED ---
    if right_lane is None:
        # 오른쪽 차선 감지: 왼쪽 센서가 WHITE 안정적이거나 오른쪽 센서가 GREEN 안정적이면
        if color_counters['L']['WHITE'] >= 2 or color_counters['R']['GREEN'] >= 2:
            right_lane = True
            print("Detected initial lane: RIGHT")
        # 왼쪽 차선 감지: 오른쪽 센서가 WHITE 안정적이거나 왼쪽 센서가 GREEN 안정적이면
        elif color_counters['R']['WHITE'] >= 2 or color_counters['L']['GREEN'] >= 2:
            right_lane = False
            print("Detected initial lane: LEFT")
    
    # --- BLUE: 정지 후 출발 ---
    if lc == "BLUE" or rc == "BLUE":
        left_motor.stop()
        right_motor.stop()
        print("BLUE detected → Stop 3s")
        wait(3000)
        integral = 0
        last_err = 0
        # 파란색 끝날 때까지 앞으로 조금 진행
        while get_color_rgb(L) == "BLUE" or get_color_rgb_right(R) == "BLUE":
            left_motor.run(BASE_SPEED)
            right_motor.run(BASE_SPEED)
            wait(20)
        # 카운터 초기화
        color_counters['L']['BLUE'] = 0
        color_counters['R']['BLUE'] = 0
        continue

    # --- YELLOW: crosswalk start ---
    if lc == "YELLOW" or rc == "YELLOW":
        print("YELLOW detected → prepare crosswalk")

        max_crosswalk_time_ms = 2000   
        step = 20                      
        elapsed = 0
        L_black_counter = 0
        R_black_counter = 0
        BLACK_THRESHOLD = 5  
        CROSS_SPEED = BASE_SPEED // 2

        while True:
            
            # if parking lot detected during crosswalk, break
            distance = right_ultra_sensor.distance()
            print("Right ultra sensor distance (cm) during crosswalk:", distance // 10)
            if distance < 180:
                print("Parking lot detected during crosswalk → abort crosswalk")
                # 쿨타임 체크
                if now - last_detection_time > COOLDOWN_MS:
                    right_ultra_sensor_detected += 1
                    last_detection_time = now
                    # debug line
                    print("Obstacle on right detected:", right_ultra_sensor_detected)
                break
            
            lc = get_color_rgb(L)
            rc = get_color_rgb_right(R)
            print("In YELLOW zone:", lc, rc)

            # --- move straight only ---
            left_motor.run(CROSS_SPEED)
            right_motor.run(CROSS_SPEED)

            # --- update black detection counters ---
            L_black_counter = L_black_counter + 1 if lc == "BLACK" else 0
            R_black_counter = R_black_counter + 1 if rc == "BLACK" else 0
            elapsed += step

            # --- exit condition ---
            if (L_black_counter >= BLACK_THRESHOLD and 
                R_black_counter >= BLACK_THRESHOLD):
                print("Both sensors stable BLACK → exit crosswalk")
                break
            if elapsed >= max_crosswalk_time_ms:
                print("Max crosswalk time reached → forced exit")
                break

            wait(step)

        print("Crosswalk exited")
        
        # PID reset
        integral = 0
        last_err = 0
        in_crosswalk = False
        base_speed = BASE_SPEED
        
    # --- 장애물 감지 ---
    # 잘못 인식할 수 있으니 두번 연속 감지 시 회피 동작
    if not obstacle_detected:
        if ultra_sensor.distance() < 390:
            obstacle_detected_count += 1
        
        if obstacle_detected_count >= 2:
            # debug line
            print("Obstacle detected, distance:", ultra_sensor.distance() / 10, "cm")
            avoid_obstacle()
            obstacle_detected_count = 0
            obstacle_detected = True
            continue  # 장애물 회피 중에는 나머지 PID 루프 잠시 중단

    # --- 도로 주행 ---
    l_col = get_color_rgb(L)
    r_col = get_color_rgb_right(R)

    # 양쪽 센서가 블랙이면 PID 사용 정상 주행 
    if not in_crosswalk:
        if l_col == "BLACK" and r_col == "BLACK":
            # 정상 주행 → PID
            error = (lv - rv) - base_error
            turn, integral = pid_control(error, last_err, integral)
            err_norm = min(1.0, abs(error) / RANGE)
            base = max(BASE_SPEED // 2, base_speed * (1.0 - 0.5 * err_norm))
            left_cmd  = max(-MAX_SPEED, min(MAX_SPEED, base - turn))
            right_cmd = max(-MAX_SPEED, min(MAX_SPEED, base + turn))
            left_motor.run(left_cmd)
            right_motor.run(right_cmd)
            last_err = error
            
        # 빨간색 무시하고 직진 
        elif l_col == "RED" or r_col == "RED":
            print("Both sensors RED → going straight")
            left_motor.brake()
            right_motor.brake()
            left_motor.run(BASE_SPEED)
            right_motor.run(BASE_SPEED)
        
        # FALLBACK - 한쪽 센서가 BLACK이 아니면 그쪽으로 회전
        elif l_col != "BLACK" and l_col != "RED":
            print("FALLBACK: Left sensor not BLACK, turning right")
            # 왼쪽 센서가 BLACK이 아니면 오른쪽으로 회전 
            left_motor.run(BASE_SPEED + 30)
            right_motor.run(BASE_SPEED // 2)
        elif r_col != "BLACK" and l_col != "RED":
            print("FALLBACK: Right sensor not BLACK, turning left")
            # 오른쪽 센서가 BLACK이 아니면 왼쪽으로 회전
            left_motor.run(BASE_SPEED // 2)
            right_motor.run(BASE_SPEED + 30)
            
        # else:
        #     # 양쪽 센서가 BLACK도 RED도 아니면 일단 직진
        #     print("FALLBACK: Both sensors not BLACK/RED, going straight")
        #     left_motor.run(BASE_SPEED)
        #     right_motor.run(BASE_SPEED)
            
        
    
    

    wait(20)
    continue

