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

max_obstacle_distance = 30  # cm

BASE_SPEED = 280
MAX_SPEED  = 400

Kp, Ki, Kd = 1.0, 0.0, 1.0
I_CLAMP = 200

in_crosswalk = False  

# range에만 사용될 black, white 값
BLACK = 6
WHITE = 41
THRESHOLD = (BLACK + WHITE) / 2   # ≈ 23
RANGE = max(1, WHITE - BLACK)
MARGIN = 5

right_ultra_sensor_detected = 0

# 색상 분류 함수 (RGB 기반)
def get_color_rgb(sensor, offset=(0,0,0)):
    r, g, b = sensor.rgb()
    r += offset[0]; g += offset[1]; b += offset[2]
    total = r + g + b
    if total == 0:
        return "BLACK"
    r_ratio, g_ratio, b_ratio = r / total, g / total, b / total
    refl = sensor.reflection()

    if refl < 15 and total < 60:
        return "BLACK"
    elif b_ratio > 0.5 and g_ratio < 0.3 and refl < 30:
        return "BLUE"
    elif r_ratio > 0.35 and g_ratio > 0.35 and b_ratio < 0.25 and refl > 40:
        return "YELLOW"
    elif total > 180 and abs(r_ratio - g_ratio) < 0.12 and abs(g_ratio - b_ratio) < 0.12:
        return "WHITE"
    elif g_ratio > 0.35 and refl > 30:
        return "GREEN"
    else:
        return "UNKNOWN"

# --- NEW: map RGB to continuous lane value for PID ---
def lane_value_from_rgb(r, g, b):
    """
    Map sensor RGB to continuous 0.0 (black) → 1.0 (green) scale.
    Assumes black road is dark and green edges are high G.
    """
    total = r + g + b
    if total == 0:
        return 0.0
    g_ratio = g / total
    # Adjust scale to 0..1: black ~0.0, green ~1.0
    # You can tweak min/max thresholds if needed
    min_g, max_g = 0.1, 0.6  # empirical range for black→green
    val = (g_ratio - min_g) / (max_g - min_g)
    return max(0.0, min(1.0, val))

def stable_color(sensor, target, stable_count=5):
    for _ in range(stable_count):
        if get_color_rgb(sensor) != target:
            return False
        wait(20)
    return True

# PID 보조 함수
def pid_control(error, last_err, integral):
    norm_error = error  # already normalized 0..1 difference
    integral = max(-I_CLAMP, min(I_CLAMP, integral + norm_error))
    deriv = norm_error - last_err
    turn = Kp * norm_error + Ki * integral + Kd * deriv
    return turn, integral

# 장애물 회피 함수
def avoid_obstacle():
    global right_lane, integral, last_err
    print("Avoiding obstacle smoothly...")
    direction = 1 if right_lane == False else -1  # 왼쪽 차선이면 오른쪽으로, 반대도 동일
    max_turn_used = 0
    step = 20
    end_duration = duration = 1000
    for t in range(0, duration, step):
        speed = BASE_SPEED - BASE_SPEED//2 * (t/duration)
        phase = t / duration
        bias = direction * (1 - abs(phase * 2 - 1))
        turn = bias * 120
        max_turn_used = max(max_turn_used, abs(turn))
        left_motor.run(speed - turn)
        right_motor.run(speed + turn)
        wait(step)
        l_col = get_color_rgb(L)
        r_col = get_color_rgb(R)
        if l_col == "GREEN" or r_col == "GREEN":
            end_duration = max(t, step)
            print("Lane change detected GREEN → complete")
            break
    end_duration *= 0.3
    max_turn_used *= 0.3
    for t in range(0, int(end_duration), step):
        phase = t / end_duration
        bias = -direction * (1 - abs(phase * 2 - 1))
        turn = bias * max_turn_used  
        left_motor.run(BASE_SPEED - turn)
        right_motor.run(BASE_SPEED + turn)
        wait(step)
        lc = get_color_rgb(L)
        rc = get_color_rgb(R)
        if lc == "BLACK" and rc == "BLACK":
            print("Lane change both BLACK → complete")
            break
    right_lane = not right_lane
    integral = 0
    last_err = 0
    print("Obstacle avoided → back to", "RIGHT" if right_lane else "LEFT")

# --- START ---
ev3.screen.clear()
ev3.screen.print("Press center button")
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)

# Calibration
ev3.screen.clear()
ev3.screen.print("Calibrating sensors...")
wait(1000)
left_reflection = L.reflection()
right_reflection = R.reflection()
base_error = left_reflection - right_reflection
if abs(base_error) > 10:
    base_error = -2

# RGB offset
l_r, l_g, l_b = L.rgb()
r_r, r_g, r_b = R.rgb()
color_offset = [(l_r - r_r),(l_g - r_g),(l_b - r_b)]
wait(500)

# Main loop
integral = 0
last_err = 0
right_lane = False
base_speed = BASE_SPEED

while True:
    # Read RGB and compute continuous lane value
    l_r, l_g, l_b = L.rgb()
    r_r, r_g, r_b = R.rgb()
    l_val = lane_value_from_rgb(l_r, l_g, l_b)
    r_val = lane_value_from_rgb(r_r, r_g, r_b)
    
    # PID error using continuous RGB values
    error = l_val - r_val
    turn, integral = pid_control(error, last_err, integral)
    last_err = error
    
    # Motor command
    left_cmd  = max(-MAX_SPEED, min(MAX_SPEED, base_speed - turn * MAX_SPEED))
    right_cmd = max(-MAX_SPEED, min(MAX_SPEED, base_speed + turn * MAX_SPEED))
    left_motor.run(left_cmd)
    right_motor.run(right_cmd)
    
    # Obstacle detection (unchanged)
    if ultra_sensor.distance() < 300:
        print("Obstacle detected")
        avoid_obstacle()
        continue
    
    # Stop if right-side parking detected
    if right_ultra_sensor.distance() < 50:
        right_ultra_sensor_detected += 1
        if right_ultra_sensor_detected > 2:
            left_motor.stop(Stop.BRAKE)
            right_motor.stop(Stop.BRAKE)
            break
    
    wait(20)