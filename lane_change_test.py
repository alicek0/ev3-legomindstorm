#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait

# --- 기본 설정 ---
ev3 = EV3Brick()
left_motor  = Motor(Port.B)
right_motor = Motor(Port.C)

BASE_SPEED = 280

# 왼차선 기준 → 오른쪽으로 피하고 돌아오기
right_lane = False  # 현재 차선 방향 (왼쪽이면 False, 오른쪽이면 True)
integral = 0
last_err = 0


def avoid_obstacle():
    global right_lane, integral, last_err
    print("Avoiding obstacle smoothly...")
    print("Current lane:", "RIGHT" if right_lane else "LEFT")
    
    direction = -1 if right_lane == False else 1  # 왼쪽 차선이면 오른쪽으로, 반대도 동일
    
    # --- 부드럽게 회전 ---
    max_turn_used = 0
    step = 20           # ms 단위 루프 간격
    duration = 1000     # 회전 구간 (1초)
    for t in range(0, duration, step):
        speed = BASE_SPEED - BASE_SPEED//2 * (t/duration)  # 서서히 감속
        phase = t / duration
        bias = direction * (1 - abs(phase * 2 - 1))
        turn = bias * 120
        max_turn_used = max(max_turn_used, abs(turn))
        left_motor.run(speed - turn)
        right_motor.run(speed + turn)
        wait(step)

    # --- 복원 구간 ---
    end_duration = duration * 0.3
    max_turn_used = max_turn_used * 0.3
    for t in range(0, int(end_duration), step):
        phase = t / end_duration
        bias = -direction * (1 - abs(phase * 2 - 1))
        turn = bias * max_turn_used  
        left_motor.run(BASE_SPEED - turn)
        right_motor.run(BASE_SPEED + turn)
        wait(step)

    # 정지 및 상태 갱신
    left_motor.stop()
    right_motor.stop()
    right_lane = not right_lane
    integral = 0
    last_err = 0
    print("Obstacle avoided → back to", "RIGHT" if right_lane else "LEFT")


# --- 실행 ---
avoid_obstacle()