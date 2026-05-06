#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Color
from pybricks.tools import wait

ev3 = EV3Brick()
left_motor  = Motor(Port.B)
right_motor = Motor(Port.C)
L = ColorSensor(Port.S1)   
R = ColorSensor(Port.S4)   

BASE_SPEED = 80   
MAX_SPEED  = 400

Kp, Ki, Kd = 1.0, 0.0, 1.0
I_CLAMP = 200

def sample_ref(sensor, n=20, delay=20):
    total = 0
    for _ in range(n):
        total += sensor.reflection()
        wait(delay)
    return total / n

ev3.screen.clear()
ev3.screen.print("Place L on BLACK, R on WHITE")
wait(2000)  

BLACK = sample_ref(L, n=30, delay=20)
WHITE = sample_ref(R, n=30, delay=20)

print("BLACK avg:", BLACK)
print("WHITE avg:", WHITE)

THRESHOLD = (BLACK + WHITE) / 2
RANGE = max(1, WHITE - BLACK)
MARGIN = 5

print("BLACK:", BLACK, "WHITE:", WHITE, "THRESHOLD:", THRESHOLD, "RANGE:", RANGE)


start = False
while not start:
    ev3.screen.clear()
    ev3.screen.print("Waiting BLACK...")
    lv = L.reflection()
    print("lv:", lv)
    if lv <= (BLACK + MARGIN):
        ev3.screen.clear()
        ev3.screen.print("Start!")
        start = True
        wait(500)
        break


integral = 0
last_err = 0

while start:
    lv = L.reflection()
    rv = R.reflection()
    print("lv:", lv, "rv:", rv)

    if L.color() == Color.RED or R.color() == Color.RED:
        left_motor.stop()
        right_motor.stop()
        print("Stopped@RED")
        break

    if L.color() == Color.GREEN:
        left_motor.stop()
        right_motor.stop()
        print("Wait@GREEN")
        wait(3000)
        # PID 상태 초기화
        last_err = error
        integral = 0
        print("Forward after GREEN")
        left_motor.run_time(150, 500, then=Stop.BRAKE, wait=True)
        right_motor.run_time(150, 500, then=Stop.BRAKE, wait=True)
        continue 

    error = THRESHOLD - lv

    # 검정이 확실할때 직진 
    if abs(lv - BLACK) <= MARGIN:
        left_motor.run(BASE_SPEED+70)
        right_motor.run(BASE_SPEED+70)
        last_err = 0
        integral = 0
        wait(20)
        continue

    integral = max(-I_CLAMP, min(I_CLAMP, integral + error))
    deriv = error - last_err
    turn = Kp * error + Ki * integral + Kd * deriv

    # 보조 센서 기반 보정
    if lv >= THRESHOLD + MARGIN:  # L이 흰색 쪽
        if rv < THRESHOLD:       
            turn -= 70
        else:                     
            left_motor.run(-BASE_SPEED)
            right_motor.run(BASE_SPEED)
            while L.reflection() > THRESHOLD:  
                wait(10)
            left_motor.stop()
            right_motor.stop()
            last_err = 0
            integral = 0
            continue

    # 에러 클때 속도 줄임
    err_norm = min(1.0, abs(error) / RANGE)
    base = BASE_SPEED * (1.0 - 0.8 * err_norm)   

    left_cmd  = max(-MAX_SPEED, min(MAX_SPEED, base - turn))
    right_cmd = max(-MAX_SPEED, min(MAX_SPEED, base + turn))

    left_motor.run(left_cmd)
    right_motor.run(right_cmd)

    last_err = error

    if lv <= 0 and rv <= 0:
        left_motor.stop()
        right_motor.stop()
        print("Line lost")
        break

    wait(20)