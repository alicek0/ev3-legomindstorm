#!/usr/bin/env pybricks-micropython
# Print left sensor and right sensor values (reflection + RGB)
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait

ev3 = EV3Brick()
left_motor  = Motor(Port.B)
right_motor = Motor(Port.C)
L = ColorSensor(Port.S1)   
R = ColorSensor(Port.S4)   

# 색상 분류 함수 (RGB 기반)

def get_color_rgb(sensor, offset=(0, 0, 0)):
    r, g, b = sensor.rgb()
    
    # Apply offset
    r += offset[0]
    g += offset[1]
    b += offset[2]
    
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
    elif (r<=14 and g>=19 and g<= 50 and b >= 90 and refl < 11): return "BLUE"

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

# RGB 또는 reflection 값 기준으로 보정
l_r, l_g, l_b = L.rgb()
r_r, r_g, r_b = R.rgb()

# 오른쪽 센서 보정치 계산
color_offset = [(l_r - r_r),(l_g - r_g),(l_b - r_b)]

print("Right sensor color offset:", color_offset)
wait(1000)

while True:
    lv = L.reflection()
    rv = R.reflection()

    l_rgb = L.rgb()
    r_rgb = R.rgb()
    
    print("R:", rv, r_rgb, get_color_rgb_right(R),"L:", lv, l_rgb, get_color_rgb(L))

    wait(200)   # 0.2초 간격 출력
    
    
    
    
