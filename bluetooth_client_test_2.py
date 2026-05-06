#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.messaging import BluetoothMailboxServer, TextMailbox

# --- 1. 장치 설정 ---
ev3 = EV3Brick()

# --- 2. 블루투스 서버 설정 ---
# [중요] 리더는 'Server'입니다.
server = BluetoothMailboxServer()

# [중요] 클라이언트와 '똑같은 이름'의 메일박스를 사용해야 합니다.
# 클라이언트 코드에서 'CMD'를 썼다면 여기서도 'CMD'여야 합니다.
mbox = TextMailbox('CMD', server)

print("블루투스 서버 시작...")
ev3.screen.print("Waiting for Client...")

# 클라이언트가 연결할 때까지 기다림
server.wait_for_connection()
print("클라이언트 연결됨!")

ev3.screen.clear()
ev3.screen.print("Connected!")
ev3.speaker.beep()

# ---------------------
# 메인 루프 (양방향 통신 테스트)
# ---------------------
ev3.screen.print("Ready to Chat")
print("테스트 시작: 가운데 버튼을 눌러보세요.")

# 중복 출력 방지를 위한 변수
last_received_msg = None 

while True:
    # --- 1. 수신 확인 (Client -> Server) ---
    # wait_new() 대신 read()를 사용하여 멈추지 않고 확인합니다.
    current_msg = mbox.read()
    
    # 메시지가 있고, 이전과 다를 때만 출력
    if current_msg is not None and current_msg != last_received_msg:
        # [수정] f-string 대신 .format() 사용
        print("[수신] Client: {}".format(current_msg))
        ev3.screen.print("RX: {}".format(current_msg))
        last_received_msg = current_msg
        
        # 소리 알림
        ev3.speaker.beep()

    # --- 2. 송신 확인 (Server -> Client) ---
    # 가운데 버튼을 누르면 메시지 전송
    if Button.CENTER in ev3.buttons.pressed():
        msg_to_send = "SERVER_BTN_CLICK"
        
        try:
            mbox.send(msg_to_send) # 클라이언트로 전송
            # [수정] f-string 대신 .format() 사용
            print("[송신] Me: {}".format(msg_to_send))
            ev3.screen.print("TX: {}".format(msg_to_send))
            ev3.speaker.beep(500, 100) # 전송 확인음 (톤 높게)
        except OSError as e:
            print("전송 실패: {}".format(e))

        # 버튼을 뗄 때까지 대기 (중복 전송 방지)
        while Button.CENTER in ev3.buttons.pressed():
            wait(10)

    # 루프 과부하 방지
    wait(20)