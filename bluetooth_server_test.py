#!/usr/bin/env pybricks-micropython
from pybricks.messaging import BluetoothMailboxServer, TextMailbox
from pybricks.parameters import Port
from pybricks.ev3devices import Motor
from pybricks.tools import wait

# 서버 객체 생성
server = BluetoothMailboxServer()

# 'greeting'이라는 이름의 텍스트 메일박스 생성
# 클라이언트와 이 이름이 일치해야 합니다.
mbox = TextMailbox('greeting', server)

# 예시: 모터 연결 (옵션)
# m = Motor(Port.A)

print("Waiting for connection...")
# 클라이언트가 연결될 때까지 대기
server.wait_for_connection()
print("Connected!")

# 메시지를 기다리는 루프
while True:
    # 'greeting' 메일박스에 메시지가 도착할 때까지 대기
    mbox.wait()

    # 메시지 읽기
    received_message = mbox.read()
    print("Received:", received_message)

    # 받은 메시지가 'Hello'이면
    if received_message == 'Hello':
        # 모터를 돌리거나 (옵션)
        # m.run_angle(500, 90)
        
        # 클라이언트에게 응답 보내기
        mbox.send('Ack')
        print("Sent: Ack")

    # 받은 메시지가 'Quit'이면 루프 종료
    if received_message == 'Quit':
        print("Received Quit. Shutting down.")
        break
    
    # 잠시 대기
    wait(100)