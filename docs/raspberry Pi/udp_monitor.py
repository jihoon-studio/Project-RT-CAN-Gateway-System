import socket
import struct
import sqlite3
import datetime
import threading
import time
import os

# 1. 설정 (내 PC의 포트)
UDP_IP = "0.0.0.0" # 모든 IP에서 오는 데이터를 다 받음
UDP_PORT = 8080

latest_data = {
    "distance": None,
    "angle": None,
    "temp": None,
    "humi": None,
    "led": None
}

DB_NAME = "sensor_data.db"
print("DB FILE:", os.path.abspath(DB_NAME))

# --- 데이터베이스 초기화 함수 ---
def init_db():
    conn = sqlite3.connect(DB_NAME)
    cursor = conn.cursor()
    # 테이블 생성 (없으면 만듦)
    # id: 번호, timestamp: 시간, sensor_type: 센서이름, val1, val2, val3: 측정값
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS sensor_snapshot (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,

            -- Board B
            distance_cm REAL,
            motor_angle_deg REAL,

            -- Board A / 기타
            temperature REAL,
            humidity REAL,
            led INTEGER
        );
    ''')
    conn.commit()
    conn.close()
    print(f"📂 데이터베이스 준비 완료: {DB_NAME}")


def save_snapshot():
    conn = sqlite3.connect(DB_NAME)
    cursor = conn.cursor()

    cursor.execute("""
        INSERT INTO sensor_snapshot (
            distance_cm,
            motor_angle_deg,
            temperature,
            humidity,
            led
        ) VALUES (?, ?, ?, ?, ?)
    """, (
        latest_data["distance"],
        latest_data["angle"],
        latest_data["temp"],
        latest_data["humi"],
        latest_data["led"]
    ))

    conn.commit()
    conn.close()

def snapshot_loop():
    while True:
        save_snapshot()
        time.sleep(1)

# 2. 프로그램 시작
init_db() # DB 생성
threading.Thread(target=snapshot_loop, daemon=True).start()




# 2. 소켓 생성 (UDP)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


try:
    sock.bind((UDP_IP, UDP_PORT))
    print(f"📡 UDP 서버 시작! Port: {UDP_PORT}에서 수신 대기 중...")
except Exception as e:
    print(f"❌ 소켓 바인딩 실패: {e}")
    print("혹시 Hercules나 다른 프로그램이 8080 포트를 쓰고 있나요?")
    exit()

while True:
    try:
        # 3. 데이터 수신 (최대 1024바이트)
        data, addr = sock.recvfrom(1024)

        # 4. 패킷 길이 확인 (1 + 2 + 1 + 8 = 12 bytes)
        if len(data) != 12:
            print(f"⚠️ 잘못된 길이의 패킷 수신: {len(data)} bytes")
            continue

        # 5. 구조체 언패킹 (STM32의 __packed 구조체와 동일하게 맞춤)
        # <: 리틀 엔디안 (STM32 기본)
        # B: Start Byte (uint8)
        # H: CAN ID (uint16)
        # B: DLC (uint8)
        # 8B: Data (uint8 * 8)
        header_fmt = '<BHB8B'
        unpacked_data = struct.unpack(header_fmt, data)

        start_byte = unpacked_data[0]
        can_id     = unpacked_data[1]
        dlc        = unpacked_data[2]
        payload    = unpacked_data[3:] # 튜플 형태 (d0, d1, ... d7)

        # 시작 바이트 확인 (0xAA)
        if start_byte != 0xAA:
            print(f"⚠️ 시작 바이트 오류: {hex(start_byte)}")
            continue

        # 6. ID별 데이터 해석 및 출력
        # 디버깅용: 원본 헥사값 출력 (무슨 값이 오는지 눈으로 확인 가능)
        raw_hex = " ".join([f"{x:02X}" for x in payload])
        can_id_hex = f"0x{can_id:03x}"

        print(f"[{addr[0]}] ID: 0x{can_id:03X} | ", end="")

        if can_id == 0x101:
            # ★ 통합 데이터 (Board B/D에서 묶어 보낸 것)
            latest_data["temp"] = payload[0]
            latest_data["humi"] = payload[1]
            latest_data["led"]  = payload[2]
            print(f"📦 [통합] 온도: {temp}°C, 습도: {humi}%, LED : {led}")

        elif can_id == 0x201:
            distance    = (payload[0] << 8) | payload[1]
            latest_data["distance"] = distance
            print(f"🌊 [초음파]거리: {distance} cm")

        elif can_id == 0x202:
            angle_x10 = (payload[0] << 8) | payload[1]
            angle = angle_x10 / 10.0
            latest_data["angle"] = angle
            print(f"🌊 [모터]ANGLE: {angle} deg ")
        else:
            print(f"❓ [기 타] Data: {payload}")

    except KeyboardInterrupt:
        print("\n종료합니다.")
        break
    except Exception as e:
        print(f"에러 발생: {e}")
