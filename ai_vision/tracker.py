import serial
import math
import time
import re 

# 통신 포트 설정 (순서가 바뀌어서 로봇이 반대로 돌면, 여기 0과 1만 서로 바꿔주면 됩니다)
PORT_ANCHOR1 = '/dev/ttyACM0' # 앵커 1번 (왼쪽)
PORT_ANCHOR2 = '/dev/ttyACM1' # 앵커 2번 (오른쪽)
BAUD_RATE = 115200
L = 0.5 # 앵커 1번과 2번 사이의 실제 설치 거리 (50cm = 0.5m)

try:
    ser1 = serial.Serial(PORT_ANCHOR1, BAUD_RATE, timeout=0.1)
    ser2 = serial.Serial(PORT_ANCHOR2, BAUD_RATE, timeout=0.1)
    print("✅ 젯슨 뇌 가동 준비 완료! 앵커와 연결되었습니다.")
except Exception as e:
    print(f"❌ 포트 연결 실패: {e}\n(USB가 꽉 꽂혔는지 확인하세요!)")
    exit()

def get_distance(serial_port):
    if serial_port.in_waiting > 0:
        try:
            line = serial_port.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return None
            
            # 앵커가 보내는 "D: 1.25" 같은 데이터에서 무조건 숫자만 뽑아내는 강력한 로직
            match = re.search(r'\d+\.\d+', line)
            if match:
                return float(match.group())
        except:
            pass
    return None

print("--- 🤖 사용자 방향 추적 시스템 가동 ---")

d1, d2 = None, None

while True:
    # 0.1초마다 앵커 1, 2에서 거리를 훔쳐옵니다.
    new_d1 = get_distance(ser1)
    new_d2 = get_distance(ser2)
    
    if new_d1 is not None: d1 = new_d1
    if new_d2 is not None: d2 = new_d2

    # 두 앵커의 값이 모두 확보되었을 때만 계산 시작
    if d1 is not None and d2 is not None:
        
        # 삼각함수로 사용자의 좌우(X) 위치 계산
        x = (d1**2 - d2**2) / (2 * L)

        # 사용자의 전방(Y) 거리 계산 
        y_sq = d1**2 - x**2
        y = math.sqrt(y_sq) if y_sq > 0 else 0

        print(f"📡 측정 거리 -> [왼쪽 앵커]: {d1:.2f}m | [오른쪽 앵커]: {d2:.2f}m")
        print(f"🎯 타겟 위치 -> 좌우 편차(X): {x:.2f}m | 남은 거리(Y): {y:.2f}m")

        # 로봇 모터 조향 지시 (데드존 15cm 적용)
        if x > 0.15:
            print("▶️ 우회전 지시! (사용자가 오른쪽에 있음)\n")
        elif x < -0.15:
            print("◀️ 좌회전 지시! (사용자가 왼쪽에 있음)\n")
        else:
            if y > 0.6: 
                print("⬆️ 직진 지시! (사용자 추적 중)\n")
            else:       
                print("⏹️ 정지! (목표 거리 도달, 충돌 방지)\n")
        
        time.sleep(0.1) 
        d1, d2 = None, None # 계산 후 데이터 초기화 (꼬임 방지)