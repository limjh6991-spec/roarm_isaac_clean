# 🤖 RoArm-M3 실제 하드웨어 제어 가이드

## 📡 연결 방법 (3가지)

### 1️⃣ **WiFi 연결** (가장 쉬움) ⭐ 추천!

#### 초기 설정:
```bash
1. RoArm-M3 전원 켜기 (12V 5A 어댑터)
2. WiFi 연결:
   - SSID: RoArm-M3
   - 비밀번호: 12345678
3. 브라우저에서 접속: http://192.168.4.1
```

#### Python으로 WiFi 제어 (HTTP):
```python
# http_simple_ctrl.py
import requests

ip_addr = "192.168.4.1"  # AP 모드 기본 IP

# JSON 명령 전송
command = '{"T":106,"cmd":1.57,"spd":0,"acc":0}'  # 그리퍼 열기
url = f"http://{ip_addr}/js?json={command}"
response = requests.get(url)
print(response.text)

# 예제 명령들:
# 그리퍼 열기: {"T":106,"cmd":1.57,"spd":0,"acc":0}
# 그리퍼 닫기: {"T":106,"cmd":3.14,"spd":0,"acc":0}
# 초기화: {"T":101}
```

**실행 방법**:
```bash
# Python 데모 다운로드
wget https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_Python.zip
unzip RoArm-M3_Python.zip

# HTTP 제어 실행
python http_simple_ctrl.py 192.168.4.1
```

---

### 2️⃣ **USB Serial 연결** (안정적) ⭐ 추천!

#### 연결:
```bash
1. Type-C 케이블로 PC ↔ RoArm-M3 연결
   (ESP32 USB 포트 - No. 9번)
2. 포트 확인:
   - Linux: /dev/ttyUSB0 또는 /dev/ttyACM0
   - Windows: COM3, COM4 등
```

#### Python으로 Serial 제어:
```python
# serial_simple_ctrl.py
import serial
import threading

def read_serial():
    """로봇 피드백 수신"""
    while True:
        data = ser.readline().decode('utf-8')
        if data:
            print(f"Received: {data}", end='')

# Serial 포트 열기
ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)
ser.setRTS(False)
ser.setDTR(False)

# 수신 스레드 시작
recv_thread = threading.Thread(target=read_serial)
recv_thread.daemon = True
recv_thread.start()

# 명령 전송
while True:
    command = input("JSON Command: ")
    ser.write(command.encode() + b'\n')
```

**실행 방법**:
```bash
# 포트 확인
ls /dev/ttyUSB* /dev/ttyACM*

# Serial 제어 실행
python serial_simple_ctrl.py /dev/ttyUSB0
```

---

### 3️⃣ **ROS2 연결** (로봇 개발용)

```bash
# ROS2 설치 (Ubuntu 22.04 권장)
sudo apt install ros-humble-desktop

# RoArm-M3 ROS2 패키지 설치
git clone https://github.com/Waveshare/RoArm-M3-ROS2.git
cd RoArm-M3-ROS2
colcon build

# 실제 로봇 제어
source install/setup.bash
ros2 launch roarm_moveit demo_real.launch.py
```

---

## 🎮 JSON 명령어

### 기본 형식:
```json
{"T": 명령_타입, "cmd": 값, "spd": 속도, "acc": 가속도}
```

### 자주 쓰는 명령:

#### 1. 그리퍼 제어
```json
// 그리퍼 열기
{"T":106,"cmd":1.57,"spd":0,"acc":0}

// 그리퍼 닫기 (물체 잡기)
{"T":106,"cmd":3.14,"spd":0,"acc":0}
```

#### 2. Joint 각도 제어
```json
// Base 회전 (좌측 180도)
{"T":100,"joint":1,"cmd":3.14,"spd":0,"acc":0}

// Shoulder 제어
{"T":100,"joint":2,"cmd":0.5,"spd":0,"acc":0}

// Elbow 제어
{"T":100,"joint":3,"cmd":1.0,"spd":0,"acc":0}
```

#### 3. 좌표 제어 (X, Y, Z)
```json
// 특정 위치로 이동
{"T":103,"x":0.2,"y":0.1,"z":0.3,"spd":0,"acc":0}
```

#### 4. 시스템 명령
```json
// 초기 위치로 리셋
{"T":101}

// 토크 끄기 (수동 조작 가능)
{"T":202,"cmd":0}

// 토크 켜기
{"T":202,"cmd":1}

// LED 제어
{"T":500,"r":255,"g":0,"b":0}  // 빨강
```

---

## 🚀 빠른 시작 (Python)

### 설치:
```bash
# 필요한 패키지
pip install requests pyserial
```

### 간단한 제어 예제:

#### WiFi 버전:
```python
import requests
import time

ip = "192.168.4.1"

def send_cmd(json_cmd):
    url = f"http://{ip}/js?json={json_cmd}"
    return requests.get(url).text

# 1. 초기화
send_cmd('{"T":101}')
time.sleep(2)

# 2. 그리퍼 열기
send_cmd('{"T":106,"cmd":1.57,"spd":0,"acc":0}')
time.sleep(1)

# 3. Base 90도 회전
send_cmd('{"T":100,"joint":1,"cmd":1.57,"spd":0,"acc":0}')
time.sleep(2)

# 4. 그리퍼 닫기
send_cmd('{"T":106,"cmd":3.14,"spd":0,"acc":0}')
time.sleep(1)
```

#### Serial 버전:
```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)
ser.setRTS(False)
ser.setDTR(False)

def send_cmd(json_cmd):
    ser.write(json_cmd.encode() + b'\n')
    time.sleep(0.1)
    # 피드백 읽기
    if ser.in_waiting:
        return ser.readline().decode('utf-8')
    return ""

# 1. 초기화
send_cmd('{"T":101}')
time.sleep(2)

# 2. 그리퍼 제어
send_cmd('{"T":106,"cmd":1.57,"spd":0,"acc":0}')  # 열기
time.sleep(1)
send_cmd('{"T":106,"cmd":3.14,"spd":0,"acc":0}')  # 닫기
```

---

## 📚 공식 자료

### 다운로드:
```bash
# Python 데모
wget https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_Python.zip

# 펌웨어 (ESP32)
wget https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_example-250108.zip

# 3D 모델 (STEP)
wget https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_STEP.zip
```

### 문서:
- **공식 Wiki**: https://www.waveshare.com/wiki/RoArm-M3
- **JSON 명령어**: https://www.waveshare.com/wiki/RoArm-M3-S_JSON_Command_Meaning
- **ROS2 튜토리얼**: https://www.waveshare.com/wiki/RoArm-M3_How_to_Install_ROS2

---

## 🔧 연결 문제 해결

### WiFi 연결 안 될 때:
```bash
1. OLED 화면 확인:
   - 1번 줄: AP 모드 확인 (RoArm-M3)
   - 2번 줄: IP 주소 확인
2. WiFi 비밀번호 재확인: 12345678
3. 브라우저 캐시 삭제 후 재접속
```

### Serial 연결 안 될 때:
```bash
# 1. 포트 권한 설정
sudo chmod 666 /dev/ttyUSB0

# 2. 드라이버 설치
# CP2102 드라이버 다운로드:
wget https://files.waveshare.com/wiki/common/CP210x_USB_TO_UART.zip
unzip CP210x_USB_TO_UART.zip
# Windows: 설치 실행
# Linux: 자동 인식 (드라이버 불필요)

# 3. 포트 확인
dmesg | grep tty
```

### 로봇이 응답하지 않을 때:
```bash
1. 전원 확인 (12V 5A)
2. USB 케이블 연결 확인 (No. 9번 포트)
3. NVS 초기화 (Serial로 전송):
   {"T":604}
```

---

## 💡 실전 팁

### 1. 속도 조절:
```json
// 빠른 움직임
{"T":100,"joint":1,"cmd":1.57,"spd":0,"acc":0}  // spd=0 (최대)

// 느린 움직임
{"T":100,"joint":1,"cmd":1.57,"spd":100,"acc":50}
```

### 2. 안전한 제어:
```python
# 항상 초기화 먼저
send_cmd('{"T":101}')
time.sleep(2)

# 토크 확인
send_cmd('{"T":202,"cmd":1}')

# 부드러운 움직임 (spd, acc 지정)
send_cmd('{"T":100,"joint":1,"cmd":1.57,"spd":50,"acc":30}')
```

### 3. 피드백 활용:
```python
# Serial에서 피드백 받기
response = send_cmd('{"T":100,"joint":1,"cmd":1.57}')
print(f"로봇 응답: {response}")
```

---

## 🎯 다음 단계

1. **기본 제어 테스트**:
   ```bash
   python http_simple_ctrl.py 192.168.4.1
   ```

2. **시뮬레이션과 연동**:
   - Isaac Sim에서 학습한 정책을 실제 로봇에 적용
   - Joint 각도를 JSON 명령으로 변환

3. **RL 정책 배포**:
   ```python
   # Isaac Sim에서 학습한 모델
   action = model.predict(obs)
   
   # RoArm-M3에 전송
   for i, angle in enumerate(action[:6]):
       send_cmd(f'{{"T":100,"joint":{i+1},"cmd":{angle}}}')
   ```

---

**지금 바로 연결해보세요!** 🚀

WiFi 연결이 가장 쉽습니다:
1. RoArm-M3 WiFi 연결 (RoArm-M3 / 12345678)
2. http://192.168.4.1 접속
3. Web UI에서 조작!
