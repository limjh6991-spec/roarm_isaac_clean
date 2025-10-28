# 🛠️ RoArm-M3 WiFi 연결 완벽 가이드 (문제 해결 포함)

## 🔍 현재 상태 확인

### 1단계: RoArm-M3 전원 확인
```bash
✅ 체크리스트:
□ 12V 5A 어댑터 연결됨
□ 전원 스위치 ON 위치 (드라이버 보드의 No. 12)
□ OLED 화면에 내용 표시됨
□ 모든 Joint가 중간 위치로 자동 이동함
```

**OLED 화면이 안 켜지면**:
- 전원 어댑터 확인 (12V 5A 필수)
- 전원 스위치 위치 확인
- 전원 케이블 접촉 확인

---

## 📱 WiFi 연결 (상세)

### 2단계: WiFi 모드 확인

RoArm-M3 전원을 켜고 **OLED 화면**을 확인하세요:

```
┌─────────────────────┐
│ AP:RoArm-M3         │ ← 1번째 줄 (AP 모드)
│ STA:OFF             │ ← 2번째 줄 (STA 모드)
│ AA:BB:CC:DD:EE:FF   │ ← 3번째 줄 (MAC 주소)
└─────────────────────┘
```

**1번째 줄 설명**:
- `AP:RoArm-M3` → WiFi 핫스팟 이름
- 이 이름이 보이면 OK!

**2번째 줄 설명**:
- `STA:OFF` → 공유기 연결 안 함 (처음엔 정상)
- `STA:192.168.x.x` → 공유기에 연결됨 (나중에)

---

### 3단계: PC/스마트폰에서 WiFi 연결

#### 방법 1: 스마트폰 (더 쉬움) ⭐

```bash
1. 스마트폰 설정 → WiFi
2. 사용 가능한 네트워크 찾기
3. "RoArm-M3" 선택
4. 비밀번호 입력: 12345678
5. 연결 완료!
```

**자주 발생하는 문제**:

❌ **"RoArm-M3" WiFi가 안 보여요**
```bash
해결:
1. RoArm-M3 전원 껐다 켜기 (리부트)
2. 30초 대기 (WiFi 활성화 시간)
3. 스마트폰 WiFi 껐다 켜기
4. WiFi 목록 새로고침
5. 2.4GHz WiFi 채널 확인 (5GHz 아님!)
```

❌ **비밀번호가 틀렸다고 나와요**
```bash
확인:
- 정확히 입력: 12345678 (숫자 8개)
- 대소문자 없음, 공백 없음
- 복사-붙여넣기 사용 추천
```

❌ **"인터넷에 연결되지 않음" 경고**
```bash
→ 정상입니다! 무시하세요.
RoArm-M3는 로컬 네트워크만 제공합니다.
"계속 사용" 또는 "연결 유지" 선택
```

#### 방법 2: PC (Linux)

```bash
# 1. WiFi 스캔
nmcli device wifi list

# 2. RoArm-M3 연결
nmcli device wifi connect "RoArm-M3" password "12345678"

# 3. 연결 확인
nmcli connection show --active
```

**연결 성공 확인**:
```bash
# IP 확인
ip addr show wlan0 | grep inet

# 결과:
inet 192.168.4.2/24 brd 192.168.4.255 scope global dynamic wlan0
      ^^^^^^^^
      이 IP를 받았으면 성공!
```

---

### 4단계: Web 브라우저 접속

#### A. 스마트폰

```bash
1. Chrome 또는 Safari 열기
2. 주소창에 입력: 192.168.4.1
3. Enter!
```

**화면이 안 뜨면**:
```bash
시도 1: http:// 명시
  → http://192.168.4.1

시도 2: 캐시 삭제
  → Chrome: 설정 → 개인정보 → 인터넷 사용기록 삭제

시도 3: 시크릿 모드
  → Chrome: 새 시크릿 탭

시도 4: 다른 브라우저
  → Safari, Firefox 등
```

#### B. PC (Linux)

```bash
# 1. 기본 브라우저
xdg-open http://192.168.4.1

# 2. Chrome 직접
google-chrome http://192.168.4.1

# 3. Firefox
firefox http://192.168.4.1
```

---

## 🎮 Web UI 사용법

성공하면 이런 화면이 나옵니다:

```
┌──────────────────────────────────┐
│  RoArm-M3 Web Control            │
├──────────────────────────────────┤
│  AngleCtrl: Servo Angle Control  │
│  [B L] [B R]  Base: 0.00         │
│  [S D] [S U]  Shoulder: 0.00     │
│  [E D] [E U]  Elbow: 1.57        │
│  ...                             │
│  [INIT]  [Torque ON/OFF]         │
└──────────────────────────────────┘
```

**첫 테스트**:
```bash
1. "INIT" 버튼 클릭 → 로봇이 초기 자세로
2. "G-UG" 버튼 클릭 → 그리퍼가 열림
3. "G+DG" 버튼 클릭 → 그리퍼가 닫힘
```

---

## 🐛 문제 해결 (상세)

### 문제 1: WiFi 목록에 "RoArm-M3"가 안 보임

**원인**: ESP32 WiFi 모듈 문제

**해결**:
```bash
# 1. 완전 전원 차단
→ 전원 스위치 OFF
→ 어댑터 뽑기
→ 10초 대기
→ 다시 연결

# 2. OLED 화면 확인
→ "AP:RoArm-M3" 표시되는지 확인
→ 안 보이면 펌웨어 문제 (아래 참조)

# 3. 주변 WiFi 간섭 확인
→ 2.4GHz 채널 혼잡도 확인
→ 라우터에서 멀리 떨어진 곳에서 테스트
```

---

### 문제 2: 연결은 되는데 192.168.4.1이 안 열림

**원인**: IP 할당 문제 또는 방화벽

**해결**:
```bash
# A. IP 확인
# 스마트폰:
→ WiFi 설정 → RoArm-M3 정보
→ IP 주소 확인 (192.168.4.2 등)
→ 게이트웨이 확인 (192.168.4.1이어야 함)

# PC (Linux):
ip addr show wlan0
ip route show

# B. Ping 테스트
ping -c 3 192.168.4.1

# 성공:
64 bytes from 192.168.4.1: icmp_seq=1 ttl=64 time=5.23 ms
→ 연결 OK, 브라우저 문제

# 실패:
Destination Host Unreachable
→ 네트워크 문제

# C. 방화벽 확인 (PC)
sudo ufw status
→ inactive면 OK
→ active면 일시적으로 끄기:
  sudo ufw disable

# D. 수동 IP 설정 (최후 수단)
# PC (Linux):
sudo nmcli connection modify "RoArm-M3" ipv4.addresses 192.168.4.100/24
sudo nmcli connection modify "RoArm-M3" ipv4.gateway 192.168.4.1
sudo nmcli connection down "RoArm-M3"
sudo nmcli connection up "RoArm-M3"
```

---

### 문제 3: 펌웨어 리셋 필요

**증상**:
- OLED에 "AP:RoArm-M3" 안 보임
- WiFi 전혀 작동 안 함
- Web에서 JSON 명령 오류

**해결**:
```bash
# 1. 펌웨어 다운로드
wget https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_FACTORY-250113.zip
unzip RoArm-M3_FACTORY-250113.zip

# 2. USB Serial 연결
→ Type-C 케이블로 PC ↔ RoArm-M3 연결
→ USB 포트 No. 9 사용

# 3. 포트 확인
ls /dev/ttyUSB* /dev/ttyACM*
# 결과: /dev/ttyUSB0

# 4. NVS 초기화 (Serial로)
python3 << 'EOF'
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)
ser.setRTS(False)
ser.setDTR(False)

# NVS 초기화 명령
cmd = '{"T":604}\n'
ser.write(cmd.encode())
time.sleep(1)

# 응답 확인
if ser.in_waiting:
    print(ser.read(ser.in_waiting).decode())

ser.close()
print("✅ NVS 초기화 완료. 전원을 껐다 켜세요.")
EOF

# 5. 전원 재부팅
→ 전원 스위치 OFF → ON
→ OLED 확인: "AP:RoArm-M3"
```

---

## 🚀 Python으로 WiFi 제어 (연결 성공 후)

### 간단한 테스트:

```python
#!/usr/bin/env python3
"""RoArm-M3 WiFi 연결 테스트"""

import requests
import time

# RoArm-M3 IP (AP 모드 기본값)
ip = "192.168.4.1"

def send_cmd(json_cmd):
    """JSON 명령 전송"""
    url = f"http://{ip}/js?json={json_cmd}"
    try:
        response = requests.get(url, timeout=5)
        print(f"✅ 명령 전송: {json_cmd}")
        print(f"📨 응답: {response.text}")
        return response.text
    except requests.exceptions.ConnectionError:
        print("❌ 연결 실패! RoArm-M3 WiFi에 연결되었는지 확인하세요.")
        return None
    except requests.exceptions.Timeout:
        print("⏱️ 응답 시간 초과")
        return None

# 연결 테스트
print("🔍 RoArm-M3 연결 테스트...")
print(f"   WiFi: RoArm-M3")
print(f"   IP: {ip}\n")

# 1. 초기화
print("1️⃣ 초기화 (INIT)")
send_cmd('{"T":101}')
time.sleep(2)

# 2. 그리퍼 열기
print("\n2️⃣ 그리퍼 열기")
send_cmd('{"T":106,"cmd":1.57,"spd":0,"acc":0}')
time.sleep(1)

# 3. 그리퍼 닫기
print("\n3️⃣ 그리퍼 닫기")
send_cmd('{"T":106,"cmd":3.14,"spd":0,"acc":0}')
time.sleep(1)

# 4. Base 회전
print("\n4️⃣ Base 90도 회전")
send_cmd('{"T":100,"joint":1,"cmd":1.57,"spd":0,"acc":0}')
time.sleep(2)

# 5. 초기 위치
print("\n5️⃣ 초기 위치로 복귀")
send_cmd('{"T":101}')

print("\n✅ 테스트 완료!")
```

**실행**:
```bash
# 1. requests 설치
pip install requests

# 2. 스크립트 저장
nano test_roarm_wifi.py
# (위 코드 붙여넣기)

# 3. 실행
python3 test_roarm_wifi.py
```

**성공 시 출력**:
```
🔍 RoArm-M3 연결 테스트...
   WiFi: RoArm-M3
   IP: 192.168.4.1

1️⃣ 초기화 (INIT)
✅ 명령 전송: {"T":101}
📨 응답: {"T":101,"status":"ok"}

2️⃣ 그리퍼 열기
✅ 명령 전송: {"T":106,"cmd":1.57,"spd":0,"acc":0}
📨 응답: {"T":106,"status":"ok"}
...
```

**실패 시**:
```
❌ 연결 실패! RoArm-M3 WiFi에 연결되었는지 확인하세요.

→ 해결:
1. WiFi 연결 확인: nmcli connection show --active
2. Ping 테스트: ping 192.168.4.1
3. 브라우저 테스트: http://192.168.4.1
```

---

## 📋 체크리스트 (순서대로)

```bash
□ 1. RoArm-M3 전원 켜짐 (12V 5A)
□ 2. OLED 화면에 "AP:RoArm-M3" 표시
□ 3. PC/스마트폰에서 "RoArm-M3" WiFi 보임
□ 4. WiFi 연결 성공 (비밀번호: 12345678)
□ 5. IP 192.168.4.2 받음 (또는 .3, .4 등)
□ 6. Ping 성공: ping 192.168.4.1
□ 7. 브라우저에서 http://192.168.4.1 열림
□ 8. Web UI 화면 표시됨
□ 9. INIT 버튼 클릭 → 로봇 반응함
□ 10. Python 스크립트 실행 성공
```

---

## 🆘 그래도 안 되면?

### 최종 해결책:

1. **USB Serial 방식 사용**:
   ```bash
   # WiFi 대신 USB로 직접 연결
   # 더 안정적이고 빠름
   python serial_simple_ctrl.py /dev/ttyUSB0
   ```

2. **공유기 연결 (STA 모드)**:
   ```bash
   # RoArm-M3를 집 WiFi에 연결
   # Web UI → WIFI Config에서 설정
   # 더 안정적인 연결
   ```

3. **펌웨어 재설치**:
   ```bash
   # ESP32 펌웨어 다시 설치
   wget https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_example-250108.zip
   # Arduino IDE로 업로드
   ```

---

## 💬 자주 묻는 질문

**Q: WiFi 비밀번호를 바꿀 수 있나요?**
```bash
A: 네! Web UI → Settings에서 변경 가능
또는 Serial로 JSON 명령:
{"T":600,"ssid":"MyRobot","pwd":"NewPassword123"}
```

**Q: 여러 기기에서 동시 연결 가능한가요?**
```bash
A: 네! AP 모드는 최대 4개 기기 동시 연결 지원
```

**Q: WiFi 범위는 얼마나 되나요?**
```bash
A: 약 10-20m (장애물 없을 때)
벽이 있으면 5-10m
```

**Q: 5GHz WiFi 지원하나요?**
```bash
A: 아니요. 2.4GHz만 지원합니다.
```

---

**지금 바로 시도해보세요!** 🚀

궁금한 점이나 문제가 있으면 알려주세요!
