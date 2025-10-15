# RoArm-M3 Wiki 요약

**출처**: https://www.waveshare.com/wiki/RoArm-M3  
**수집일**: 2025년 10월 15일

## 기본 스펙

### 자유도 (DOF)
- **5+1 DOF** (5개 관절 + 1개 그리퍼)
- Base: 360° 회전 (3.14 ~ -3.14 rad)
- Shoulder: 180° 회전 (1.57 ~ -1.57 rad)
- Elbow: 180° 회전
- Wrist1: 180° 회전 (1.57 ~ -1.57 rad)
- Wrist2: 360° 회전 (3.14 ~ -3.14 rad)
- Gripper: 135° 개폐 (3.14 ~ 1.08 rad)

### 작업 공간
- 직경: 최대 1m
- 유효 하중: 0.2kg (0.5m 거리에서)

### 제어 방식
- JSON 명령 기반
- 웹 인터페이스 제공
- HTTP/Serial/ESP-NOW 통신 지원
- ROS2 호환

## 하드웨어 정보

### 서보 모터
- RoArm-M3-S: ST3215 Servo (플라스틱 쉘)
- RoArm-M3-Pro: ST3235 Servo (메탈 쉘, 그리퍼 제외)
- 12-bit 고정밀 마그네틱 엔코더
- 재위치 정확도: 0.088°

### 컨트롤러
- ESP32-WROOM-32
- WiFi 지원 (AP/STA 모드)
- 기본 WiFi: RoArm-M3, PW: 12345678
- 웹 주소: 192.168.4.1

### 전원
- 입력: DC 7~12.6V
- 권장: 12V 5A 또는 3S 리튬 배터리

## 관절 구조

```
Base (기본 관절)
  ↓
Shoulder (어깨 관절) - Dual-drive 기술
  ↓
Elbow (팔꿈치 관절)
  ↓
Wrist1 (손목 관절 1)
  ↓
Wrist2 (손목 관절 2)
  ↓
Gripper (그리퍼/집게)
```

## 제공되는 자료

### 3D 모델
- STEP 파일: https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_STEP.zip
- 2D 치수: https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_2Dsize.zip

### 소프트웨어
- 오픈소스 프로그램: https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_example-250108.zip
- Python Demo: https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_Python.zip

### ROS2 튜토리얼
1. ROS2 설치 방법
2. ROS2 Workspace 설명
3. 실제 로봇팔 제어
4. Moveit2 드래그 앤 드롭
5. 키보드 제어
6. 명령어 제어
7. Moveit MTC 데모

## JSON 명령 예시

### 그리퍼 열기
```json
{"T":106,"cmd":1.57,"spd":0,"acc":0}
```
- T: 명령 타입 (106 = 그리퍼 제어)
- cmd: 목표 각도 (radians)
- spd: 속도 (0 = 최대 속도)
- acc: 가속도 (0 = 최대 가속도)

## 통신 방법

### 1. HTTP (WiFi)
```python
import requests
url = f"http://{ip_addr}/js?json={command}"
response = requests.get(url)
```

### 2. Serial (UART)
- 보드레이트: 115200
- 양방향 통신
- 저지연, 안정적

### 3. ROS2
- Moveit2 호환
- 시뮬레이션 환경 제공

## 주의사항

1. **전압 범위**: 7-12.6V 엄수 (초과 금지)
2. **고 토크**: 서보 모터 토크가 높아 안전 주의
3. **분해 비권장**: 공장 조립 상태 유지 권장
4. **속도 설정**: 기본 속도는 느림 (안전), 조정 가능

## URDF/USD 변환 가능성

### 제공되는 파일
- ✅ STEP 3D 모델 (기계 설계 파일)
- ✅ 2D 치수 도면
- ✅ ROS2 지원 (URDF 존재 가능성 높음)

### 다음 단계
1. RoArm-M3 오픈소스 프로그램 다운로드
2. ROS2 관련 파일에서 URDF 확인
3. URDF → USD 변환

## LeRobot 통합

- RoArm-M3는 LeRobot 프로젝트 지원
- 사전 학습 모델, 데이터셋, 시뮬레이션 환경 제공
- 딥러닝, 모방 학습, 강화학습 지원

## 참고 링크

- 공식 Wiki: https://www.waveshare.com/wiki/RoArm-M3
- 제품 페이지: https://www.waveshare.com/roarm-m3.htm
- 기술 지원: https://service.waveshare.com/
