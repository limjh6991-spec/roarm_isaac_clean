# urdf-viz GUI 사용 방법

## 🎮 urdf-viz 3D Viewer 실행 및 사용법

### 📺 GUI 창 열기

#### 방법 1: 터미널에서 직접 실행
```bash
cd /home/roarm_m3/roarm_isaac_clean
urdf-viz assets/roarm_m3/urdf/roarm_m3_enhanced.urdf
```

#### 방법 2: 백그라운드로 실행
```bash
urdf-viz assets/roarm_m3/urdf/roarm_m3_enhanced.urdf &
```

#### 방법 3: 자동화 스크립트 사용
```bash
./view_urdf.sh
```

---

## 🖥️ GUI 창 구성

urdf-viz를 실행하면 **새 윈도우 창**이 열립니다:

```
┌─────────────────────────────────────────────────────────┐
│  urdf-viz - roarm_m3_enhanced.urdf              [_][□][X]│
├──────────────┬──────────────────────────────────────────┤
│              │                                          │
│  Joint       │                                          │
│  Control     │        3D Robot Model                   │
│  Panel       │        (회전/줌 가능)                    │
│              │                                          │
│ ┌──────────┐ │                                          │
│ │joint_base│ │         🤖                              │
│ │  [====] │ │        /│\                              │
│ ├──────────┤ │       / │ \                             │
│ │shoulder  │ │      📷  ✋                             │
│ │  [====] │ │                                          │
│ ├──────────┤ │                                          │
│ │elbow     │ │    좌측: Joint 슬라이더                  │
│ │  [====] │ │    중앙: 3D 모델 (마우스로 회전)          │
│ ├──────────┤ │                                          │
│ │wrist1    │ │                                          │
│ │  [====] │ │                                          │
│ ├──────────┤ │                                          │
│ │wrist2    │ │                                          │
│ │  [====] │ │                                          │
│ ├──────────┤ │                                          │
│ │gripper_L │ │                                          │
│ │  [====] │ │                                          │
│ ├──────────┤ │                                          │
│ │gripper_R │ │                                          │
│ │  [====] │ │                                          │
│ └──────────┘ │                                          │
└──────────────┴──────────────────────────────────────────┘
```

---

## 🖱️ 마우스 조작

### 3D 뷰 제어:
| 조작 | 기능 | 설명 |
|------|------|------|
| **좌클릭 + 드래그** | 카메라 회전 | 로봇을 다양한 각도에서 관찰 |
| **마우스 휠 위/아래** | 줌 인/아웃 | 로봇 확대/축소 |
| **우클릭 + 드래그** | 카메라 이동 (Pan) | 로봇 위치 이동 |
| **휠 버튼 + 드래그** | 카메라 이동 (Pan) | 대체 방법 |

### Joint 제어:
| 조작 | 기능 |
|------|------|
| **슬라이더 클릭 + 드래그** | Joint 각도/위치 변경 |
| **슬라이더 더블클릭** | 중앙값으로 리셋 |
| **슬라이더 휠** | 미세 조정 |

---

## ⌨️ 키보드 단축키

### 뷰 제어:
| 키 | 기능 | 설명 |
|----|------|------|
| **↑ ↓ ← →** | 카메라 이동 | 화살표 키로 Pan |
| **Home** | 뷰 리셋 | 초기 카메라 위치로 복귀 |
| **+** / **-** | 줌 인/아웃 | 키보드로 줌 제어 |

### 표시 옵션:
| 키 | 기능 | 설명 |
|----|------|------|
| **c** | Collision Geometry 토글 | 충돌 형상 표시/숨김 |
| **v** | Visual Geometry 토글 | 시각 형상 표시/숨김 |
| **g** | Grid 토글 | 바닥 그리드 표시/숨김 |
| **a** | Axes 토글 | 좌표축 표시/숨김 |
| **l** | Link Names 토글 | 링크 이름 라벨 표시 |
| **j** | Joint Names 토글 | 조인트 이름 라벨 표시 |

### 기타:
| 키 | 기능 | 설명 |
|----|------|------|
| **r** | Reset Robot | 모든 Joint를 0 위치로 리셋 |
| **Space** | Play/Pause Animation | 애니메이션 재생/정지 |
| **q** 또는 **Esc** | 종료 | urdf-viz 닫기 |
| **F11** | 전체화면 토글 | 전체화면 모드 |

---

## 🎯 Enhanced URDF에서 확인할 사항

### 1. 🤖 전체 로봇 구조 (13 Links)
```
base_link (gray) → link_base (gray) → link_shoulder (gray)
  → link_elbow (gray) → link_wrist1 (gray) → link_wrist2 (gray)
  → gripper_base_link (gray)
    ├── gripper_left_finger (red)
    ├── gripper_right_finger (red)
    └── camera_link (black) + 4 optical frames
```

### 2. 🎮 Joint 슬라이더 (7개)

#### Arm Joints (5개):
| # | Joint 이름 | 타입 | 범위 | 색상 |
|---|------------|------|------|------|
| 1 | `joint_base` | Continuous | ±∞° | 회전 무제한 |
| 2 | `joint_shoulder` | Revolute | 0~180° | 어깨 |
| 3 | `joint_elbow` | Revolute | 0~135° | 팔꿈치 |
| 4 | `joint_wrist1` | Revolute | ±90° | 손목1 |
| 5 | `joint_wrist2` | Continuous | ±∞° | 손목2 |

#### Gripper Joints (2개):
| # | Joint 이름 | 타입 | 범위 | 동작 |
|---|------------|------|------|------|
| 6 | `gripper_left_finger_joint` | Prismatic | 0~40mm | Master (독립) |
| 7 | `gripper_right_finger_joint` | Prismatic | 0~40mm | **Mimic** (자동 추종) |

**⚠️ 중요**: `gripper_right_finger_joint`는 `gripper_left_finger_joint`를 mimic하므로, 
**left 슬라이더를 움직이면 right가 자동으로 대칭 이동**합니다!

### 3. 📸 Camera 위치 확인

**RealSense D455 Camera**:
- **위치**: gripper_base_link 끝, 5cm 전방, 2cm 위
- **각도**: 30도 아래로 기울어짐 (작업 공간 향함)
- **색상**: 검정색 (Black) 작은 박스
- **크기**: 26mm(H) × 124mm(W) × 29mm(D)

**확인 방법**:
1. `joint_wrist2` 슬라이더를 움직여 gripper 회전
2. Gripper 끝에 검정색 작은 박스(카메라) 확인
3. 카메라가 약간 앞으로 돌출되어 있음
4. 'a' 키를 눌러 axes를 표시하면 카메라 방향 확인 가능

### 4. ✋ Gripper 동작 테스트

**Mimic Joint 테스트**:
1. `gripper_left_finger_joint` 슬라이더 찾기
2. 슬라이더를 왼쪽(0mm)으로 이동 → **Gripper 닫힘** (양쪽 손가락 모두)
3. 슬라이더를 오른쪽(40mm)으로 이동 → **Gripper 열림** (양쪽 손가락 모두)
4. `gripper_right_finger_joint` 슬라이더는 자동으로 따라감 (읽기 전용처럼 동작)

**예상 동작**:
```
Left: 0mm  → Right: 0mm  (닫힘) ✋→✊
Left: 20mm → Right: 20mm (반만 열림) ✋→🤏
Left: 40mm → Right: 40mm (완전히 열림) ✋→🖐️
```

---

## 🎨 색상 구분

### Links 색상:
| Link | 색상 | RGB | 용도 |
|------|------|-----|------|
| Base/Arm links | **Gray** | (0.5, 0.5, 0.5) | 주 구조 |
| base_link | **Aluminum** | (0.7, 0.7, 0.7) | 베이스 플레이트 |
| Gripper fingers | **Red** | (0.8, 0.1, 0.1) | 강조 (end effector) |
| Camera | **Black** | (0.15, 0.15, 0.15) | 센서 장치 |

---

## 🐛 문제 해결

### GUI 창이 열리지 않을 때:

#### 1. X11 디스플레이 확인:
```bash
echo $DISPLAY
# 출력 예: :0 또는 :1
```

#### 2. urdf-viz 재시작:
```bash
# 기존 프로세스 종료
pkill urdf-viz

# 다시 실행
urdf-viz assets/roarm_m3/urdf/roarm_m3_enhanced.urdf
```

#### 3. OpenGL 지원 확인:
```bash
glxinfo | grep "OpenGL version"
# 출력 예: OpenGL version string: 4.6.0
```

#### 4. 권한 문제:
```bash
xhost +local:
# X11 접근 권한 허용
```

### GUI 창이 느릴 때:

```bash
# 하드웨어 가속 확인
glxinfo | grep "direct rendering"
# 출력: direct rendering: Yes (정상)

# Mesa 드라이버 업데이트
sudo apt-get update
sudo apt-get install -y mesa-utils libgl1-mesa-glx
```

### Joint 슬라이더가 보이지 않을 때:

urdf-viz는 자동으로 제어 가능한 Joint(revolute, continuous, prismatic)만 슬라이더로 표시합니다.
Fixed joint는 표시되지 않습니다 (camera joints 등).

**Expected: 7개 슬라이더**
- 5 Arm joints (base, shoulder, elbow, wrist1, wrist2)
- 2 Gripper joints (left_finger, right_finger)

**Not shown: 6개 Fixed joints**
- gripper_base_joint
- camera_joint
- camera_depth_joint
- camera_depth_optical_joint
- camera_color_joint
- camera_color_optical_joint

---

## 📸 스크린샷 캡처

### 방법 1: Linux 스크린샷 도구
```bash
# Gnome Screenshot (우분투)
gnome-screenshot -w  # 현재 창만

# 또는 PrintScreen 키
```

### 방법 2: urdf-viz 내장 기능
GUI 창에서:
1. 원하는 각도로 로봇 배치
2. **'s'** 키 누르기 → 스크린샷 저장 (screenshot.png)

---

## 🎬 애니메이션 재생

urdf-viz는 Joint 애니메이션 기능을 지원합니다:

1. **Space** 키를 눌러 자동 애니메이션 시작
2. 모든 Joint가 자동으로 범위 내에서 움직임
3. 다시 **Space**를 눌러 정지
4. **'r'** 키로 초기 위치로 리셋

---

## 💡 추천 확인 순서

### Step 1: 전체 구조 확인
1. urdf-viz 실행
2. **마우스 드래그**로 로봇을 360도 회전
3. **'g'** 키로 Grid 활성화
4. **'a'** 키로 Axes 활성화 (X축: 빨강, Y축: 초록, Z축: 파랑)

### Step 2: Arm Joints 테스트
1. `joint_shoulder` 슬라이더를 0 → 180° 이동
2. `joint_elbow` 슬라이더를 0 → 135° 이동
3. 팔이 펴지고 접히는 동작 확인

### Step 3: Gripper Mimic Joint 테스트
1. `gripper_left_finger_joint` 슬라이더 찾기
2. 0mm → 40mm 이동
3. **양쪽 손가락이 동시에 대칭으로** 벌어지는지 확인 ✅

### Step 4: Camera 위치 확인
1. 로봇을 측면에서 보기
2. Gripper 끝에 검정색 작은 박스 찾기
3. 카메라가 약간 아래를 향하는지 확인 (30°)
4. **'l'** 키로 링크 이름 표시 → `camera_link` 확인

### Step 5: End Effector 동작 확인
1. Wrist joints를 움직여 end effector 방향 변경
2. Gripper를 열고 닫으며 작업 공간 확인
3. Camera가 항상 end effector를 따라가는지 확인

---

## 📊 확인 완료 체크리스트

- [ ] ✅ GUI 창이 정상적으로 열림
- [ ] ✅ 7개 Joint 슬라이더가 모두 보임
- [ ] ✅ 3D 로봇 모델이 렌더링됨 (13 links)
- [ ] ✅ 마우스로 카메라 회전 가능
- [ ] ✅ Joint 슬라이더로 로봇 제어 가능
- [ ] ✅ Gripper left 슬라이더 → right가 자동 추종 (mimic)
- [ ] ✅ Camera(검정 박스)가 gripper 끝에 부착됨
- [ ] ✅ Camera가 약간 아래를 향함 (30°)
- [ ] ✅ 전체 동작이 자연스러움

---

## 🚀 다음 단계

GUI에서 Enhanced URDF 확인 완료 후:

### Isaac Sim 통합:
```bash
# USD 변환
python scripts/utils/urdf_to_usd.py

# 또는 Isaac Sim GUI에서 직접 Import
# Assets → Import Robot → Browse:
# assets/roarm_m3/urdf/roarm_m3_enhanced.urdf
```

### 실제 RL 환경 테스트:
1. Enhanced URDF로 로봇 교체
2. Gripper action space 업데이트 (1D → 2D)
3. Camera sensor 추가 (RGB 256x256)
4. 100K steps 학습 후 GRIP rate 측정

---

**작성일**: 2024-10-28  
**urdf-viz 버전**: v0.46.1  
**URDF**: roarm_m3_enhanced.urdf (380 lines, 7 DOF)
