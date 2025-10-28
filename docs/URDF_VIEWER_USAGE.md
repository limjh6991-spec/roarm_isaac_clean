# URDF Viewer 사용 가이드

## 설치 완료
✅ **urdf-viz v0.46.1** 설치 완료 (2024-10-28)
- 빌드 시간: 28.88s
- 패키지: 351개
- 실행 파일: `/home/roarm_m3/.cargo/bin/urdf-viz`

## 실행 방법

### 방법 1: 자동화 스크립트 사용 (권장)
```bash
cd /home/roarm_m3/roarm_isaac_clean
./view_urdf.sh
```

**스크립트 기능:**
- ✅ URDF 파일 존재 확인
- ✅ `check_urdf`로 유효성 검사
- ✅ urdf-viz 자동 실행 (--joint 옵션)
- ✅ Link/Joint 구조 출력
- ✅ Camera frame 목록 출력

### 방법 2: 직접 실행
```bash
cd /home/roarm_m3/roarm_isaac_clean

# Joint control UI와 함께 실행
urdf-viz --joint assets/roarm_m3/urdf/roarm_m3_enhanced.urdf

# 기본 실행 (Joint UI 없음)
urdf-viz assets/roarm_m3/urdf/roarm_m3_enhanced.urdf
```

## urdf-viz 조작법

### 🖱️ 카메라 제어
| 조작 | 기능 |
|------|------|
| **마우스 드래그** | 카메라 회전 |
| **마우스 휠** | 줌 인/아웃 |
| **↑ ↓ ← →** | 카메라 이동 |
| **Home** | 카메라 리셋 |

### ⚙️ Joint 제어 (--joint 옵션 사용 시)
- **UI 슬라이더**: 각 Joint 개별 제어
- **실시간 시각화**: Joint 변화 즉시 반영

**제어 가능한 Joints (7개):**
1. `joint_base` (Revolute, ±90°)
2. `joint_shoulder` (Revolute, 0~180°)
3. `joint_elbow` (Revolute, 0~135°)
4. `joint_wrist1` (Revolute, ±90°)
5. `joint_wrist2` (Revolute, ±135°)
6. `gripper_left_finger_joint` (Prismatic, 0~40mm)
7. `gripper_right_finger_joint` (Prismatic, mimic)

### 🔍 표시 옵션
| 키 | 기능 |
|----|------|
| **c** | Collision geometry 토글 |
| **v** | Visual geometry 토글 |
| **g** | Grid 표시 토글 |
| **a** | Axes 표시 토글 |
| **j** | Joint 제어 UI 토글 |
| **r** | 로봇 리셋 (Joint 초기화) |
| **q** 또는 **Esc** | 종료 |

## Enhanced URDF 확인 사항

### 1. 전체 링크 구조 (13 links)
```
base_link (root)
├── link_base
├── link_shoulder
├── link_elbow
├── link_wrist1
├── link_wrist2
└── gripper_base_link
    ├── gripper_left_finger
    ├── gripper_right_finger
    └── camera_link
        ├── camera_depth_frame
        ├── camera_depth_optical_frame
        ├── camera_color_frame
        └── camera_color_optical_frame
```

### 2. Gripper 동작 확인
**Left Finger Joint를 움직이면:**
- ✅ Left finger: 0~40mm 이동 (master)
- ✅ Right finger: 자동으로 동일하게 이동 (mimic)
- ✅ 양쪽 손가락 대칭적으로 벌어짐/닫힘

**Mimic Joint 설정:**
```xml
<joint name="gripper_right_finger_joint" type="prismatic">
  <mimic joint="gripper_left_finger_joint" multiplier="1" offset="0"/>
</joint>
```

### 3. Camera 위치 확인
**RealSense D455 위치:**
- **Parent**: gripper_base_link
- **Offset**: `xyz="0.05 0 0.02"` (그리퍼 앞 5cm, 위 2cm)
- **Orientation**: `rpy="0 0.524 0"` (30도 아래 향함)
- **크기**: 26mm(H) × 124mm(W) × 29mm(D)
- **색상**: Black (RGB: 0.15, 0.15, 0.15)

**시각화에서 확인:**
- Camera link는 gripper 앞쪽에 작은 박스로 표시
- Camera가 작업 공간을 향하도록 각도 조정됨

### 4. Inertial Properties 확인
각 링크의 질량 분포:
| Link | Mass (kg) | 특징 |
|------|-----------|------|
| base_link | 0.5 | 가장 무거움 (지지대) |
| link_shoulder | 0.4 | 큰 모터 |
| link_elbow | 0.3 | 중간 크기 |
| link_wrist1/2 | 0.08 | 가벼운 손목 |
| gripper_base | 0.15 | 그리퍼 본체 |
| gripper_finger | 0.04 | 각 손가락 |
| camera_link | 0.072 | D455 실제 무게 |

## URDF Graph 생성 (선택사항)

시각적 트리 다이어그램 생성:
```bash
cd /home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf

# Generate .gv file
urdf_to_graphiz roarm_m3_enhanced.urdf

# Convert to PDF
dot -Tpdf robot.gv -o urdf_graph.pdf

# Convert to PNG
dot -Tpng robot.gv -o urdf_graph.png -Gdpi=300
```

**생성 파일:**
- `robot.gv`: Graphviz dot format
- `urdf_graph.pdf`: 고해상도 PDF
- `urdf_graph.png`: PNG 이미지 (300 DPI)

## 문제 해결

### urdf-viz가 실행되지 않을 때
```bash
# Cargo 환경 변수 로드
source $HOME/.cargo/env

# 다시 실행
urdf-viz --version  # v0.46.1 확인

# PATH 확인
echo $PATH | grep .cargo/bin
```

### OpenGL 오류 발생 시
```bash
# 하드웨어 가속 확인
glxinfo | grep "OpenGL version"

# Mesa 드라이버 업데이트
sudo apt-get update
sudo apt-get install -y mesa-utils libgl1-mesa-glx
```

### Joint가 움직이지 않을 때
1. `--joint` 옵션 사용 확인
2. Joint UI 패널에서 슬라이더 확인
3. Joint limit 범위 확인 (URDF 파일 참조)

## 다음 단계: Isaac Sim 통합

### Step 1: URDF Import
```python
from omni.isaac.urdf import _urdf

# Import 설정
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False  # Camera frames 유지
import_config.import_inertia_tensor = True  # Inertial properties 포함

# URDF Import 실행
success, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_enhanced.urdf",
    import_config=import_config,
)

print(f"Import success: {success}")
print(f"Prim path: {prim_path}")
```

### Step 2: Gripper Mimic Joint 구현
```python
# envs/robot/robot_controller.py

def apply_gripper_action(self, gripper_position):
    """
    Apply gripper action with mimic joint behavior.
    
    Args:
        gripper_position: 0.0 (closed) ~ 1.0 (open)
    """
    # Normalize to joint range (0~40mm)
    left_pos = gripper_position * 0.04
    right_pos = gripper_position * 0.04  # Mimic
    
    # Apply to both fingers
    self._articulation.set_joint_positions(
        [left_pos, right_pos],
        joint_indices=[6, 7]  # gripper_left, gripper_right
    )
```

### Step 3: Camera Sensor 추가
```python
from omni.isaac.sensor import Camera

# RealSense D455 Camera 생성
camera = Camera(
    prim_path="/World/RoArm_M3/camera_link/Camera",
    frequency=20,  # 20 FPS
    resolution=(256, 256),
    position=np.array([0.05, 0, 0.02]),  # Gripper 앞
    orientation=euler_angles_to_quat([0, 30, 0])  # 30도 아래
)
camera.initialize()

# RGB 이미지 가져오기
rgb_image = camera.get_rgba()[:, :, :3]  # Remove alpha
```

## 성능 벤치마크

### 기존 URDF (roarm_m3.urdf)
- ❌ GRIP rate: 0%
- ⚠️ reach_rate: 18% (10M steps)
- 문제: 단순 gripper, 카메라 없음

### Enhanced URDF (roarm_m3_enhanced.urdf)
**예상 성능 (Isaac Sim):**
- ✅ GRIP rate: **30%+** (2-finger gripper)
- ✅ reach_rate: **40%+** (Vision RL)
- ✅ 안정성 향상 (Inertial properties)
- ✅ 시뮬레이션 정확도 향상 (Joint dynamics)

**실험 계획:**
1. Enhanced URDF로 100K steps 학습
2. GRIP rate 측정
3. 기존 URDF와 비교
4. Vision RL 활성화 (Phase 2)

## 참고 자료

### URDF 상세 리포트
- 📄 `docs/URDF_ENHANCEMENT_REPORT.md`

### 원본 자료
- 📁 `resources/grippers/realsense-ros/`
- 📁 `resources/grippers/robotiq/`

### Phase 2 리소스 요약
- 📄 `resources/RESOURCE_COLLECTION_SUMMARY.md`

---

**작성일**: 2024-10-28  
**urdf-viz 버전**: v0.46.1  
**URDF 파일**: roarm_m3_enhanced.urdf (380 lines)
