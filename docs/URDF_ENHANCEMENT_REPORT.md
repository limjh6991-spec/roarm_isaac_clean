# URDF Enhancement Report

## 개선 완료 시각 (2025-10-28 18:45)

---

## 1. 기존 URDF 문제점 분석

### roarm_m3.urdf (Original)
```xml
<!-- 문제점들 -->
1. Inertial 속성 부정확
   - 모든 링크 mass=1.0 (비현실적)
   - COM (Center of Mass) 위치 미설정
   - Inertia tensor 단순화 (대각 성분만)

2. Collision geometry 미흡
   - Visual과 Collision geometry 동일 (최적화 안됨)
   - 복잡한 형상에 단순 실린더 사용

3. 그리퍼 구조 단순
   - 단일 revolute joint (1-DOF)
   - Mimic joint 없음 (대칭 파지 불가)
   - Contact sensor 위치 불명확

4. 카메라 없음
   - Vision-based RL 불가능
   - RGB-D observation 지원 안됨

5. Joint dynamics 미설정
   - Damping, friction 없음
   - 물리 시뮬레이션 불안정
```

---

## 2. 개선 사항 (roarm_m3_enhanced.urdf)

### ✅ Inertial Properties (관성 속성)
```xml
<!-- 예시: link_shoulder -->
<inertial>
  <mass value="0.4"/>  <!-- 현실적인 질량 -->
  <origin xyz="0 0 0.075" rpy="0 0 0"/>  <!-- COM 위치 -->
  <inertia ixx="0.00075" iyy="0.00075" izz="0.00008"  <!-- 정확한 관성 텐서 -->
           ixy="0" ixz="0" iyz="0"/>
</inertial>
```

**개선 효과**:
- 물리 시뮬레이션 정확도 향상
- 로봇 동역학 모델 현실화
- Isaac Sim PhysX 안정성 증가

---

### ✅ Enhanced Gripper (2-Finger Parallel Jaw)
```xml
<!-- Left Finger (Master) -->
<joint name="gripper_left_finger_joint" type="prismatic">
  <axis xyz="0 1 0"/>
  <limit lower="0.0" upper="0.04" effort="20.0" velocity="0.5"/>
  <dynamics damping="1.0" friction="0.5"/>
</joint>

<!-- Right Finger (Mimic) -->
<joint name="gripper_right_finger_joint" type="prismatic">
  <axis xyz="0 -1 0"/>
  <mimic joint="gripper_left_finger_joint" multiplier="1" offset="0"/>
</joint>
```

**개선 효과**:
- **GRIP rate 0% → 30%+** 예상
- Parallel Jaw 대칭 파지
- 파지력 제어 가능 (effort=20N)
- Contact sensor 부착 위치 명확

---

### ✅ RealSense D455 Camera Integration
```xml
<joint name="camera_joint" type="fixed">
  <parent link="gripper_base_link"/>
  <child link="camera_link"/>
  <!-- 5cm 전방, 2cm 위, 30도 아래 -->
  <origin xyz="0.05 0 0.02" rpy="0 0.524 0"/>
</joint>

<link name="camera_link">
  <inertial>
    <mass value="0.072"/>  <!-- D455 actual weight -->
  </inertial>
  <visual>
    <geometry>
      <box size="0.026 0.124 0.029"/>  <!-- D455 dimensions -->
    </geometry>
  </visual>
</link>

<!-- Optical Frames (ROS REP-103) -->
<link name="camera_depth_optical_frame"/>
<link name="camera_color_optical_frame"/>
```

**Camera Specs** (RealSense D455):
- **Depth range**: 0.4 ~ 6.0m
- **RGB resolution**: 1920x1080 @ 30fps
- **Depth resolution**: 1280x720 @ 90fps
- **FOV**: 87° x 58° (depth), 90° x 65° (RGB)
- **Weight**: 72g

**개선 효과**:
- Vision-based RL 가능
- RGB-D observation (4 channels)
- CNN Policy 학습 지원

---

### ✅ Joint Dynamics (Damping & Friction)
```xml
<joint name="joint_shoulder" type="revolute">
  <dynamics damping="0.7" friction="0.2"/>
</joint>
```

**Damping 값 설정 근거**:
| Joint | Damping | Friction | 이유 |
|-------|---------|----------|------|
| Base | 0.5 | 0.1 | 연속 회전, 중간 damping |
| Shoulder | 0.7 | 0.2 | 무거운 링크, 높은 damping |
| Elbow | 0.6 | 0.15 | 중간 링크 |
| Wrist1 | 0.5 | 0.1 | 가벼운 링크 |
| Wrist2 | 0.4 | 0.08 | 연속 회전, 낮은 damping |
| Gripper | 1.0 | 0.5 | 정밀 제어, 높은 damping |

**개선 효과**:
- 시뮬레이션 떨림 감소
- 에너지 소산 모델링
- 학습 안정성 향상

---

### ✅ Material Definitions
```xml
<material name="black">
  <color rgba="0.2 0.2 0.2 1.0"/>
</material>
<material name="gray">
  <color rgba="0.6 0.6 0.6 1.0"/>
</material>
<material name="aluminum">
  <color rgba="0.8 0.85 0.9 1.0"/>
</material>
```

**개선 효과**:
- 시각화 개선
- URDF Viewer에서 구분 용이

---

## 3. URDF 검증 결과

### check_urdf 출력
```bash
$ check_urdf roarm_m3_enhanced.urdf

robot name is: roarm_m3_enhanced
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  link_base
        child(1):  link_shoulder
            child(1):  link_elbow
                child(1):  link_wrist1
                    child(1):  link_wrist2
                        child(1):  gripper_base_link
                            child(1):  camera_link
                                child(1):  camera_depth_frame
                                    child(1):  camera_color_frame
                                        child(1):  camera_color_optical_frame
                                    child(2):  camera_depth_optical_frame
                            child(2):  gripper_left_finger
                            child(3):  gripper_right_finger
```

✅ **검증 성공!**

---

## 4. Joint 구조 비교

### Original URDF (6 Joints)
```
1. joint_base (continuous)
2. joint_shoulder (revolute, ±90°)
3. joint_elbow (revolute, ±90°)
4. joint_wrist1 (revolute, ±90°)
5. joint_wrist2 (continuous)
6. joint_gripper (revolute, 1.08~3.14 rad)  ← 단일 조인트
```

### Enhanced URDF (7 Joints + Camera)
```
1. joint_base (continuous)
2. joint_shoulder (revolute, ±90°)
3. joint_elbow (revolute, ±90°)
4. joint_wrist1 (revolute, ±90°)
5. joint_wrist2 (continuous)
6. gripper_base_joint (fixed)
7. gripper_left_finger_joint (prismatic, 0~40mm)  ← Master
8. gripper_right_finger_joint (prismatic, mimic)  ← Follower
9. camera_joint (fixed)
10. camera_depth_joint (fixed)
11. camera_depth_optical_joint (fixed)
12. camera_color_joint (fixed)
13. camera_color_optical_joint (fixed)
```

**변화**:
- 6 DOF → 7 DOF (그리퍼 2개 조인트)
- Camera frames 5개 추가

---

## 5. Isaac Sim 통합 계획

### Step 1: URDF Import
```python
from omni.isaac.urdf import _urdf

import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False  # Camera frames 유지
import_config.convex_decomp = False
import_config.import_inertia_tensor = True  # Inertial properties 사용

success, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="assets/roarm_m3/urdf/roarm_m3_enhanced.urdf",
    import_config=import_config,
)
```

### Step 2: Camera Sensor 추가
```python
from omni.isaac.sensor import Camera

camera = Camera(
    prim_path="/World/RoArm_M3/camera_link/Camera",
    frequency=20,  # 20 Hz
    resolution=(256, 256),
)
camera.initialize()

# Get RGBD data
rgb = camera.get_rgba()[:, :, :3]  # (256, 256, 3)
depth = camera.get_depth()  # (256, 256)
```

### Step 3: Mimic Joint 구현
```python
# envs/robot/robot_controller.py
def apply_gripper_action(self, gripper_position):
    # Mimic joint: left = right
    self._articulation.set_joint_positions(
        [gripper_position, gripper_position],
        joint_indices=[6, 7]  # left, right
    )
```

---

## 6. 예상 성능 개선

### Phase 1 (10M 학습 완료 후)
| Metric | Before | After (예상) |
|--------|--------|-------------|
| reach_rate | 18% | 18% (변화 없음) |
| **grip_rate** | **0%** | **30%+** |
| lift_rate | 0% | 10%+ |
| 평균 보상 | 4,775 | 6,000+ |

### Phase 2 (Vision RL 통합 후)
| Metric | Vector Obs | RGB-D Obs (예상) |
|--------|-----------|------------------|
| reach_rate | 18% | 40%+ |
| grip_rate | 30% | 50%+ |
| success_rate | 5% | 20%+ |

---

## 7. URDF Viewer 환경 구축

### 설치 완료
```bash
✅ liburdfdom-tools (check_urdf, urdf_to_graphiz)
🔄 urdf-viz (Rust 기반, 설치 중...)
```

### urdf-viz 사용법 (설치 후)
```bash
# 1. 기본 실행
urdf-viz assets/roarm_m3/urdf/roarm_m3_enhanced.urdf

# 2. Joint 제어 모드
urdf-viz --joint assets/roarm_m3/urdf/roarm_m3_enhanced.urdf

# 3. 충돌 체크 모드
urdf-viz --collision assets/roarm_m3/urdf/roarm_m3_enhanced.urdf
```

**urdf-viz 기능**:
- 3D 시각화 (OpenGL)
- 실시간 Joint 제어 (슬라이더)
- Collision geometry 표시
- 좌표계 (TF) 표시

---

## 8. 다음 단계

### 즉시 (오늘 밤)
1. urdf-viz 설치 완료 대기
2. Enhanced URDF 시각화 확인
3. 10M 학습 완료 (00:48)

### 내일 (10/29)
1. Isaac Sim에서 Enhanced URDF Import
2. Camera Sensor 통합 테스트
3. Gripper 동작 테스트 (mimic joint)

### 이번 주 (10/29-11/1)
1. 그리퍼 Contact Sensor 추가
2. 보상 함수 조정 (GRIP_SUCCESS_REWARD)
3. 초기 학습 실험 (100K steps)

---

## 9. 파일 위치

### Original URDF
```
assets/roarm_m3/urdf/roarm_m3.urdf  (102 lines)
```

### Enhanced URDF
```
assets/roarm_m3/urdf/roarm_m3_enhanced.urdf  (380 lines)
```

### 참고 자료
```
resources/grippers/realsense-ros/realsense2_description/urdf/_d455.urdf.xacro
resources/grippers/robotiq/robotiq_2f_85_gripper_visualization/urdf/
```

---

## 10. 기술적 세부사항

### D455 Camera Extrinsics (실제 값)
```python
# Depth to Color offset (D455 calibration)
depth_to_color_x = -0.059  # -59mm (왼쪽)
depth_to_color_y = 0.0
depth_to_color_z = 0.0

# Camera mount offset
mount_offset_x = 0.0158  # 15.8mm
glass_to_front = 0.0001  # 0.1mm
zero_depth_to_glass = 0.00455  # 4.55mm
```

### Inertia Tensor 계산 (Cylinder 예시)
```python
# link_shoulder: cylinder (r=0.02m, h=0.15m, m=0.4kg)
r = 0.02
h = 0.15
m = 0.4

Ixx = Iyy = (1/12) * m * (3*r**2 + h**2) = 0.00075
Izz = (1/2) * m * r**2 = 0.00008
```

---

**작성일**: 2025-10-28 18:45  
**상태**: URDF 디테일 강화 완료, D455 통합 완료, Viewer 설치 중  
**다음**: Isaac Sim 통합 테스트
