# Gripper and Joint URDF Resources

Phase 2를 위한 그리퍼 및 조인트 URDF 자료 모음입니다.

## 목표
- Parallel Jaw 그리퍼 URDF 모델 수집
- 조인트 구성 및 제약 조건 학습
- Isaac Sim에 통합 가능한 URDF 구조 분석

---

## 1. RealSense Camera Integration (카메라 통합)

### Intel RealSense ROS Wrapper
- **GitHub**: https://github.com/IntelRealSense/realsense-ros
- **설명**: ROS2 Humble/Iron/Jazzy 지원, Intel RealSense D455/D435i 카메라 통합
- **주요 기능**:
  - RGB-D 토픽 제공 (color + depth 동기화)
  - `realsense2_description` 패키지에 URDF 모델 포함
  - Camera info, extrinsics, TF static/dynamic 제공
  - IMU 데이터 통합 (gyro + accel)

### URDF 위치
```
realsense2_description/
├── meshes/          # D455, D435i STL 메시
├── urdf/            # RealSense 카메라 URDF
```

### 통합 방법
1. `package://realsense2_description/urdf/d455.urdf.xacro` 참조
2. RoArm-M3 URDF에 `<joint>` 추가 (end-effector에 카메라 부착)
3. Extrinsics: depth_to_color 변환 행렬 제공
4. Isaac Sim에서 Camera Sensor Prim으로 매핑

**다운로드**:
```bash
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/realsense2_description
```

---

## 2. Robotiq Gripper URDF

### ros-industrial/robotiq
- **GitHub**: https://github.com/ros-industrial/robotiq
- **설명**: Robotiq 2F-85, 2F-140 Parallel Jaw 그리퍼 공식 URDF
- **지원 모델**:
  - `robotiq_2f_85`: 85mm stroke, 최대 파지력 120N
  - `robotiq_2f_140`: 140mm stroke, 최대 파지력 200N
  - `robotiq_3f`: 3-finger adaptive gripper

### URDF 구조
```xml
<!-- Parallel Jaw Gripper 조인트 구조 -->
<joint name="finger_joint" type="prismatic">
  <parent link="gripper_base"/>
  <child link="left_finger"/>
  <limit lower="0.0" upper="0.085" effort="120" velocity="0.2"/>
  <dynamics damping="0.1" friction="0.5"/>
</joint>

<joint name="finger_mimic_joint" type="prismatic">
  <mimic joint="finger_joint" multiplier="1" offset="0"/>
  <parent link="gripper_base"/>
  <child link="right_finger"/>
</joint>
```

**핵심 특징**:
- **Mimic Joint**: 한 조인트가 다른 조인트를 자동으로 따라감 (대칭 파지)
- **Prismatic Joint**: Linear 이동 (0 → 85mm)
- **Effort Limit**: 최대 파지력 제약
- **Contact Geometry**: 손가락 끝에 `<collision>` 태그

**다운로드**:
```bash
git clone https://github.com/ros-industrial/robotiq.git
cd robotiq/robotiq_2f_85_gripper_visualization/meshes
```

---

## 3. Universal Robots URDF (참고용)

### ros-industrial/universal_robot
- **GitHub**: https://github.com/ros-industrial/universal_robot
- **설명**: UR3, UR5, UR10, UR16 로봇팔 URDF (ROS Noetic/Melodic)
- **주요 학습 포인트**:
  - 6-DOF 로봇팔 조인트 구성
  - Revolute Joint 제약 조건 (`<limit>` 태그)
  - MoveIt! 통합 사례
  - Gazebo Simulation 설정

**URDF 예시** (UR5):
```xml
<joint name="shoulder_pan_joint" type="revolute">
  <parent link="base_link"/>
  <child link="shoulder_link"/>
  <origin xyz="0 0 0.089159" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-6.28318530718" upper="6.28318530718"
         effort="150.0" velocity="3.15"/>
  <dynamics damping="0.0" friction="0.0"/>
</joint>
```

**다운로드**:
```bash
git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
cd universal_robot/ur_description/urdf
```

---

## 4. iCub Humanoid Robot URDF (고급 참고)

### robotology/icub-models
- **GitHub**: https://github.com/robotology/icub-models
- **설명**: iCub 휴머노이드 로봇 URDF/SDF 모델
- **주요 학습 포인트**:
  - 복잡한 Multi-body 시스템 (팔 + 다리 + 손)
  - 손가락 조인트 구성 (20+ DOF)
  - Gazebo Classic/Modern 통합

**C++ 라이브러리** (경로 찾기):
```cpp
#include <iCubModels/iCubModels.h>
std::string path = iCubModels::getModelFile("iCubGazeboV2_7");
```

**다운로드**:
```bash
conda install -c conda-forge icub-models
# 또는
git clone https://github.com/robotology/icub-models.git
```

---

## 5. 기타 Gripper 자료

### ROBOTIS Open Manipulator
- **GitHub**: https://github.com/ROBOTIS-GIT/open_manipulator
- **설명**: Open Manipulator-X, 저가형 4-DOF 로봇팔 + 그리퍼
- **그리퍼**: Simple 2-finger gripper (prismatic joint)

### cambel/robotiq
- **GitHub**: https://github.com/cambel/robotiq
- **설명**: Robotiq 2F-85/140 Python 제어 라이브러리
- **주요 기능**: Gripper 제어 인터페이스, Gazebo 시뮬레이션

---

## Isaac Sim 통합 가이드

### URDF → USD 변환
1. **Isaac Sim URDF Importer** 사용:
   ```python
   from omni.isaac.urdf import _urdf
   import_config = _urdf.ImportConfig()
   import_config.merge_fixed_joints = False
   import_config.convex_decomp = False
   import_config.self_collision = False
   
   success, prim_path = omni.kit.commands.execute(
       "URDFParseAndImportFile",
       urdf_path="/path/to/gripper.urdf",
       import_config=import_config,
   )
   ```

2. **Articulation Root** 설정:
   - Gripper Base → ArticulationRoot
   - Finger Joints → RevoluteJoint or PrismaticJoint

3. **Contact Sensors** 추가:
   - 손가락 끝에 ContactSensor Prim
   - `force_threshold = 0.5` (뉴턴)

### Mimic Joint 구현 (Isaac Sim)
Isaac Sim은 URDF의 `<mimic>` 태그를 직접 지원하지 않음.

**해결책 1**: Python Controller
```python
# envs/robot/robot_controller.py
def apply_gripper_action(self, gripper_position):
    self._articulation.set_joint_positions(
        [gripper_position, gripper_position],  # 대칭
        joint_indices=[6, 7]  # left_finger, right_finger
    )
```

**해결책 2**: USD Mimic Joint (PhysX Schema)
```python
# USD에서 mimic relationship 추가
mimic_joint = UsdPhysics.PrismaticJoint.Define(stage, prim_path)
mimic_joint.CreateMimicJointRel().AddTarget("/gripper/left_finger_joint")
```

---

## RoArm-M3 그리퍼 개선 계획 (Week 1-2)

### 현재 문제점
- **GRIP rate = 0%**: 그리퍼 조인트가 파지를 수행하지 못함
- **단순한 조인트**: 1-DOF prismatic joint, 제약 조건 미흡
- **Contact Sensor 없음**: 파지 성공 여부 감지 불가

### 개선 방향
1. **Robotiq 2F-85 스타일로 업그레이드**:
   - 2-finger parallel jaw (mimic joint)
   - Contact sensors on fingertips
   - Force/torque limits

2. **URDF 수정**:
   - `assets/roarm_m3/urdf/roarm_m3.urdf` 업데이트
   - 손가락 메시 추가 (`meshes/gripper/`)
   - Collision geometry 정확히 설정

3. **보상 함수 조정**:
   - `GRIP_SUCCESS_REWARD` 증가 (50 → 100)
   - `MIN_GRASP_FORCE` 임계값 조정 (0.5 → 1.0N)
   - Episode 길이 증가 (max_steps: 300 → 500)

---

## 다운로드 체크리스트

- [ ] Intel RealSense ROS (`realsense-ros`)
- [ ] Robotiq Gripper (`ros-industrial/robotiq`)
- [ ] Universal Robots URDF (`universal_robot`)
- [ ] iCub Models (선택 사항)
- [ ] Open Manipulator (선택 사항)

**다음 단계**:
1. 위 저장소들을 `resources/grippers/` 폴더에 다운로드
2. URDF 파일 분석 (joint 구조, collision, inertia)
3. Isaac Sim에서 URDF Import 테스트
4. RoArm-M3 URDF에 카메라 + 그리퍼 통합

---

**작성일**: 2025-10-28  
**상태**: 리소스 수집 완료, 다운로드 대기
