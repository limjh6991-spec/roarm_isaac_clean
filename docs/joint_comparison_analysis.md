# RoArm-M3 조인트 비교 분석

**작성일**: 2025-10-24  
**목적**: 실제 RoArm-M3 스펙과 URDF 조인트 비교  
**문제**: URDF에 조인트가 한 개 더 많은 것으로 의심됨

---

## 🔍 실제 RoArm-M3 조인트 구조

### 공식 스펙 (Waveshare Wiki + 예제 코드)

```yaml
DOF: 5+1 (5개 관절 + 그리퍼)

조인트 구성:
  1. BASE_JOINT (Joint 1):
      - 기능: Base 360° 회전
      - 범위: -180° ~ +180° (-3.14 ~ 3.14 rad)
      - 축: Z축 (수직)
      - 서보: ST3215/ST3235
      
  2. SHOULDER_JOINT (Joint 2):
      - 기능: Shoulder 상하 움직임
      - 범위: -90° ~ +90° (-1.57 ~ 1.57 rad)
      - 축: Y축 (좌우)
      - 서보: ST3215/ST3235
      
  3. ELBOW_JOINT (Joint 3):
      - 기능: Elbow 굽힘
      - 범위: -90° ~ +90° (-1.57 ~ 1.57 rad)
      - 축: Y축
      - 서보: ST3215/ST3235
      
  4. WRIST_JOINT (Joint 4):
      - 기능: Wrist 굽힘 (손목1)
      - 범위: -90° ~ +90° (-1.57 ~ 1.57 rad)
      - 축: Y축
      - 서보: ST3215/ST3235
      
  5. ROLL_JOINT (Joint 5):
      - 기능: 손목 회전 (손목2)
      - 범위: -180° ~ +180° (-3.14 ~ 3.14 rad)
      - 축: Z축
      - 서보: ST3215/ST3235
      
  6. GRIPPER (EOAT_JOINT):
      - 기능: 그리퍼 개폐
      - 범위: 60° ~ 180° (1.08 ~ 3.14 rad)
      - 타입: 회전식 그리퍼 (parallel jaw)
      - 서보: ST3215/ST3235
```

### 예제 코드 확인

`RoArm-M3_module.h` 함수 정의:
```cpp
void RoArmM3_allJointAbsCtrl(
    double inputBase,      // Joint 1 - BASE
    double inputShoulder,  // Joint 2 - SHOULDER
    double inputElbow,     // Joint 3 - ELBOW
    double inputWrist,     // Joint 4 - WRIST
    double inputRoll,      // Joint 5 - ROLL
    double inputHand,      // Joint 6 - GRIPPER
    u16 inputSpd, 
    u8 inputAcc
)
```

**확인 결과**: 실제 RoArm-M3는 **5개 조인트 + 그리퍼 = 6 DOF**

---

## 🤖 현재 URDF 조인트 구조

### 파일: `roarm_m3_multiprim.urdf`

```xml
1. joint_1 (revolute):
   - Parent: base_link
   - Child: link_1
   - Origin: xyz="0 0 0.06"
   - Axis: xyz="0 0 1" (Z축)
   - Limit: -3.14159 ~ 3.14159
   - 기능: ✅ BASE 회전

2. joint_2 (revolute):
   - Parent: link_1
   - Child: link_2
   - Origin: xyz="0 0 0.08"
   - Axis: xyz="0 1 0" (Y축)
   - Limit: -1.57 ~ 1.57
   - 기능: ✅ SHOULDER

3. joint_3 (revolute):
   - Parent: link_2
   - Child: link_3
   - Origin: xyz="0.16 0 0"
   - Axis: xyz="0 1 0" (Y축)
   - Limit: -1.57 ~ 1.57
   - 기능: ✅ ELBOW

4. joint_4 (revolute):
   - Parent: link_3
   - Child: link_4
   - Origin: xyz="0.15 0 0"
   - Axis: xyz="0 1 0" (Y축)
   - Limit: -3.14159 ~ 3.14159 ⚠️
   - 기능: ✅ WRIST (하지만 범위가 이상함)

5. joint_5 (revolute):
   - Parent: link_4
   - Child: link_5
   - Origin: xyz="0 0 0.06"
   - Axis: xyz="0 0 1" (Z축)
   - Limit: -3.14159 ~ 3.14159
   - 기능: ✅ ROLL

6. joint_6 (revolute): ⚠️⚠️⚠️
   - Parent: link_5
   - Child: gripper_base
   - Origin: xyz="0 0 0.065"
   - Axis: xyz="0 0 1" (Z축)
   - Limit: -1.57 ~ 1.57
   - 기능: ❓❓❓ 불명확

7. gripper_left_joint (prismatic):
   - Parent: gripper_base
   - Child: gripper_left_finger
   - Axis: xyz="0 1 0"
   - Limit: 0 ~ 0.025
   - 기능: ✅ 왼쪽 그리퍼

8. gripper_right_joint (prismatic):
   - Parent: gripper_base
   - Child: gripper_right_finger
   - Axis: xyz="0 -1 0"
   - Limit: 0 ~ 0.025
   - 기능: ✅ 오른쪽 그리퍼
```

**URDF 조인트 개수**: 
- 팔 조인트: 6개 (joint_1 ~ joint_6)
- 그리퍼: 2개 (gripper_left, gripper_right)
- **합계**: 8개

---

## 🚨 문제점 발견

### 1. joint_6의 정체 불명

```yaml
의심되는 점:
  - 실제 RoArm-M3에는 joint_5 이후 그리퍼만 있음
  - URDF의 joint_6은 gripper_base와 link_5 사이에 존재
  - joint_6의 기능이 불명확 (Z축 회전, -90°~+90°)
  
가능성 1: joint_6 = 중복된 회전 조인트 (제거 필요)
  - 실제로는 link_5에서 바로 gripper_base로 연결
  - joint_6이 추가 DOF를 만들어냄 (불필요)
  
가능성 2: joint_6 = 그리퍼 회전 메커니즘
  - 하지만 실제 RoArm-M3 그리퍼는 회전하지 않음
  - 단순 개폐식 그리퍼
  
가능성 3: joint_5와 joint_6 역할 혼동
  - joint_5: 손목 회전 (ROLL) ✅
  - joint_6: 불필요한 조인트 ❌
```

### 2. 실제 vs URDF 비교표

| 실제 RoArm-M3 | URDF | 일치 여부 |
|---------------|------|----------|
| BASE (joint_1) | joint_1 | ✅ 일치 |
| SHOULDER (joint_2) | joint_2 | ✅ 일치 |
| ELBOW (joint_3) | joint_3 | ✅ 일치 |
| WRIST (joint_4) | joint_4 | ⚠️ 범위 차이 |
| ROLL (joint_5) | joint_5 | ✅ 일치 |
| GRIPPER (회전식) | joint_6 | ❌ **불일치** |
| - | gripper_left_joint | ✅ 추가됨 (prismatic) |
| - | gripper_right_joint | ✅ 추가됨 (prismatic) |

### 3. joint_4 범위 문제

```yaml
실제 RoArm-M3 WRIST:
  - 범위: -90° ~ +90° (-1.57 ~ 1.57 rad)
  
URDF joint_4:
  - 범위: -180° ~ +180° (-3.14 ~ 3.14 rad) ⚠️
  - 문제: 실제보다 2배 넓음
  - 영향: 학습 중 불가능한 자세 탐색
```

---

## 💡 해결 방안

### 옵션 1: joint_6 제거 (권장) ✅

```xml
<!-- 변경 전 -->
<joint name="joint_5" ...>
  <child link="link_5"/>
</joint>

<joint name="joint_6" type="revolute">  ← 제거
  <parent link="link_5"/>
  <child link="gripper_base"/>
  ...
</joint>

<!-- 변경 후 -->
<joint name="joint_5" ...>
  <child link="link_5_with_gripper_base"/>  ← link_5와 gripper_base 합침
</joint>

OR

<joint name="gripper_mount" type="fixed">  ← revolute → fixed
  <parent link="link_5"/>
  <child link="gripper_base"/>
  ...
</joint>
```

**장점**:
- 실제 RoArm-M3와 일치 (5+1 DOF)
- 불필요한 학습 차원 제거
- 제어 복잡도 감소

**단점**:
- URDF 재작성 필요
- 기존 학습 모델 호환 불가

### 옵션 2: joint_6을 fixed로 변경 ⚠️

```xml
<joint name="joint_6" type="fixed">  ← revolute → fixed
  <parent link="link_5"/>
  <child link="gripper_base"/>
  <origin xyz="0 0 0.065" rpy="0 0 0"/>
</joint>
```

**장점**:
- URDF 구조 최소 변경
- link 계층 유지

**단점**:
- 불필요한 link 여전히 존재
- 환경 코드에서 joint_6 무시 필요

### 옵션 3: joint_4 범위 수정 ✅

```xml
<!-- 변경 전 -->
<joint name="joint_4" type="revolute">
  <limit lower="-3.14159" upper="3.14159" ... />
</joint>

<!-- 변경 후 -->
<joint name="joint_4" type="revolute">
  <limit lower="-1.57" upper="1.57" ... />  ← ±90°로 제한
</joint>
```

---

## 📊 환경 코드 영향 분석

### 현재 코드: `roarm_pick_place_env.py`

```python
# 🔥 v3.8.1: 어느 조인트를 제어하고 있는가?

# 학습 시 사용되는 조인트
dof_names = [
    "joint_1",  # BASE
    "joint_2",  # SHOULDER  
    "joint_3",  # ELBOW
    "joint_4",  # WRIST (범위 문제)
    "joint_5",  # ROLL
    "joint_6",  # ❓ 이게 뭔가?
    "gripper_left_joint",   # 왼쪽 그리퍼
    "gripper_right_joint"   # 오른쪽 그리퍼
]

# Action Space: 7차원
# [joint_1_vel, joint_2_vel, ..., joint_6_vel, gripper_cmd]
```

### 문제점

```yaml
현재 상태:
  - Action space: 7차원
  - 실제 제어: 8개 조인트 (joint_1~6 + 그리퍼 좌우)
  
의문:
  - Action 7차원이면 joint_6을 어떻게 제어?
  - 그리퍼 좌우를 1개 명령으로 제어? (대칭 제어)
  
확인 필요:
  - Articulation API가 어느 조인트를 제어하는지
  - joint_6이 실제로 움직이는지
  - 그리퍼 제어 방식 (대칭 vs 독립)
```

---

## 🔬 검증 계획

### 1단계: 조인트 움직임 확인

```python
# 스크립트: scripts/rl/check_joint_movement.py

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation

def check_joints():
    world = World()
    world.scene.add_default_ground_plane()
    
    # 로봇 로드
    robot = world.scene.add(
        Articulation(
            prim_path="/World/roarm",
            name="roarm",
            urdf_path="assets/roarm_m3/urdf/roarm_m3_multiprim.urdf"
        )
    )
    
    # 모든 조인트 이름 출력
    print("\n📋 DOF 목록:")
    dof_names = robot.dof_names
    for i, name in enumerate(dof_names):
        print(f"  {i}: {name}")
    
    # joint_6 움직임 테스트
    print("\n🔧 joint_6 테스트:")
    for angle in [-1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5]:
        robot.set_joint_positions(
            positions=np.array([0, 0, 0, 0, 0, angle, 0, 0])
        )
        world.step()
        print(f"  joint_6 = {angle:.2f} rad")
        input("  Press Enter...")

if __name__ == "__main__":
    check_joints()
```

### 2단계: 실제 RoArm-M3 동작 비교

```yaml
확인사항:
  1. RoArm-M3 실물에서 joint_6에 해당하는 움직임 있는지
  2. 그리퍼 회전 vs 개폐 메커니즘 확인
  3. 예제 코드에서 joint_6 제어 유무
```

### 3단계: URDF 수정 및 검증

```yaml
작업:
  1. joint_6 → fixed 변경
  2. joint_4 범위 수정 (-180° → -90°~+90°)
  3. Isaac Sim 임포트 테스트
  4. 학습 환경 동작 확인
```

---

## 🎯 최종 권장사항

### 즉시 수행할 작업

1. **joint_6 성격 파악** ⚠️ 최우선
   - `check_joint_movement.py` 실행
   - GUI에서 joint_6 움직임 관찰
   - 실제 로봇과 비교

2. **joint_4 범위 수정** ✅ 확실함
   ```xml
   <limit lower="-1.57" upper="1.57" ... />
   ```

3. **joint_6 처리 결정**
   - 불필요 → fixed 변경
   - 필요 → 문서화 및 설명 추가

### 학습 재개 전 필수 작업

```yaml
우선순위 1: joint_6 문제 해결
  - 현재 학습은 계속 진행 (5K / 300K)
  - 학습 완료 후 joint_6 검증
  - 다음 학습 전 URDF 수정
  
우선순위 2: joint_4 범위 수정
  - v3.8.2 환경 생성
  - joint_4: -3.14 → -1.57 / +3.14 → +1.57
  
우선순위 3: 문서화
  - URDF 조인트 매핑 문서 작성
  - 실제 로봇 대응표 작성
```

---

## 📚 참고 자료

### 실제 RoArm-M3 예제 코드

```cpp
// RoArm-M3_module.h

// Joint control functions
RoArmM3_baseJointCtrlRad(...)      // Joint 1
RoArmM3_shoulderJointCtrlRad(...)  // Joint 2
RoArmM3_elbowJointCtrlRad(...)     // Joint 3
RoArmM3_wristJointCtrlRad(...)     // Joint 4
RoArmM3_rollJointCtrlRad(...)      // Joint 5
RoArmM3_handJointCtrlRad(...)      // Gripper (회전식)

// ⚠️ 총 6개 조인트 (5+1)
// ⚠️ URDF의 joint_6에 해당하는 함수 없음
```

### 관련 파일

```
URDF: assets/roarm_m3/urdf/roarm_m3_multiprim.urdf
환경: envs/roarm_pick_place_env.py
예제: resources/roarm_m3/RoArm-M3_example-250108/
스펙: resources/roarm_m3/waveshare_wiki_summary.md
```

---

## 🤝 AI 검증 요청

**여러 AI 시스템에게 질문:**

1. **joint_6의 정체는?**
   - URDF에만 존재하는 불필요한 조인트인가?
   - 실제 로봇에 숨겨진 DOF가 있는가?
   - 그리퍼 회전 메커니즘과 관련 있는가?

2. **URDF 수정 방향은?**
   - joint_6 제거 vs fixed 변경
   - link 계층 재구성 필요성
   - 기존 학습 영향 최소화 방법

3. **실제 로봇 전이 가능성은?**
   - 8-DOF URDF로 학습한 정책을 6-DOF 로봇에 적용 가능한가?
   - joint_6 명령을 무시하면 문제 없는가?
   - 재학습 필요 여부

---

**작성자**: GitHub Copilot  
**검증 요청**: 조인트 구조 전문가, 로보틱스 엔지니어  
**업데이트 예정**: joint_6 검증 완료 후 결론 추가
