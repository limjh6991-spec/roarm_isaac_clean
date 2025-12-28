# IsaacLab Articulation 제어 완전 가이드

## 📚 개요

IsaacLab 0.47.5에서 Articulation(로봇) 제어를 위한 완전 가이드입니다. Isaac Sim 5.0의 새로운 API를 반영합니다.

---

## 🎯 핵심 개념

### 1. Actuator 모델

IsaacLab은 두 가지 actuator 모델을 제공합니다:

#### **Implicit Actuator** (암시적)
- Physics engine이 제공하는 이상적인 시뮬레이션 메커니즘
- 내부적으로 PD 컨트롤러 구현
- 수치 안정성이 높음 (내부 damping)
- **특징**: `set_joint_position_target()` 후 `write_data_to_sim()` 호출 시 자동으로 토크 계산

#### **Explicit Actuator** (명시적)
- 사용자가 구현한 외부 drive 모델
- 더 현실적인 모터 동작 모사
- `IdealPDActuator`, `DCMotor`, `ANYdrive` 등 제공
- **주의**: `armature` 파라미터로 수치 안정성 확보 필요

---

## 🔧 주요 메서드

### Joint State 직접 쓰기 (즉시 적용)

```python
# 1. Joint Position과 Velocity 동시 설정
robot.write_joint_state_to_sim(
    position=torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]]),
    velocity=torch.zeros((1, 6)),
    joint_ids=None,  # All joints
    env_ids=None      # All environments
)

# 2. Joint Position만 설정
robot.write_joint_position_to_sim(
    position=torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]]),
)

# 3. Joint Velocity만 설정
robot.write_joint_velocity_to_sim(
    velocity=torch.zeros((1, 6)),
)
```

**특징**:
- ✅ **즉시 적용**: 다음 `sim.step()` 시 즉시 반영
- ✅ **물리 엔진 우회**: Actuator 모델을 거치지 않음
- ⚠️ **중력 무시 안됨**: 자세 설정 후 중력으로 인해 변할 수 있음

---

### Joint Target 설정 (Actuator를 통해 적용)

```python
# 1. Position Target 설정 (버퍼에만 저장)
robot.set_joint_position_target(
    target=torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]]),
)

# 2. Actuator를 통해 시뮬레이션에 적용
robot.write_data_to_sim()  # ← 이것이 실제 적용!

# 3. 시뮬레이션 스텝
sim.step()

# 4. 로봇 데이터 업데이트
robot.update(dt=sim_cfg.dt)
```

**동작 원리**:
1. `set_joint_position_target()`: `robot._data.joint_pos_target`에 저장
2. `write_data_to_sim()`:
   - `_apply_actuator_model()` 호출
   - Actuator가 target과 현재 state 비교
   - PD 제어로 토크 계산: `τ = kp*(q_des - q) + kd*(dq_des - dq)`
   - 계산된 토크를 PhysX에 설정
3. `sim.step()`: Physics engine이 토크를 적용하여 로봇 움직임

**특징**:
- ✅ **Actuator 모델 적용**: PD 제어, 토크 제한, damping 등
- ✅ **현실적**: 실제 로봇 동작과 유사
- ⚠️ **지연**: Target에 도달하는데 시간 소요

---

### Joint Properties 설정

```python
# Stiffness (강성) 설정
robot.write_joint_stiffness_to_sim(
    stiffness=torch.tensor([[400.0, 1000.0, 600.0, 500.0, 400.0, 250.0]]),
)

# Damping (감쇠) 설정
robot.write_joint_damping_to_sim(
    damping=torch.tensor([[8.0, 20.0, 15.0, 12.0, 8.0, 5.0]]),
)

# Armature (관성 보정) 설정
robot.write_joint_armature_to_sim(
    armature=torch.tensor([[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]]),
)

# Joint Limits 설정
robot.write_joint_position_limit_to_sim(
    limits=torch.tensor([
        [[-1.57, 1.57],    # Joint 0
         [-1.57, 1.57],    # Joint 1
         [-1.0, 2.95],     # Joint 2
         [-1.57, 1.57],    # Joint 3
         [-3.14, 3.14],    # Joint 4
         [0.0, 1.5]]       # Joint 5
    ]),
)
```

---

## 💡 실전 사용 패턴

### 패턴 1: 초기 자세 설정 (Reset 시)

```python
def reset_robot_to_initial_pose(robot, sim):
    # 1. Root pose 설정 (base link 위치)
    root_state = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(root_state[:, 7:])
    
    # 2. Joint state 직접 설정 (즉시 적용!)
    joint_pos = torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=robot.device)
    joint_vel = torch.zeros((1, 6), device=robot.device)
    robot.write_joint_state_to_sim(joint_pos, joint_vel)
    
    # 3. Actuator target도 동일하게 설정 (유지를 위해)
    robot.set_joint_position_target(joint_pos)
    robot.write_data_to_sim()
    
    # 4. Reset robot buffers
    robot.reset()
    
    # 5. 몇 프레임 안정화
    for _ in range(10):
        sim.step()
        robot.update(dt=sim.get_physics_dt())
```

**핵심**:
- ✅ `write_joint_state_to_sim()`: 즉시 자세 설정
- ✅ `set_joint_position_target()`: Actuator가 그 자세 유지
- ✅ 안정화 프레임: 중력과 물리 안정화

---

### 패턴 2: 동적 제어 (Simulation Loop 내)

```python
def control_loop(robot, sim):
    # 목표 각도
    target_joint_pos = torch.tensor([[1.57, 0.5, -1.0, 0.0, 0.0, 0.0]], device=robot.device)
    
    # Target 설정 (버퍼에만 저장)
    robot.set_joint_position_target(target_joint_pos)
    
    # Actuator를 통해 적용
    robot.write_data_to_sim()
    
    # Simulation step
    sim.step()
    
    # 로봇 상태 업데이트
    robot.update(dt=sim.get_physics_dt())
    
    # 현재 상태 확인
    current_pos = robot.data.joint_pos
    error = target_joint_pos - current_pos
    print(f"Position error: {error}")
```

---

### 패턴 3: Effort (Torque) 직접 제어

```python
def torque_control(robot, sim):
    # 토크 직접 설정
    efforts = torch.randn_like(robot.data.joint_pos) * 5.0
    robot.set_joint_effort_target(efforts)
    
    # 시뮬레이션에 적용
    robot.write_data_to_sim()
    sim.step()
    robot.update(dt=sim.get_physics_dt())
```

---

## 🔍 문제 해결

### 문제 1: 로봇이 초기 자세를 유지하지 못함

**증상**:
```python
# init_state에서 설정
joint_pos={"link2_to_link3": -0.9}  # 설정
# 하지만 실제로는
actual_pos = robot.data.joint_pos[0, 2]  # 2.48! (완전 반대)
```

**원인**:
- `ArticulationCfg.init_state.joint_pos`는 **초기 spawn 위치만** 설정
- Actuator에 target이 없으면 **중력에 의해 자유낙하**
- 로봇이 평형 상태(펴진 자세)에 도달

**해결**:
```python
# Reset 후 명시적으로 자세 설정 + target 설정
def proper_reset(robot, sim):
    # 1. 직접 자세 설정 (즉시)
    desired_pos = torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=robot.device)
    robot.write_joint_state_to_sim(desired_pos, torch.zeros_like(desired_pos))
    
    # 2. Actuator target도 설정 (유지)
    robot.set_joint_position_target(desired_pos)
    robot.write_data_to_sim()
    
    # 3. Reset
    robot.reset()
    
    # 4. 안정화
    for _ in range(20):
        sim.step()
        robot.update(dt=sim.get_physics_dt())
```

---

### 문제 2: `set_joint_position_target()`이 작동하지 않음

**증상**:
```python
robot.set_joint_position_target(target)
# Joint가 전혀 움직이지 않음
```

**원인**:
- `set_joint_position_target()`은 **버퍼에만** 저장
- `write_data_to_sim()` 호출 안함

**해결**:
```python
robot.set_joint_position_target(target)
robot.write_data_to_sim()  # ← 이것이 핵심!
sim.step()
robot.update(dt)
```

---

### 문제 3: Stiffness가 너무 낮아서 로봇이 처짐

**증상**:
- 카메라 등 추가 하중으로 팔이 처짐
- 목표 자세에 도달하지 못함

**해결**:
```python
# 1. Stiffness 증가
robot.write_joint_stiffness_to_sim(
    stiffness=torch.tensor([[400.0, 1000.0, 600.0, 500.0, 400.0, 250.0]]),
)

# 2. Damping 적절히 증가 (Critically damped)
# damping = 2 * sqrt(inertia * stiffness)
robot.write_joint_damping_to_sim(
    damping=torch.tensor([[8.0, 20.0, 15.0, 12.0, 8.0, 5.0]]),
)

# 3. Armature 추가 (수치 안정성)
robot.write_joint_armature_to_sim(
    armature=torch.tensor([[0.05, 0.05, 0.05, 0.05, 0.05, 0.05]]),
)
```

**계산 공식**:
```python
# 실제 서보 토크 기반
servo_torque = 2.94  # Nm (ST3215/ST3235)
acceptable_deflection = 0.0175  # rad (1도)
safety_factor = 2.0

stiffness = (servo_torque / acceptable_deflection) * safety_factor
# = 336 Nm/rad

# Critically damped
inertia = 0.001  # kg⋅m² (estimated)
damping = 2 * sqrt(inertia * stiffness)
# = 2 * sqrt(0.001 * 336) = 1.16
```

---

## 📖 참고 자료

### IsaacLab 공식 문서
- Actuators: `~/IsaacLab/docs/source/overview/core-concepts/actuators.rst`
- Articulation API: `~/IsaacLab/source/isaaclab/isaaclab/assets/articulation/`

### 튜토리얼
- Run Articulation: `~/IsaacLab/scripts/tutorials/01_assets/run_articulation.py`

### 테스트 코드
- Implicit Actuator Test: `~/IsaacLab/source/isaaclab/test/actuators/test_implicit_actuator.py`
- Articulation Test: `~/IsaacLab/source/isaaclab/test/assets/test_articulation.py`

---

## 🎓 베스트 프랙티스

### 1. 항상 `write_data_to_sim()` 호출
```python
# ❌ 잘못된 방법
robot.set_joint_position_target(target)
sim.step()

# ✅ 올바른 방법
robot.set_joint_position_target(target)
robot.write_data_to_sim()  # ← 필수!
sim.step()
robot.update(dt)
```

### 2. Reset 후 자세 명시적 설정
```python
# ❌ init_state만 믿는 방법
cfg = ArticulationCfg(
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={"joint": -0.9}  # 이것만으로는 유지 안됨!
    )
)

# ✅ Reset 후 명시적 설정
robot.reset()
robot.write_joint_state_to_sim(desired_pos, desired_vel)
robot.set_joint_position_target(desired_pos)
robot.write_data_to_sim()
```

### 3. Actuator 파라미터 튜닝
```python
# Stiffness: 실제 서보 토크 기반
# Damping: Critically damped
# Armature: 수치 안정성 (0.01~0.1)

actuators={
    "arm": ImplicitActuatorCfg(
        joint_names_expr=[".*"],
        stiffness={
            "joint1": 400.0,   # 토크 기반 계산
            "joint2": 1000.0,  # Shoulder (높은 하중)
        },
        damping={
            "joint1": 8.0,     # 2*sqrt(I*K)
            "joint2": 20.0,    # 강화
        },
        armature=0.05,  # 수치 안정성
    )
}
```

---

## 🚀 요약

| 메서드 | 용도 | 적용 시점 | Actuator |
|--------|------|----------|----------|
| `write_joint_state_to_sim()` | 직접 자세 설정 | 즉시 | 우회 |
| `set_joint_position_target()` | Target 설정 | `write_data_to_sim()` 후 | 통과 |
| `write_data_to_sim()` | Actuator 적용 | 매 스텝 | 적용 |
| `sim.step()` | 물리 시뮬레이션 | 매 스텝 | - |
| `robot.update(dt)` | 데이터 업데이트 | 매 스텝 | - |

**핵심 흐름**:
```
초기화: write_joint_state_to_sim() + set_joint_position_target()
  ↓
루프: set_joint_position_target() → write_data_to_sim() → sim.step() → update()
```

---

생성일: 2025-11-02
IsaacLab 버전: 0.47.5
Isaac Sim 버전: 5.0
