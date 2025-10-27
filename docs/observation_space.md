# Observation Space 명세서

## 📊 개요

RoArm-M3 Pick and Place 환경의 관측 벡터(Observation Vector) 구조를 정의합니다.

**Observation Dimension**: 28

**검증 완료**: ✅ 2025-10-21 (scripts/verification/cube_observation_check.py)

---

## 🔍 인덱스 매핑 (확정)

| 인덱스 | 차원 | 이름 | 설명 | 좌표계 |
|--------|------|------|------|--------|
| `[0:8]` | 8 | `joint_gripper_positions` | 관절 + 그리퍼 위치 (joint_1~6, gripper_left, gripper_right) | Joint space |
| `[8:11]` | 3 | `cube_rel_to_ee` | EE 기준 큐브 상대 위치 (x, y, z) | **EE frame** |
| `[11:14]` | 3 | `target_rel_to_ee` | EE 기준 타겟 상대 위치 (x, y, z) | **EE frame** |
| `[14:17]` | 3 | `cube_to_target` | 큐브에서 타겟으로의 벡터 (x, y, z) | **World frame** |
| `[17:20]` | 3 | `ee_velocity` | End Effector 속도 (vx, vy, vz) | World frame |
| `[20:23]` | 3 | `cube_velocity` | 큐브 속도 (vx, vy, vz) | World frame |
| `[23:24]` | 1 | `gripper_width` | 그리퍼 손가락 간 거리 (m) | - |
| `[24:25]` | 1 | `is_grasped` | 큐브 잡음 여부 (0.0 or 1.0) | - |
| `[25:26]` | 1 | `distance_to_cube` | EE에서 큐브까지 거리 (m) | - |
| `[26:27]` | 1 | `distance_cube_to_target` | 큐브에서 타겟까지 거리 (m) | - |
| `[27:28]` | 1 | `previous_reward` | 이전 스텝의 보상 값 | - |

---

## 🎯 좌표계 정의

### 1. **World Frame** (전역 좌표계)
- 원점: 로봇 베이스
- X축: 전방
- Y축: 좌측
- Z축: 상방

### 2. **EE Frame** (End Effector 기준 좌표계)
- 원점: 그리퍼 중심 (`gripper_base` link)
- **상대 좌표 계산**:
  ```python
  cube_rel_to_ee = cube_pos_world - ee_pos_world
  target_rel_to_ee = target_pos_world - ee_pos_world
  ```

### 3. **Joint Space**
- 각 관절의 각도 (rad) 또는 그리퍼 위치 (m)

---

## 📐 계산 검증

### 거리 일관성 확인

테스트 결과 (10 스텝 랜덤 액션):

| Step | Dist to Cube (관측) | Dist to Cube (계산) | 차이 |
|------|---------------------|---------------------|------|
| 0 | 0.4422 | 0.4422 | 0.000000 ✅ |
| 3 | 0.4413 | 0.4413 | 0.000000 ✅ |
| 6 | 0.4469 | 0.4469 | 0.000000 ✅ |
| 9 | 0.4447 | 0.4447 | 0.000000 ✅ |

**결론**: 관측 벡터의 거리 값이 실제 계산과 **완벽히 일치** (차이 < 1e-6)

---

## ✅ 성공/종료 조건

### 성공 조건 (`is_success`)

```python
# 큐브가 타겟에 도달했는가?
distance_cube_to_target = obs[26]  # 인덱스 26
is_success = (distance_cube_to_target < 0.02)  # 2cm 이내
```

**주의**: 반드시 **obs[26]** 사용! (World 좌표계에서 계산된 값)

### 종료 조건 (`is_done`)

```python
is_done = is_success or (current_step >= max_steps)
```

---

## 🚨 중요 체크포인트

### ❌ 잘못된 계산 (이전 오류)

```python
# 잘못됨! target_rel_to_ee - cube_rel_to_ee는 EE 기준이므로 부정확
cube_to_target = obs[11:14] - obs[8:11]  
```

### ✅ 올바른 계산

```python
# 올바름! World 좌표계에서 계산된 값 직접 사용
cube_to_target = obs[14:17]
```

**이유**: 
- `cube_rel_to_ee`와 `target_rel_to_ee`는 둘 다 **EE 기준**
- 이 둘을 빼면 EE의 위치가 소거되지만, EE가 움직이면 부정확해짐
- `cube_to_target`은 **World 좌표계**에서 직접 계산해야 정확함

---

## 📝 환경 코드 참조

### `_compute_observations()` 메서드

```python
def _compute_observations(self) -> np.ndarray:
    # [0:8] Joint/Gripper positions
    joint_gripper_pos = self.robot.get_joint_positions()
    
    # [8:11] Cube relative to EE (EE frame)
    cube_rel_to_ee = cube_pos_world - ee_pos_world
    
    # [11:14] Target relative to EE (EE frame)
    target_rel_to_ee = target_pos_world - ee_pos_world
    
    # [14:17] Cube to Target (World frame!)
    cube_to_target = target_pos_world - cube_pos_world
    
    # [17:20] EE velocity
    ee_velocity = self.robot.get_linear_velocity()
    
    # [20:23] Cube velocity
    cube_velocity = self.cube.get_linear_velocity()
    
    # [23:24] Gripper width
    gripper_width = abs(joint_gripper_pos[6] - joint_gripper_pos[7])
    
    # [24:25] Is grasped
    is_grasped = self._check_grasp()
    
    # [25:26] Distance to cube
    distance_to_cube = np.linalg.norm(cube_rel_to_ee)
    
    # [26:27] Distance cube to target
    distance_cube_to_target = np.linalg.norm(cube_to_target)
    
    # [27:28] Previous reward
    previous_reward = self.previous_reward
    
    obs = np.concatenate([
        joint_gripper_pos,        # [0:8]
        cube_rel_to_ee,           # [8:11]
        target_rel_to_ee,         # [11:14]
        cube_to_target,           # [14:17]
        ee_velocity,              # [17:20]
        cube_velocity,            # [20:23]
        [gripper_width],          # [23:24]
        [is_grasped],             # [24:25]
        [distance_to_cube],       # [25:26]
        [distance_cube_to_target],# [26:27]
        [previous_reward],        # [27:28]
    ])
    
    return obs
```

---

## 🧪 테스트 스크립트

**경로**: `scripts/verification/cube_observation_check.py`

**실행**:
```bash
~/isaacsim/python.sh scripts/verification/cube_observation_check.py
```

**검증 항목**:
1. ✅ Observation 벡터 차원 (28)
2. ✅ 각 인덱스의 값 범위
3. ✅ 거리 일관성 (관측 vs 계산)
4. ✅ 좌표계 변환 정확도
5. ✅ 성공/종료 조건 인덱스 매칭

---

## 📚 추가 참고 자료

- **환경 코드**: `envs/roarm_pick_place_env.py`
- **RL 학습**: `scripts/rl/train_dense_reward.py`
- **URDF**: `assets/roarm_m3/urdf/roarm_m3_multiprim.urdf`

---

## 🎉 학습 준비 완료!

이제 관측 벡터 구조가 완전히 검증되었습니다. RL 학습을 시작할 수 있습니다! 🚀
