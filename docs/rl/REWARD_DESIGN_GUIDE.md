# 보상 함수 설계 가이드

**목적**: 효과적인 보상 함수 설계 원칙 및 예제

---

## 🎯 핵심 원칙

### 1. Dense vs Sparse

**❌ Sparse만 사용** (학습 매우 어려움):
```python
reward = 100 if success else 0
```
- 문제: 성공 전까지 학습 신호 없음
- 결과: Random exploration만 계속

**✅ Shaped Reward** (Dense + Sparse):
```python
# Dense guidance
reward = -0.1 * distance_to_goal

# Sparse bonus
if success:
    reward += 100
```
- 장점: 매 step 학습 신호
- 결과: 빠른 수렴

---

### 2. 보상 스케일

**적절한 범위**: -10 ~ +100

**❌ 너무 큼**:
```python
reward = -1000 * distance + 10000 * success
```
- 문제: Gradient 폭발

**✅ 적절한 스케일**:
```python
reward = -0.1 * distance + 100 * success
```

---

### 3. 1회성 보상 (One-time Reward)

**❌ 매 step 지급**:
```python
if grasped:
    reward += 10  # 매번 10점!
```
- 문제: Over-rewarding
- 결과: 그립 상태만 유지

**✅ 첫 달성 시만**:
```python
if grasped and not self.first_grasp:
    reward += 10
    self.first_grasp = True
```

---

### 4. 다단계 마일스톤

**목표**: REACH → GRIP → LIFT → GOAL

```python
# Phase 1: Reaching
if ee_to_cube < 0.05 and not reached:
    reward += 5
    reached = True

# Phase 2: Grasping
if grasped and not gripped:
    reward += 10
    gripped = True

# Phase 3: Lifting
if lifted and not lifted_flag:
    reward += 15
    lifted_flag = True

# Phase 4: Goal
if at_goal and not goal_flag:
    reward += 20
    goal_flag = True

# Success
if stable_at_goal:
    reward += 100
```

---

## 📐 RoArm M3 보상 구조 (현재 구현)

### 전체 보상 함수

```python
def compute_reward(self):
    reward = 0.0
    
    # 1. REACH (+5): EE → 큐브 5cm
    ee_to_cube_dist = np.linalg.norm(ee_pos - cube_pos)
    if ee_to_cube_dist < 0.05 and not self.first_reach:
        reward += 5.0
        self.first_reach = True
        self.milestone_count['reach'] += 1
    
    # 2. GRIP (+10): 유효 그립 3프레임
    grasp_valid = (
        ee_to_cube_dist < 0.08 and
        gripper_width < 0.02 and
        cube_pos[2] > 0.03
    )
    
    if grasp_valid:
        self.grip_frames += 1
    else:
        self.grip_frames = 0
    
    if self.grip_frames >= 3 and not self.valid_grip:
        reward += 10.0
        self.valid_grip = True
        self.milestone_count['grip'] += 1
    
    # 3. LIFT (+15): 큐브 5cm 상승
    cube_height = cube_pos[2] - self.initial_cube_height
    if cube_height > 0.05 and not self.lifted:
        reward += 15.0
        self.lifted = True
        self.milestone_count['lift'] += 1
    
    # 4. GOAL (+20): 큐브 → 타겟 8cm
    cube_to_goal_dist = np.linalg.norm(cube_pos - goal_pos)
    if cube_to_goal_dist < 0.08 and not self.goal_near:
        reward += 20.0
        self.goal_near = True
        self.milestone_count['goal'] += 1
    
    # 5. SUCCESS (+100): 타겟 5cm, 5프레임
    if cube_to_goal_dist < 0.05:
        self.success_frames += 1
    else:
        self.success_frames = 0
    
    if self.success_frames >= 5:
        reward += 100.0
        self.milestone_count['success'] += 1
        done = True
    
    # 6. Time penalty
    reward -= 0.01
    
    return reward, done
```

---

## 🔧 문제별 해결책

### 문제 1: REACH 달성 안됨

**원인**: 거리 임계값 너무 작음

**해결**:
```python
# Before
if ee_to_cube < 0.05:  # 5cm (너무 가까움)
    
# After
if ee_to_cube < 0.08:  # 8cm (완화)
```

---

### 문제 2: GRIP 달성 안됨 (현재 문제!)

**원인**: 조건이 너무 엄격

**현재 조건**:
```python
grasp_valid = (
    ee_to_cube_dist < 0.08 and  # 8cm
    gripper_width < 0.02 and    # 거의 닫힘
    cube_pos[2] > 0.03          # 3cm 높이
)
```

**개선안 1: 조건 완화**:
```python
grasp_valid = (
    ee_to_cube_dist < 0.10 and  # 10cm (완화)
    gripper_width < 0.03 and    # 3cm (완화)
    cube_pos[2] > 0.02          # 2cm (완화)
)
```

**개선안 2: Contact force 추가**:
```python
grasp_valid = (
    ee_to_cube_dist < 0.08 and
    contact_force > 0.1 and      # Contact 감지
    cube_pos[2] > 0.02
)
```

---

### 문제 3: 로봇이 특정 동작만 반복

**원인**: 보상이 로컬 최적에 갇힘

**해결**: Shaped reward 추가
```python
# Dense guidance 추가
reach_reward = -0.1 * ee_to_cube_dist
lift_reward = 5.0 * max(0, cube_height)
goal_reward = -0.2 * cube_to_goal_dist

reward += reach_reward + lift_reward + goal_reward
```

---

## 📊 보상 비율 분석

### 이상적인 비율

| 마일스톤 | 보상 | 비율 | 난이도 |
|---------|------|------|--------|
| REACH | 5 | 3% | 쉬움 |
| GRIP | 10 | 7% | 중간 |
| LIFT | 15 | 10% | 중간 |
| GOAL | 20 | 13% | 어려움 |
| SUCCESS | 100 | 67% | 매우 어려움 |

**총합**: 150점

---

### 현재 100K steps 달성률

| 마일스톤 | 달성 횟수 | 달성률 |
|---------|----------|--------|
| REACH | 12 | 0.012% |
| GRIP | 0 | 0% ❌ |
| LIFT | 0 | 0% |
| GOAL | 0 | 0% |
| SUCCESS | 0 | 0% |

**분석**: REACH만 가끔 달성, GRIP부터 막힘

---

## 💡 디버깅 팁

### 1. 보상 로그 추가

```python
def compute_reward(self):
    reward = 0.0
    info = {}
    
    # ... 보상 계산 ...
    
    # 디버깅 정보 추가
    info['ee_to_cube_dist'] = ee_to_cube_dist
    info['gripper_width'] = gripper_width
    info['cube_height'] = cube_height
    info['grasp_valid'] = grasp_valid
    info['grip_frames'] = self.grip_frames
    
    return reward, done, info
```

### 2. 마일스톤 달성 시 출력

```python
if self.grip_frames >= 3 and not self.valid_grip:
    reward += 10.0
    self.valid_grip = True
    print(f"🎯 GRIP 달성! Step: {self.step_count}")
```

### 3. 주기적 통계 출력

```python
if self.episode_count % 100 == 0:
    print(f"마일스톤 통계 (최근 100 에피소드):")
    print(f"  REACH: {self.milestone_count['reach']}")
    print(f"  GRIP: {self.milestone_count['grip']}")
    print(f"  LIFT: {self.milestone_count['lift']}")
```

---

## 🎓 고급 기법

### 1. Curriculum Reward

**아이디어**: 초기엔 쉬운 보상, 나중엔 어려운 보상

```python
# Phase 0: REACH에 집중
if curriculum_phase == 0:
    reward = 10.0 if reached else -0.1 * ee_to_cube_dist

# Phase 1: GRIP 추가
elif curriculum_phase == 1:
    reward = 5.0 * reached + 10.0 * gripped

# Phase 2: 전체 작업
else:
    reward = 5*reach + 10*grip + 15*lift + 20*goal + 100*success
```

---

### 2. Adaptive Reward Scaling

**아이디어**: 학습 진행에 따라 보상 스케일 조정

```python
# 초기: 큰 보상 (빠른 학습)
if step_count < 100000:
    reward_scale = 2.0
# 후기: 작은 보상 (안정적 학습)
else:
    reward_scale = 1.0

reward *= reward_scale
```

---

### 3. Multi-objective Reward

**아이디어**: 여러 목표 동시 추구

```python
# 주 목표
primary_reward = milestone_rewards

# 부 목표 1: 에너지 효율
energy_penalty = -0.001 * np.sum(np.abs(action))

# 부 목표 2: 부드러운 동작
smoothness_penalty = -0.01 * np.sum((action - prev_action)**2)

total_reward = primary_reward + energy_penalty + smoothness_penalty
```

---

## 📚 참고 자료

1. **OpenAI Spinning Up - Reward Engineering**
   - https://spinningup.openai.com/en/latest/spinningup/rl_intro3.html

2. **DeepMind - Reward Design**
   - "Reward is Enough" (Silver et al., 2021)

3. **실전 예제**
   - IsaacGymEnvs: Franka Cube Stack
   - RLBench: Pick and Place 작업

---

**작성자**: GitHub Copilot  
**최종 업데이트**: 2025-10-20
