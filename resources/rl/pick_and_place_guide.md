# Pick and Place 강화학습 자료집

## 📚 목차
1. [핵심 논문 및 방법론](#핵심-논문-및-방법론)
2. [보상 함수 설계](#보상-함수-설계)
3. [환경 구성 Best Practices](#환경-구성-best-practices)
4. [하이퍼파라미터 튜닝](#하이퍼파라미터-튜닝)
5. [Curriculum Learning](#curriculum-learning)
6. [디버깅 체크리스트](#디버깅-체크리스트)

---

## 핵심 논문 및 방법론

### 1. **Learning Dexterous In-Hand Manipulation** (OpenAI, 2018)
- **핵심**: 대규모 병렬 시뮬레이션 + Domain Randomization
- **적용 가능**: 
  - 물리 파라미터 랜덤화 (마찰, 질량, 댐핑)
  - 100+ 병렬 환경으로 학습 가속
  - PPO 알고리즘 사용
- **보상 함수**:
  ```python
  # 거리 기반 dense reward
  reward = -distance_to_goal
  # + 성공 보너스
  if success: reward += 100
  ```

### 2. **QT-Opt: Scalable Deep RL for Vision-Based Robotic Manipulation** (Google Brain, 2018)
- **핵심**: Off-policy 학습 (Q-learning) + 대규모 데이터
- **적용 가능**:
  - 탐색 전략: ε-greedy, Boltzmann
  - 리플레이 버퍼 사용
  - 시각 기반 학습 준비

### 3. **Hindsight Experience Replay (HER)** (OpenAI, 2017)
- **핵심**: 실패한 에피소드도 학습에 활용
- **적용 방법**:
  ```python
  # 실패 에피소드: 큐브를 A로 옮기려다 B에 떨어뜨림
  # → "B로 옮기는 것이 목표였다"고 재해석
  # → 성공 경험으로 학습!
  ```
- **장점**: Sparse reward 환경에서 매우 효과적
- **구현**: Stable-Baselines3에 HER 내장

### 4. **Automatic Curriculum Learning** (Berkeley AI, 2020)
- **핵심**: 난이도 자동 조절
- **단계별 학습**:
  1. Phase 0: 큐브 가까이 (10-15cm)
  2. Phase 1: 중간 거리 (20-30cm)
  3. Phase 2: 먼 거리 (30-50cm)
- **승급 조건**: 성공률 60% 이상 100 에피소드

---

## 보상 함수 설계

### A. Sparse Reward (단순하지만 학습 느림)
```python
reward = 0
if cube_at_target:
    reward = 100  # 성공 시에만 보상
```
**장점**: 명확한 목표
**단점**: 탐색 어려움 (운 좋게 성공해야 학습 시작)

### B. Dense Reward (학습 빠름)
```python
# 1. 거리 기반 보상 (연속적)
reward = -distance_ee_to_cube * 10  # EE가 큐브에 가까워지도록

# 2. 성공 단계별 보상
if gripper_near_cube:
    reward += 5
if grasped:
    reward += 10
if lifted:
    reward += 15
if cube_near_target:
    reward += 20
if success:
    reward += 100
```
**장점**: 에이전트가 진행 방향 알 수 있음
**단점**: 보상 함수 설계 중요 (잘못하면 local minimum)

### C. Shaped-Sparse (현재 적용 방식, 권장!)
```python
# 주요 이벤트에만 보상 (1회성)
reward = 0

# 게이팅 (한 번만 보상)
if first_time_near_cube:
    reward += 5
    mark_as_completed("near_cube")

if first_time_grasped:
    reward += 10
    mark_as_completed("grasped")

# ... 생략
```
**장점**: Dense의 학습 속도 + Sparse의 명확성
**단점**: 이벤트 감지 로직 필요

### D. 현재 RoArm-M3 보상 함수 (Shaped-Sparse)
```python
# envs/roarm_pick_place_env.py
def _compute_rewards(self):
    reward = 0.0
    
    # 1. 근접 보너스 (1회)
    if not self.reached_near_cube and distance_to_cube < 0.1:
        reward += 5.0
        self.reached_near_cube = True
    
    # 2. 그립 보너스 (1회)
    if not self.reached_grasp and is_grasped:
        reward += 10.0
        self.reached_grasp = True
    
    # 3. 리프트 보너스 (1회)
    if not self.reached_lift and cube_z > 0.15:
        reward += 15.0
        self.reached_lift = True
    
    # 4. 목표 근접 보너스 (1회)
    if not self.reached_near_target and distance_cube_to_target < 0.1:
        reward += 20.0
        self.reached_near_target = True
    
    # 5. 성공 보너스
    if is_success:
        reward += 100.0
    
    # 6. 시간 페널티 (효율성 유도)
    reward -= 0.01
    
    return reward
```

---

## 환경 구성 Best Practices

### 1. 관측 공간 (Observation Space)
**권장**: **상대 좌표** (EE 기준)
```python
obs = [
    joint_positions,           # 8 dim
    cube_rel_to_ee,           # 3 dim (EE 기준!)
    target_rel_to_ee,         # 3 dim (EE 기준!)
    cube_to_target,           # 3 dim (World 기준)
    ee_velocity,              # 3 dim
    cube_velocity,            # 3 dim
    gripper_width,            # 1 dim
    is_grasped,               # 1 dim
    distance_to_cube,         # 1 dim
    distance_cube_to_target,  # 1 dim
    previous_reward,          # 1 dim (temporal context)
]
# Total: 28 dim
```

**이유**:
- 절대 좌표는 일반화 어려움 (로봇 위치 바뀌면 학습 실패)
- 상대 좌표는 이동 불변성 보장
- 속도 정보는 모멘텀 학습 도움

### 2. 액션 공간 (Action Space)
**현재 방식**: Position delta (위치 증분)
```python
action = [-1, +1] 범위의 8차원 벡터
new_joint_pos = current_joint_pos + action * action_scale
```

**대안**: Velocity control (속도 제어)
```python
joint_velocity = action * max_velocity
```
- **장점**: 더 부드러운 동작
- **단점**: 구현 복잡도 증가

**권장**: Position delta (현재 방식 유지)

### 3. 에피소드 길이
- **너무 짧음** (< 5초): 학습 부족
- **너무 김** (> 15초): 비효율적
- **권장**: **10초 (600 steps @ 60 FPS)**

### 4. 성공 조건
```python
# 현재 설정 (엄격함!)
success_threshold = 0.02  # 2cm
success_hold_frames = 10  # 10프레임 연속 유지

# 초기 학습용 (완화)
success_threshold = 0.05  # 5cm
success_hold_frames = 5   # 5프레임
```

---

## 하이퍼파라미터 튜닝

### PPO 하이퍼파라미터 (현재 설정)
```python
learning_rate = 3e-4       # ✅ 기본값
n_steps = 2048            # ✅ 충분히 큼
batch_size = 64           # ✅ 적절
n_epochs = 10             # ✅ 기본값
gamma = 0.99              # ✅ 장기 보상 중시
gae_lambda = 0.95         # ✅ TD(λ) 균형
clip_range = 0.2          # ✅ 정책 안정성
clip_range_vf = 1.0       # ✅ Value clipping
ent_coef = 0.01           # ✅ 탐색 강화
vf_coef = 0.5             # ✅ Value 학습 비중
max_grad_norm = 0.5       # ✅ 그래디언트 클리핑
target_kl = 0.03          # ✅ KL divergence 제한
```

### 튜닝 우선순위
1. **learning_rate**: 3e-4 (기본) → 학습 불안정하면 1e-4로 감소
2. **ent_coef**: 0.01 → 탐색 부족하면 0.02로 증가
3. **clip_range**: 0.2 → 학습 느리면 0.3으로 증가
4. **n_epochs**: 10 → 과적합 징후 시 5로 감소

### 조기 종료 조건 (Early Stopping)
```python
# Explained Variance < 0.05 연속 5회
if ev < 0.05 for 5 iterations:
    print("⚠️ EV 너무 낮음 → 학습 중단")
    break

# Value Loss 증가 연속 5회
if value_loss increasing for 5 iterations:
    print("⚠️ Value 과적합 → 학습 중단")
    break
```

---

## Curriculum Learning

### Phase 0: Easy Mode (현재 단계)
```python
# 큐브 위치: 15-20cm (로봇 가까이)
cube_distance = np.random.uniform(0.15, 0.20)

# 타겟 위치: 25-30cm
target_distance = np.random.uniform(0.25, 0.30)
```
**목표**: 기본 그립 학습 (REACH, GRIP)
**승급 조건**: 성공률 30% 이상 (100 에피소드)

### Phase 1: Normal Mode (다음 단계)
```python
# 큐브 위치: 35-50cm (중간 거리)
cube_distance = np.random.uniform(0.35, 0.50)

# 타겟 위치: 35-50cm
target_distance = np.random.uniform(0.35, 0.50)
```
**목표**: 리프트 및 이동 학습 (LIFT, PLACE)
**승급 조건**: 성공률 60% 이상 (200 에피소드)

### Phase 2: Hard Mode (최종 단계)
```python
# 완전 랜덤 (작업 공간 전체)
cube_pos = random_in_workspace()
target_pos = random_in_workspace()
```
**목표**: 일반화 능력 강화

---

## 디버깅 체크리스트

### 학습이 안 될 때 (보상 증가 없음)
1. ✅ **관측 벡터 확인**: `scripts/verification/cube_observation_check.py`
2. ✅ **보상 함수 로깅**: 각 보상 요소 출력
3. ✅ **액션 범위 확인**: 관절이 실제로 움직이는가?
4. ✅ **성공 조건 완화**: threshold 0.02 → 0.05

### 학습은 되지만 성능 낮음
1. ✅ **Curriculum 활성화**: Phase 0부터 시작
2. ✅ **탐색 증가**: ent_coef 0.01 → 0.02
3. ✅ **에피소드 길이 증가**: 10초 → 15초
4. ✅ **병렬 환경 추가**: num_envs 1 → 4 or 8

### Value Loss 폭발
1. ✅ **VecNormalize 확인**: norm_reward=True
2. ✅ **Reward clipping**: clip_reward=10.0
3. ✅ **Learning rate 감소**: 3e-4 → 1e-4
4. ✅ **Value clipping**: clip_range_vf=1.0

### 정책이 local minimum에 빠짐
1. ✅ **탐색 강화**: ent_coef 증가
2. ✅ **보상 함수 수정**: local minimum 유발 요소 제거
3. ✅ **초기화 랜덤화**: 큐브/타겟 위치 다양화

---

## 성공 사례 분석

### OpenAI Dactyl (큐브 회전)
- **학습 시간**: 50시간 (384 CPU 코어)
- **에피소드 수**: ~2.7M
- **성공률**: 80%+
- **핵심**: Domain Randomization + 대규모 병렬

### Google Grasp (픽앤플레이스)
- **학습 시간**: 800 로봇 시간 (14시간 실제)
- **성공률**: 96%
- **핵심**: 대규모 데이터 + QT-Opt

### 현재 RoArm-M3 목표
- **학습 시간**: 1-2시간 (1 GPU)
- **Timesteps**: 100K-200K
- **목표 성공률**: 80%+
- **전략**: Curriculum + Shaped-Sparse

---

## 참고 문헌

1. Andrychowicz et al. "Hindsight Experience Replay" (2017)
2. OpenAI et al. "Learning Dexterous In-Hand Manipulation" (2018)
3. Kalashnikov et al. "QT-Opt" (2018)
4. Narvekar et al. "Curriculum Learning for RL: A Survey" (2020)
5. Stable-Baselines3 Documentation (2024)

---

## 추가 자료

- `resources/rl/tutorials/`: 단계별 튜토리얼
- `resources/rl/papers/`: 핵심 논문 PDF
- `resources/rl/best_practices/`: 실전 팁
- `docs/observation_space.md`: 관측 공간 명세서

