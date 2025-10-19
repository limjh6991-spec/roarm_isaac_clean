# 강화학습 베스트 프랙티스 가이드
> 수집 날짜: 2025-10-19  
> 출처: Stable-Baselines3, OpenAI Spinning Up, RLlib, HuggingFace

## 📌 목차
1. [보상 설계 (Reward Shaping)](#보상-설계)
2. [PPO 알고리즘 하이퍼파라미터](#ppo-하이퍼파라미터)
3. [학습 디버깅](#학습-디버깅)
4. [환경 설계](#환경-설계)
5. [일반적인 실수와 해결법](#일반적인-실수)

---

## 보상 설계 (Reward Shaping)

### ✅ 핵심 원칙
1. **보상 스케일링**: 보상을 -1 ~ +1 또는 0 ~ 1 범위로 정규화
2. **Dense vs Sparse**:
   - **Sparse**: 목표 달성 시에만 보상 (학습 어려움, 안정적)
   - **Dense**: 매 스텝마다 중간 보상 (학습 빠름, 불안정 가능)
3. **보상 함수 구성요소**:
   - **Progress Reward**: 목표에 가까워질수록 증가
   - **Success Reward**: 목표 달성 시 큰 보상
   - **Penalty**: 실패/충돌 시 감점
   - **Time Penalty**: 효율성 유도 (작은 값)

### ⚠️ RoArm-M3 Pick & Place 분석

**문제**: 보상이 수천 단위로 폭발 (+3719, -3625)

**원인**:
```python
# 잘못된 예 (거리 기반 보상이 너무 큼)
reach_reward = -ee_to_cube_dist * 5.0   # 거리 0.5m → -2.5
move_reward = -cube_to_target_dist * 10.0  # 거리 1.0m → -10
```

**해결책**:
```python
# ✅ 올바른 스케일 (Stable-Baselines3 권장)
reach_reward_scale: 0.5 ~ 1.0
grasp_reward: 5.0
lift_reward_scale: 1.0 ~ 2.0
move_reward_scale: 1.0 ~ 2.0
success_reward: 10.0 ~ 50.0

# 예상 보상 범위: -50 ~ +50
```

### 📐 보상 정규화 공식
```python
# 방법 1: 거리 기반 보상 클리핑
reward = np.clip(-distance * scale, -max_val, max_val)

# 방법 2: 지수 감쇠
reward = scale * np.exp(-distance / threshold)

# 방법 3: Tanh 정규화
reward = scale * np.tanh(distance / threshold)
```

---

## PPO 하이퍼파라미터

### 🎯 기본 설정 (Stable-Baselines3 권장)

```python
from stable_baselines3 import PPO

model = PPO(
    "MlpPolicy",
    env,
    # === 핵심 파라미터 ===
    learning_rate=3e-4,        # 학습률 (3e-4 ~ 1e-3)
    n_steps=2048,              # 배치 크기 (2048 ~ 4096)
    batch_size=64,             # 미니배치 (32 ~ 128)
    n_epochs=10,               # PPO 업데이트 반복 (10 ~ 20)
    gamma=0.99,                # 할인율 (0.99 ~ 0.995)
    gae_lambda=0.95,           # GAE 람다 (0.9 ~ 0.99)
    clip_range=0.2,            # PPO 클리핑 (0.1 ~ 0.3)
    ent_coef=0.0,              # 엔트로피 계수 (0.0 ~ 0.01)
    
    # === 최적화 ===
    max_grad_norm=0.5,         # 그래디언트 클리핑 (0.5 ~ 1.0)
    vf_coef=0.5,               # Value function 계수
    
    # === 로깅 ===
    verbose=1,
    tensorboard_log="logs/tensorboard/",
)
```

### 📊 하이퍼파라미터 튜닝 가이드

| 문제 증상 | 조정 방법 |
|---------|---------|
| 학습 안 됨 (보상 flat) | `learning_rate` ↑ (1e-3), `n_steps` ↑ (4096) |
| 학습 불안정 (보상 진동) | `learning_rate` ↓ (1e-4), `clip_range` ↓ (0.1) |
| 수렴 느림 | `n_epochs` ↑ (20), `batch_size` ↑ (128) |
| 과적합 | `ent_coef` ↑ (0.01), `gamma` ↓ (0.95) |

---

## 학습 디버깅

### 🔍 체크리스트

1. **보상 추이 확인**:
   ```python
   # Tensorboard 사용
   tensorboard --logdir logs/tensorboard/
   
   # 또는 Monitor CSV 파싱
   import pandas as pd
   df = pd.read_csv("logs/monitor.csv", skiprows=1)
   print(df['r'].describe())  # 평균, 최소, 최대 확인
   ```

2. **보상 스케일 진단**:
   ```python
   # ✅ 정상: -100 ~ +100 범위
   # ⚠️ 문제: -10000 ~ +10000 (스케일 과도)
   # ⚠️ 문제: -1 ~ 0 (학습 신호 약함)
   ```

3. **에피소드 길이 확인**:
   ```python
   # 모든 에피소드가 타임아웃?
   # → 보상 함수 문제 또는 목표가 너무 어려움
   
   # 에피소드가 너무 짧음?
   # → Penalty가 너무 강함
   ```

4. **Value Loss 모니터링**:
   ```python
   # Value loss가 계속 증가?
   # → 보상 스케일 문제 또는 학습률 너무 높음
   
   # Policy loss가 0에 수렴?
   # → 학습이 멈춤, exploration 부족
   ```

### 🐛 일반적인 에러

#### 1. **NaN/Inf 발생**
```python
# 해결책
from stable_baselines3.common.vec_env import VecCheckNan

env = VecCheckNan(env, raise_exception=True)

# 또는 보상 클리핑
reward = np.clip(reward, -10, 10)
```

#### 2. **학습이 안 됨**
```python
# 체크 사항:
# 1. 보상이 항상 0 또는 음수?
# 2. 관찰 공간이 너무 크거나 작음?
# 3. 행동이 환경에 영향을 주지 않음?

# 디버그 방법
env.reset()
for i in range(100):
    obs, reward, done, info = env.step(env.action_space.sample())
    print(f"Step {i}: reward={reward:.2f}, obs_range=({obs.min():.2f}, {obs.max():.2f})")
```

---

## 환경 설계

### ✅ 관찰 공간 (Observation Space)

```python
# 좋은 예: 정규화된 관찰
obs = np.concatenate([
    joint_positions / np.pi,              # [-1, 1]
    joint_velocities / max_vel,          # [-1, 1]
    (ee_pos - center) / workspace_size,  # [-1, 1]
    cube_pos / workspace_size,           # [0, 1]
])

# 나쁜 예: 스케일이 다른 값들
obs = np.concatenate([
    joint_positions,      # [-3.14, 3.14]
    joint_velocities,     # [-10, 10]
    ee_pos,               # [0, 1]
    cube_pos,             # [0, 0.5]
])  # → NN이 학습하기 어려움!
```

### ✅ 행동 공간 (Action Space)

```python
# 연속 행동 (-1, +1)
action_space = gym.spaces.Box(
    low=-1.0,
    high=1.0,
    shape=(num_joints,),
    dtype=np.float32
)

# 행동을 실제 제어값으로 변환
target_positions = current_positions + actions * max_delta
```

### ✅ 종료 조건 (Termination)

```python
def _is_done(self):
    # 성공
    if distance_to_target < threshold:
        return True, "success"
    
    # 실패
    if cube_fell_off or robot_out_of_bounds:
        return True, "failure"
    
    # 타임아웃
    if self.steps >= max_steps:
        return True, "timeout"
    
    return False, None
```

---

## 일반적인 실수

### ❌ 실수 1: 보상 함수 너무 복잡
```python
# 나쁜 예: 10개 이상의 보상 요소
reward = (
    reach_reward + grasp_reward + lift_reward + 
    move_reward + success_reward + stability_reward +
    smoothness_reward + energy_reward + ...
)  # → 학습 불안정!

# ✅ 좋은 예: 3-5개 요소
reward = reach_reward + grasp_reward + success_reward
```

### ❌ 실수 2: 보상 스케일 무시
```python
# 나쁜 예
reward = -1000 * distance  # 보상 -10000 ~ 0

# ✅ 좋은 예
reward = -np.clip(distance * 5.0, 0, 10)  # 보상 -10 ~ 0
```

### ❌ 실수 3: Curriculum Learning 없이 어려운 목표
```python
# 나쁜 예: 처음부터 어려운 목표
target_pos = [0.5, 0.5, 0.5]  # 먼 거리

# ✅ 좋은 예: 점진적으로 어려워짐
difficulty = min(1.0, episode_count / 10000)
target_pos = initial_pos + direction * 0.1 * difficulty
```

---

## 📚 추가 리소스

### 필독 자료
1. **Stable-Baselines3 RL Tips**: https://stable-baselines3.readthedocs.io/en/master/guide/rl_tips.html
2. **OpenAI Spinning Up**: https://spinningup.openai.com/
3. **HuggingFace Deep RL Course**: https://huggingface.co/deep-rl-course

### 핵심 논문
1. **PPO**: [Proximal Policy Optimization](https://arxiv.org/abs/1707.06347)
2. **Reward Shaping**: [Policy Invariance Under Reward Transformations](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/NgHaradaRussell-shaping-ICML1999.pdf)
3. **Curriculum Learning**: [Automatic Curriculum Learning](https://arxiv.org/abs/2003.04960)

---

## 🎯 RoArm-M3 적용 요약

### 현재 문제
- ✅ Dense Reward 구현 완료
- ❌ 보상 스케일 과도 (수천 단위)
- ❌ 타임아웃 100%
- ❌ 성공률 0%

### 해결 방안
1. **보상 스케일 1/5 ~ 1/10 축소**
2. **관찰 공간 정규화 확인**
3. **학습률 조정** (3e-4 → 1e-4)
4. **더 긴 학습** (150K ~ 200K steps)
5. **Curriculum Learning 도입** (큐브 위치 점진적 증가)

