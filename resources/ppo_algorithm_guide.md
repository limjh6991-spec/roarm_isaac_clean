# PPO (Proximal Policy Optimization) 완전 가이드
> 출처: OpenAI Spinning Up, Stable-Baselines3  
> 날짜: 2025-10-19

## 📌 목차
1. [PPO란 무엇인가?](#ppo란-무엇인가)
2. [PPO 알고리즘 작동 원리](#작동-원리)
3. [PPO vs 다른 알고리즘](#ppo-vs-다른-알고리즘)
4. [PPO 하이퍼파라미터 상세](#하이퍼파라미터-상세)
5. [실전 팁](#실전-팁)

---

## PPO란 무엇인가?

**Proximal Policy Optimization (PPO)**는 2017년 OpenAI가 발표한 On-Policy 강화학습 알고리즘입니다.

### 🎯 핵심 아이디어
- **Policy Gradient** 방법의 개선판
- **Trust Region** 개념 도입 (큰 업데이트 방지)
- **Clipping** 메커니즘으로 안정적 학습

### ✅ 장점
1. **안정적**: TRPO보다 구현 간단, 비슷한 성능
2. **범용적**: 연속/이산 행동 모두 사용 가능
3. **샘플 효율적**: On-Policy 중에서는 우수
4. **하이퍼파라미터 민감도 낮음**: 대부분의 환경에서 기본 설정으로 작동

### ❌ 단점
1. **샘플 비효율적**: Off-Policy (SAC, TD3) 대비
2. **메모리 사용**: 큰 배치 필요
3. **병렬화 필수**: 빠른 학습을 위해

---

## 작동 원리

### 1. 데이터 수집 (Rollout)
```python
# n_steps 동안 환경과 상호작용
for step in range(n_steps):
    action = policy.predict(observation)
    next_obs, reward, done, info = env.step(action)
    buffer.store(obs, action, reward, done, value)
    obs = next_obs
```

### 2. Advantage 계산 (GAE)
```python
# Generalized Advantage Estimation
advantage = 0
for t in reversed(range(n_steps)):
    delta = reward[t] + gamma * value[t+1] - value[t]
    advantage[t] = delta + gamma * gae_lambda * advantage[t+1]

# 정규화
advantage = (advantage - advantage.mean()) / (advantage.std() + 1e-8)
```

### 3. PPO 손실 함수
```python
# Policy Loss (Clipped Surrogate Objective)
ratio = torch.exp(new_log_prob - old_log_prob)
surr1 = ratio * advantage
surr2 = torch.clamp(ratio, 1 - clip_range, 1 + clip_range) * advantage
policy_loss = -torch.min(surr1, surr2).mean()

# Value Loss
value_loss = F.mse_loss(value_pred, returns)

# Entropy Loss (Exploration)
entropy_loss = -entropy.mean()

# Total Loss
total_loss = policy_loss + vf_coef * value_loss + ent_coef * entropy_loss
```

### 4. 업데이트 반복
```python
# n_epochs 동안 미니배치로 반복 학습
for epoch in range(n_epochs):
    for batch in get_minibatches(buffer, batch_size):
        optimizer.zero_grad()
        loss = compute_loss(batch)
        loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), max_grad_norm)
        optimizer.step()
```

---

## PPO vs 다른 알고리즘

| 알고리즘 | 타입 | 샘플 효율 | 안정성 | 적용 |
|---------|-----|----------|--------|------|
| **PPO** | On-Policy | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 로봇, 게임, 제어 |
| **SAC** | Off-Policy | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | 연속 제어 |
| **TD3** | Off-Policy | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | 연속 제어 |
| **A2C** | On-Policy | ⭐⭐ | ⭐⭐⭐ | 빠른 프로토타입 |
| **DQN** | Off-Policy | ⭐⭐⭐ | ⭐⭐ | 이산 행동 |

### 🤔 언제 PPO를 사용할까?
- ✅ **안정적인 학습**이 중요할 때
- ✅ **로봇 제어** 같은 연속 행동
- ✅ **처음 시도**하는 환경 (기본 선택)
- ❌ 샘플 효율이 매우 중요할 때 (SAC 사용)

---

## 하이퍼파라미터 상세

### 🎛️ 핵심 파라미터

#### 1. `learning_rate` (학습률)
```python
# 기본: 3e-4
# 범위: 1e-5 ~ 1e-3

# 너무 높으면: 학습 불안정, NaN 발생
# 너무 낮으면: 학습 느림, 수렴 안 함

# 스케줄링 예제
from stable_baselines3.common.callbacks import LinearSchedule
learning_rate = LinearSchedule(3e-4, 1e-5).value
```

#### 2. `n_steps` (배치 크기)
```python
# 기본: 2048
# 범위: 512 ~ 8192

# 작으면: 학습 빠름, 불안정
# 크면: 학습 안정, 메모리 많이 사용

# 계산 공식
total_samples_per_update = n_steps * num_envs
# 예: 2048 * 4 = 8192 samples
```

#### 3. `batch_size` (미니배치)
```python
# 기본: 64
# 범위: 32 ~ 512

# 규칙: batch_size < n_steps
# 일반적: batch_size = n_steps / {8, 16, 32}

# 예:
n_steps = 2048
batch_size = 64  # 2048 / 32 = 64
```

#### 4. `n_epochs` (업데이트 반복)
```python
# 기본: 10
# 범위: 3 ~ 30

# 적으면: 샘플 비효율
# 많으면: 과적합 위험

# 적응 예제
if value_loss < threshold:
    n_epochs = 15  # 더 많이 학습
```

#### 5. `gamma` (할인율)
```python
# 기본: 0.99
# 범위: 0.9 ~ 0.9999

# 낮으면: 단기 보상 중시
# 높으면: 장기 보상 중시

# 에피소드 길이에 따라 조정
# 짧은 에피소드: gamma = 0.95
# 긴 에피소드: gamma = 0.99
```

#### 6. `gae_lambda` (GAE 람다)
```python
# 기본: 0.95
# 범위: 0.9 ~ 0.99

# 낮으면: 편향 높음, 분산 낮음
# 높으면: 편향 낮음, 분산 높음

# 일반적: gamma와 비슷한 값
gae_lambda = 0.95
gamma = 0.99
```

#### 7. `clip_range` (클리핑 범위)
```python
# 기본: 0.2
# 범위: 0.1 ~ 0.3

# 작으면: 안정, 느림
# 크면: 빠름, 불안정

# 적응적 클리핑
clip_range = schedule(initial=0.2, final=0.1)
```

#### 8. `ent_coef` (엔트로피 계수)
```python
# 기본: 0.0
# 범위: 0.0 ~ 0.01

# 0이면: Exploration 약함
# 높으면: 랜덤 행동 증가

# 학습 초기: 0.01 (탐색)
# 학습 후기: 0.0 (Exploitation)
```

---

## 실전 팁

### 💡 팁 1: 하이퍼파라미터 튜닝 순서
1. **먼저 보상 함수 검증** (가장 중요!)
2. `learning_rate` 조정 (1e-4 ~ 1e-3)
3. `n_steps` 조정 (1024 ~ 4096)
4. `clip_range` 조정 (0.1 ~ 0.3)
5. 나머지는 기본값 유지

### 💡 팁 2: 학습 모니터링
```python
# Tensorboard 필수 확인 항목
# 1. ep_rew_mean: 증가하는가?
# 2. ep_len_mean: 감소하는가? (효율성)
# 3. policy_loss: 너무 크거나 작지 않은가?
# 4. value_loss: 감소하는가?
# 5. approx_kl: 0.01 ~ 0.05 범위인가?
# 6. clip_fraction: 0.1 ~ 0.3 범위인가?
```

### 💡 팁 3: 병렬 환경
```python
# 단일 환경: 느림
env = gym.make("CartPole-v1")

# 병렬 환경: 빠름 (권장)
from stable_baselines3.common.vec_env import SubprocVecEnv

def make_env():
    return gym.make("CartPole-v1")

env = SubprocVecEnv([make_env for _ in range(4)])

# n_steps * num_envs = total samples
# 2048 * 4 = 8192 samples per update
```

### 💡 팁 4: 학습 시간 단축
```python
# 1. GPU 사용
model = PPO("MlpPolicy", env, device="cuda")

# 2. 병렬 환경
env = SubprocVecEnv([make_env for _ in range(8)])

# 3. n_steps 증가
n_steps = 4096  # 더 큰 배치

# 4. n_epochs 감소
n_epochs = 5  # 빠른 업데이트
```

### 💡 팁 5: 디버깅
```python
# 문제: 학습 안 됨
# 체크 1: 보상이 변화하는가?
# 체크 2: approx_kl이 너무 큰가? (> 0.1)
# 체크 3: value_loss가 감소하는가?

# 해결:
# - learning_rate 낮추기
# - clip_range 낮추기
# - n_epochs 줄이기
```

---

## 🔬 고급 기법

### 1. Learning Rate 스케줄링
```python
def linear_schedule(initial_value):
    def func(progress_remaining):
        return progress_remaining * initial_value
    return func

model = PPO(
    "MlpPolicy",
    env,
    learning_rate=linear_schedule(3e-4),
)
```

### 2. Curriculum Learning
```python
class CurriculumEnv(gym.Env):
    def __init__(self):
        self.difficulty = 0.1
    
    def increase_difficulty(self):
        self.difficulty = min(1.0, self.difficulty + 0.1)
    
    def reset(self):
        # 어려움에 따라 초기 상태 변경
        return self._get_obs()
```

### 3. Reward Scaling
```python
from stable_baselines3.common.vec_env import VecNormalize

env = DummyVecEnv([make_env])
env = VecNormalize(env, norm_reward=True, norm_obs=True)

# 학습 후
env.save("vec_normalize.pkl")
```

---

## 📚 참고 자료

### 논문
1. **PPO 원본**: [Proximal Policy Optimization Algorithms](https://arxiv.org/abs/1707.06347)
2. **GAE**: [High-Dimensional Continuous Control Using Generalized Advantage Estimation](https://arxiv.org/abs/1506.02438)
3. **TRPO**: [Trust Region Policy Optimization](https://arxiv.org/abs/1502.05477)

### 코드
1. **Stable-Baselines3 PPO**: https://github.com/DLR-RM/stable-baselines3/blob/master/stable_baselines3/ppo/ppo.py
2. **OpenAI Spinning Up PPO**: https://github.com/openai/spinningup/tree/master/spinup/algos/pytorch/ppo
3. **CleanRL PPO**: https://github.com/vwxyzjn/cleanrl/blob/master/cleanrl/ppo.py

### 튜토리얼
1. **HuggingFace Deep RL**: https://huggingface.co/deep-rl-course/unit8/introduction
2. **Stable-Baselines3 Docs**: https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html
3. **Spinning Up PPO**: https://spinningup.openai.com/en/latest/algorithms/ppo.html

---

## 🎯 RoArm-M3 적용

### 현재 설정
```python
PPO(
    "MlpPolicy",
    env,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
)
```

### 제안 수정
```python
PPO(
    "MlpPolicy",
    env,
    learning_rate=1e-4,        # ↓ 안정성
    n_steps=4096,              # ↑ 샘플 수
    batch_size=128,            # ↑ 배치
    n_epochs=15,               # ↑ 업데이트
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.1,            # ↓ 안정성
    ent_coef=0.005,            # + Exploration
    max_grad_norm=0.5,
    device="cuda",
)
```

