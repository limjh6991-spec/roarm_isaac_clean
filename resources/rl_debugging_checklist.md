# 강화학습 디버깅 체크리스트
> 출처: Stable-Baselines3, Spinning Up, 실전 경험  
> 날짜: 2025-10-19

## 🔍 문제 증상별 해결 가이드

### 문제 1: 학습이 전혀 안 됨 (보상 flat)

#### 증상
```
ep_rew_mean: -500 → -500 → -500 (변화 없음)
ep_len_mean: 600 → 600 → 600 (타임아웃)
```

#### 체크리스트
- [ ] **보상 함수 검증**
  ```python
  # 테스트: 랜덤 행동으로 보상 확인
  env.reset()
  rewards = []
  for _ in range(1000):
      action = env.action_space.sample()
      obs, reward, done, info = env.step(action)
      rewards.append(reward)
  
  print(f"Min: {min(rewards)}, Max: {max(rewards)}, Mean: {np.mean(rewards)}")
  # ✅ 정상: 보상이 변화함 (-10 ~ +10)
  # ❌ 문제: 보상이 항상 0 또는 음수
  ```

- [ ] **관찰 공간 확인**
  ```python
  # 관찰값이 너무 크거나 작지 않은가?
  obs = env.reset()
  print(f"Obs range: [{obs.min():.2f}, {obs.max():.2f}]")
  # ✅ 정상: [-1, 1] 또는 [0, 1]
  # ❌ 문제: [-1000, 1000] (정규화 필요)
  ```

- [ ] **행동이 환경에 영향**
  ```python
  # 같은 행동 반복
  obs1 = env.reset()
  for _ in range(10):
      obs2, _, _, _ = env.step(np.ones(env.action_space.shape))
  
  # obs1과 obs2가 다른가?
  print(f"Obs changed: {not np.allclose(obs1, obs2)}")
  # ✅ 정상: True
  # ❌ 문제: False (환경이 변하지 않음)
  ```

#### 해결 방법
1. **보상 함수 재설계** (가장 중요!)
2. **관찰 공간 정규화**
3. `learning_rate` 증가 (3e-4 → 1e-3)
4. `n_steps` 증가 (2048 → 4096)

---

### 문제 2: 학습이 불안정 (보상 진동)

#### 증상
```
ep_rew_mean: -100 → +200 → -50 → +300 → -200 (심하게 진동)
value_loss: 1000 → 5000 → 500 (불안정)
```

#### 체크리스트
- [ ] **보상 스케일 확인**
  ```python
  # 보상이 수천 단위?
  print(f"Reward range: [{rewards.min()}, {rewards.max()}]")
  # ❌ 문제: [-5000, +3000]
  # ✅ 해결: 보상 스케일 1/10 또는 1/100
  ```

- [ ] **approx_kl 확인**
  ```python
  # Tensorboard에서 확인
  # approx_kl > 0.1 → 업데이트가 너무 큼
  # approx_kl < 0.001 → 업데이트가 너무 작음
  # ✅ 정상: 0.01 ~ 0.05
  ```

- [ ] **clip_fraction 확인**
  ```python
  # clip_fraction > 0.5 → clipping 너무 자주 발생
  # ✅ 정상: 0.1 ~ 0.3
  ```

#### 해결 방법
1. **보상 정규화**
   ```python
   from stable_baselines3.common.vec_env import VecNormalize
   env = VecNormalize(env, norm_reward=True)
   ```

2. **하이퍼파라미터 조정**
   - `learning_rate` 감소 (3e-4 → 1e-4)
   - `clip_range` 감소 (0.2 → 0.1)
   - `max_grad_norm` 감소 (0.5 → 0.3)

3. **배치 크기 증가**
   - `n_steps` 증가 (2048 → 4096)
   - `batch_size` 증가 (64 → 128)

---

### 문제 3: 수렴이 느림

#### 증상
```
100K steps: ep_rew_mean = -200
200K steps: ep_rew_mean = -180
300K steps: ep_rew_mean = -170 (매우 느린 개선)
```

#### 체크리스트
- [ ] **Policy loss 확인**
  ```python
  # policy_loss ≈ 0?
  # → 학습이 멈춤, exploration 부족
  ```

- [ ] **Entropy 확인**
  ```python
  # entropy가 너무 작음?
  # → 정책이 너무 결정적 (Exploration 부족)
  ```

#### 해결 방법
1. **Exploration 증가**
   ```python
   ent_coef = 0.01  # 기본 0.0 → 0.01
   ```

2. **학습률 증가**
   ```python
   learning_rate = 1e-3  # 기본 3e-4 → 1e-3
   ```

3. **업데이트 횟수 증가**
   ```python
   n_epochs = 20  # 기본 10 → 20
   ```

4. **Curriculum Learning**
   ```python
   # 쉬운 목표부터 시작
   target_distance = 0.1 + progress * 0.4
   ```

---

### 문제 4: NaN 또는 Inf 발생

#### 증상
```
RuntimeError: CUDA error: device-side assert triggered
value_loss: nan
policy_loss: inf
```

#### 체크리스트
- [ ] **보상 범위 확인**
  ```python
  assert not np.isnan(reward).any()
  assert not np.isinf(reward).any()
  ```

- [ ] **관찰 범위 확인**
  ```python
  assert not np.isnan(obs).any()
  assert not np.isinf(obs).any()
  ```

#### 해결 방법
1. **VecCheckNan 사용**
   ```python
   from stable_baselines3.common.vec_env import VecCheckNan
   env = VecCheckNan(env, raise_exception=True)
   ```

2. **보상 클리핑**
   ```python
   reward = np.clip(reward, -10, 10)
   ```

3. **그래디언트 클리핑**
   ```python
   max_grad_norm = 0.5  # 기본값 유지
   ```

4. **학습률 감소**
   ```python
   learning_rate = 1e-5  # 매우 작게
   ```

---

### 문제 5: 과적합 (Training 좋음, Evaluation 나쁨)

#### 증상
```
Training: ep_rew_mean = +200
Evaluation: ep_rew_mean = -100
```

#### 해결 방법
1. **Regularization**
   ```python
   ent_coef = 0.01  # Entropy 증가
   ```

2. **Gamma 감소**
   ```python
   gamma = 0.95  # 0.99 → 0.95
   ```

3. **환경 랜덤화**
   ```python
   # 초기 상태 랜덤화
   cube_pos += np.random.uniform(-0.05, 0.05, size=3)
   ```

---

## 🛠️ 디버깅 도구

### 1. Tensorboard
```bash
# 실행
tensorboard --logdir logs/tensorboard/

# 확인 항목
# - ep_rew_mean: 증가?
# - ep_len_mean: 감소?
# - value_loss: 감소?
# - policy_loss: 안정?
# - approx_kl: 0.01 ~ 0.05?
```

### 2. Monitor CSV 분석
```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("logs/monitor.csv", skiprows=1)
df.columns = ['r', 'l', 't']

# 보상 추이
df['r'].rolling(100).mean().plot(title="Reward (100-episode MA)")
plt.show()

# 통계
print(df['r'].describe())
```

### 3. 프로파일링
```python
import cProfile
import pstats

# 학습 프로파일링
profiler = cProfile.Profile()
profiler.enable()

model.learn(total_timesteps=10000)

profiler.disable()
stats = pstats.Stats(profiler)
stats.sort_stats('cumtime')
stats.print_stats(20)
```

---

## 📋 학습 전 체크리스트

### ✅ 환경 검증
- [ ] 보상 함수가 의미 있는가?
- [ ] 관찰 공간이 정규화되었는가?
- [ ] 행동이 환경에 영향을 주는가?
- [ ] 종료 조건이 명확한가?
- [ ] 랜덤 행동으로 테스트했는가?

### ✅ 하이퍼파라미터
- [ ] `learning_rate`: 1e-5 ~ 1e-3
- [ ] `n_steps`: 512 ~ 8192
- [ ] `batch_size` < `n_steps`
- [ ] `clip_range`: 0.1 ~ 0.3
- [ ] `max_grad_norm`: 0.3 ~ 1.0

### ✅ 인프라
- [ ] GPU 사용 가능?
- [ ] 병렬 환경 사용?
- [ ] Tensorboard 설정?
- [ ] 체크포인트 저장?

---

## 🔬 고급 디버깅

### 1. Policy 행동 시각화
```python
# 학습된 정책 테스트
env = gym.make("CartPole-v1")
obs = env.reset()

for _ in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    env.render()
    if done:
        obs = env.reset()
```

### 2. Value Function 검증
```python
# Value function이 올바른가?
obs = env.reset()
value = model.policy.predict_values(obs)
print(f"Predicted value: {value}")

# 실제 return과 비교
actual_return = 0
gamma = 0.99
for t in range(100):
    action, _ = model.predict(obs)
    obs, reward, done, info = env.step(action)
    actual_return += reward * (gamma ** t)
    if done:
        break

print(f"Actual return: {actual_return}")
# Value와 return이 비슷한가?
```

### 3. Advantage 분석
```python
# Advantage 분포 확인
from stable_baselines3.common.evaluation import evaluate_policy

mean_reward, std_reward = evaluate_policy(
    model, env, n_eval_episodes=100
)
print(f"Mean reward: {mean_reward:.2f} +/- {std_reward:.2f}")
```

---

## 📚 참고 리소스

1. **Stable-Baselines3 디버깅**: https://stable-baselines3.readthedocs.io/en/master/guide/checking_nan.html
2. **RL 디버깅 가이드**: https://andyljones.com/posts/rl-debugging.html
3. **OpenAI Spinning Up**: https://spinningup.openai.com/en/latest/spinningup/spinningup.html

