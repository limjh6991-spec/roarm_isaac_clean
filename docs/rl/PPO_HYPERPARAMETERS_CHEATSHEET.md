# PPO 하이퍼파라미터 치트시트

**목적**: PPO 학습 시 빠른 참조

---

## 📋 기본 설정 (로봇 조작)

```python
from stable_baselines3 import PPO

model = PPO(
    "MlpPolicy",
    env,
    learning_rate=3e-4,      # ⭐ 가장 중요!
    n_steps=2048,            # Rollout buffer
    batch_size=64,           # Minibatch size
    n_epochs=10,             # Update epochs
    gamma=0.99,              # Discount factor
    gae_lambda=0.95,         # GAE parameter
    clip_range=0.2,          # PPO clip
    ent_coef=0.01,           # Entropy bonus
    vf_coef=0.5,             # Value loss weight
    max_grad_norm=0.5,       # Gradient clipping
    verbose=1,
    tensorboard_log="./logs/"
)
```

---

## 🔧 문제별 해결책

### 문제 1: 학습 불안정 (Loss 발산)
**증상**: Policy/Value loss가 폭발적으로 증가
**해결**:
```python
learning_rate = 1e-4      # 3e-4 → 1e-4 (감소)
clip_range = 0.1          # 0.2 → 0.1 (감소)
max_grad_norm = 0.3       # 0.5 → 0.3 (감소)
```

---

### 문제 2: 학습 너무 느림
**증상**: 1M steps 후에도 성능 미개선
**해결**:
```python
learning_rate = 5e-4      # 3e-4 → 5e-4 (증가)
ent_coef = 0.02           # 0.01 → 0.02 (탐험 증가)
n_steps = 4096            # 2048 → 4096 (더 많은 데이터)
```

---

### 문제 3: Over-fitting
**증상**: Training 좋지만 Test 실패
**해결**:
```python
ent_coef = 0.05           # 0.01 → 0.05 (탐험 증가)
# + Domain Randomization 추가
# + Observation noise 추가
```

---

### 문제 4: 탐험 부족
**증상**: 로봇이 특정 동작만 반복
**해결**:
```python
ent_coef = 0.05           # 0.01 → 0.05 (엔트로피 증가)
learning_rate = 5e-4      # 더 빠른 학습
```

---

## 📊 파라미터 범위 요약표

| 파라미터 | 안정적 | 빠름 | 탐험적 | RoArm 권장 |
|---------|-------|------|--------|-----------|
| **learning_rate** | 1e-4 | 5e-4 | 3e-4 | **3e-4** |
| **clip_range** | 0.1 | 0.3 | 0.2 | **0.2** |
| **ent_coef** | 0.001 | 0.01 | 0.05 | **0.01** |
| **n_steps** | 1024 | 4096 | 2048 | **2048** |
| **batch_size** | 32 | 128 | 64 | **64** |
| **n_epochs** | 5 | 15 | 10 | **10** |

---

## 🎯 RoArm M3 현재 설정

**파일**: `scripts/rl/train_dense_reward.py`

```python
model = PPO(
    "MlpPolicy",
    env,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.01,
    vf_coef=0.5,
    max_grad_norm=0.5,
    verbose=1,
    tensorboard_log="./logs/"
)
```

**평가**: ✅ 균형잡힌 설정 (표준 권장값)

---

## 🔍 학습 중 모니터링

**TensorBoard**:
```bash
tensorboard --logdir ./logs/
```

**주요 지표**:
1. **ep_rew_mean**: 증가 추세?
2. **ep_len_mean**: 감소 추세? (효율적 해결)
3. **policy_loss**: 안정적?
4. **value_loss**: 안정적?
5. **entropy_loss**: 0에 가까워지면 탐험 부족

---

## 💡 빠른 팁

1. **처음엔 표준값 사용** (위의 RoArm 권장)
2. **50K~100K steps 후 평가**
3. **문제 발견 시 위의 해결책 적용**
4. **한 번에 한 파라미터만 변경**
5. **변경 후 최소 50K steps 재학습**

---

**출처**: Stable-Baselines3 공식 문서 + RL Zoo
