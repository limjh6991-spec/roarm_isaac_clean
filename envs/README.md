# RoArm-M3 강화학습 환경 README

## 📋 개요

RoArm-M3 로봇팔의 Pick and Place 작업을 위한 강화학습 환경입니다.

- **환경**: Isaac Sim 5.0
- **알고리즘**: PPO (Proximal Policy Optimization)
- **프레임워크**: Stable-Baselines3
- **작업**: 큐브를 집어서 타겟 위치로 옮기기

## 🎯 환경 스펙

### Observation Space (15 dim)
- Joint positions (6 dim): `joint_1 ~ joint_6`
- Gripper state (2 dim): `left_finger, right_finger` positions
- End-effector position (3 dim): `x, y, z`
- Object position (3 dim): `x, y, z`
- Object-target distance (1 dim)

### Action Space (8 dim)
- Joint velocities (6 dim): `joint_1 ~ joint_6`
- Gripper velocities (2 dim): `left_finger, right_finger`

### Reward Function
```python
reward = -distance * 10.0  # Distance-based penalty
        + 100.0           # Success bonus (distance < 5cm)
```

### Termination Conditions
1. Success: Object reaches target (distance < 5cm)
2. Timeout: Max steps reached (600 steps = 10 seconds)
3. Failure: Object falls off table (z < -0.1m)

## 🚀 빠른 시작

### 1. 환경 설정
```bash
# 필수 패키지 설치
bash scripts/setup_rl_env.sh
```

### 2. 학습
```bash
# 50K 스텝 학습 (약 30-60분)
~/isaac-sim.sh -m scripts/train_roarm_rl.py --mode train --timesteps 50000

# 더 긴 학습 (100K 스텝)
~/isaac-sim.sh -m scripts/train_roarm_rl.py --mode train --timesteps 100000
```

### 3. 테스트
```bash
# Best 모델로 테스트
~/isaac-sim.sh -m scripts/train_roarm_rl.py --mode test

# 특정 모델 테스트
~/isaac-sim.sh -m scripts/train_roarm_rl.py --mode test \\
  --model logs/rl_training/checkpoints/roarm_ppo_10000_steps.zip
```

## 📊 학습 모니터링

### TensorBoard
```bash
# 학습 중 실시간 모니터링
tensorboard --logdir logs/rl_training/tensorboard
# → http://localhost:6006
```

**주요 지표**:
- `rollout/ep_rew_mean`: 에피소드 평균 보상
- `rollout/ep_len_mean`: 에피소드 평균 길이
- `train/policy_loss`: Policy network loss
- `train/value_loss`: Value network loss

### 체크포인트
학습 중 자동 저장:
- `logs/rl_training/checkpoints/`: 5,000 스텝마다
- `logs/rl_training/best_model/`: 평가 시 최고 성능 모델

## 🔧 하이퍼파라미터 튜닝

`scripts/train_roarm_rl.py`의 PPO 파라미터:

```python
model = PPO(
    learning_rate=3e-4,      # 학습률
    n_steps=2048,            # 버퍼 크기
    batch_size=64,           # 미니배치 크기
    n_epochs=10,             # 업데이트 에폭 수
    gamma=0.99,              # 할인율
    gae_lambda=0.95,         # GAE lambda
    clip_range=0.2,          # PPO clipping
    ent_coef=0.01,           # Entropy coefficient
)
```

**튜닝 팁**:
- 학습이 너무 느리면: `learning_rate` 증가 (3e-4 → 5e-4)
- 불안정하면: `learning_rate` 감소, `clip_range` 감소
- Exploration 부족: `ent_coef` 증가 (0.01 → 0.05)

## 📈 기대 성능

### 학습 곡선 (예상)
| Timesteps | Success Rate | Avg Reward |
|-----------|--------------|------------|
| 10K       | ~5%          | -50        |
| 25K       | ~20%         | -20        |
| 50K       | ~40%         | 0          |
| 100K      | ~60%         | +20        |

**참고**: 실제 성능은 시드, 환경 설정에 따라 달라질 수 있습니다.

## 🐛 문제 해결

### ImportError: omni.isaac.core
**원인**: Isaac Sim Python 환경이 아닙니다.  
**해결**: 반드시 `~/isaac-sim.sh -m` 또는 `isaacsim` 명령어로 실행

### CUDA out of memory
**원인**: GPU 메모리 부족  
**해결**:
1. `num_envs` 감소 (환경 병렬화 수)
2. `batch_size` 감소
3. Headless mode로 실행 (스크립트에서 `headless=True`)

### 학습이 진행되지 않음
**원인**: Reward shaping 문제  
**해결**:
1. `distance_reward_scale` 조정 (10.0 → 5.0 또는 20.0)
2. Success threshold 완화 (0.05m → 0.10m)
3. Episode length 증가 (10초 → 15초)

## 📁 파일 구조

```
roarm_isaac_clean/
├── envs/
│   ├── __init__.py
│   └── roarm_pick_place_env.py    # 환경 정의
├── scripts/
│   ├── train_roarm_rl.py          # 학습/테스트 스크립트
│   └── setup_rl_env.sh            # 환경 설정
├── logs/
│   └── rl_training/               # 학습 로그
│       ├── tensorboard/           # TensorBoard 로그
│       ├── checkpoints/           # 체크포인트
│       ├── best_model/            # Best 모델
│       └── eval_logs/             # 평가 로그
└── assets/
    └── roarm_m3/
        └── urdf/
            └── roarm_m3_multiprim.urdf
```

## 🔬 고급 사용

### 커스텀 Reward Function
`envs/roarm_pick_place_env.py`의 `_calculate_reward()` 수정:

```python
def _calculate_reward(self, obs: np.ndarray) -> float:
    distance = obs[14]
    
    # 기존: 거리 기반 + Success bonus
    reward = -distance * 10.0
    if distance < 0.05:
        reward += 100.0
    
    # 추가: Gripper 닫힘 보상
    gripper_distance = abs(obs[6] - obs[7])  # Left - Right
    if gripper_distance < 0.01:  # 닫혔을 때
        reward += 10.0
    
    # 추가: End-effector가 물체에 가까울 때
    ee_pos = obs[7:10]
    obj_pos = obs[9:12]
    ee_obj_dist = np.linalg.norm(ee_pos - obj_pos)
    reward -= ee_obj_dist * 5.0
    
    return reward
```

### 병렬 환경 (속도 향상)
`train_roarm_rl.py`에서 `num_envs` 증가:

```python
from stable_baselines3.common.vec_env import SubprocVecEnv

# 4개 환경 병렬 실행
env = SubprocVecEnv([make_env for _ in range(4)])
```

**주의**: GPU 메모리 충분한지 확인!

## 📚 참고 자료

- [Stable-Baselines3 문서](https://stable-baselines3.readthedocs.io/)
- [Isaac Sim Python API](https://docs.omniverse.nvidia.com/isaacsim/latest/python_api.html)
- [PPO 논문](https://arxiv.org/abs/1707.06347)
- [Gymnasium 문서](https://gymnasium.farama.org/)

## 🎯 다음 단계

1. **환경 복잡도 증가**:
   - 여러 물체 (Multi-object)
   - 다양한 형상 (구, 실린더 등)
   - 장애물 추가

2. **알고리즘 비교**:
   - SAC (Soft Actor-Critic)
   - TD3 (Twin Delayed DDPG)
   - DreamerV3

3. **Real-to-Sim Transfer**:
   - Domain Randomization
   - System Identification
   - 실제 로봇 테스트

---

**최종 업데이트**: 2025-10-19  
**작성자**: Jarvis AI Assistant
