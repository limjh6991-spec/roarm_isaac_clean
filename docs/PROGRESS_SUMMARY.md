# Vision RL Progress Summary
*Last Updated: 2025-11-02*

## ✅ 완료된 작업

### Phase 1: Vision 전처리 검증 ✅
- **파일**: `scripts/test/test_vision_quick.py`
- **결과**: 10개 이미지 생성 성공
- **Output**: RGB (3, 84, 84) + Depth (1, 84, 84) = RGBD (4, 84, 84) [0, 1]

### Phase 2: 알고리즘 선택 ✅
- **결정**: PPO → **SAC** (Soft Actor-Critic)
- **이유**:
  * Sample Efficiency: 500K steps (PPO: 2-5M steps)
  * Off-Policy Learning: Replay buffer로 데이터 재사용
  * Vision 특화: "SAC for Pixels" 검증
  * 학습 시간: 5-10 hours (PPO: 10-20 hours)

### Phase 3: SAC 가이드 작성 ✅
- **파일**: `docs/VISION_SAC_GUIDE.md` (~500 lines)
- **내용**:
  * SAC vs PPO 비교표
  * Phase 1-4 구현 단계
  * train_vision_sac.py 완전 예시
  * eval_vision_sac.py 완전 예시
  * Hyperparameter tuning 가이드
  * TensorBoard 모니터링

### Phase 4: CNN Feature Extractor 구현 ✅
- **파일**: `models/cnn_extractor.py` (~200 lines)
- **클래스**:
  * `NatureCNN(observation_space, features_dim=512)` - DQN 스타일
  * `LargerCNN(observation_space, features_dim=1024)` - 복잡한 task용
  * `test_cnn()` - 테스트 함수

**NatureCNN Architecture**:
```python
Input: (4, 84, 84) RGBD
Conv2d(4→32, kernel=8×8, stride=4, padding=0)  # Output: (32, 20, 20)
ReLU + Conv2d(32→64, kernel=4×4, stride=2, padding=0)  # Output: (64, 9, 9)
ReLU + Conv2d(64→64, kernel=3×3, stride=1, padding=0)  # Output: (64, 7, 7)
Flatten: (64×7×7 = 3136,)
Linear(3136→512)
ReLU
```

---

## ✅ 최근 완료 (2025-11-02)

### models/__init__.py 수정 ✅
- **문제**: Markdown 형식 → Lint 오류 48개
- **해결**: Python docstring 형식으로 변경 완료

### CNN Feature Extractor 테스트 ✅
- **NatureCNN**: 1,684,128 parameters
- **LargerCNN**: 3,540,320 parameters (2.10x)
- **Forward pass**: (8, 4, 84, 84) → (8, 512/1024)
- **출력 범위**: [0.000, 0.127] (NatureCNN)

### Vision Environment 확인 ✅
**파일**: `envs/roarm_pick_place_env_vision.py`

**구현된 기능**:
- ✅ RGB-D observation (4, 84, 84)
- ✅ Phase-based reward (reach → grasp → lift → place)
- ✅ Object/Goal randomization
- ✅ Contact detection (grasp check)
- ✅ Vision guidance bonus

### SAC Training Script 생성 ✅
**파일**: `scripts/train/train_vision_sac.py`

**주요 설정**:
```python
# SAC Configuration
buffer_size = 100_000
batch_size = 256
learning_rate = 3e-4
total_timesteps = 500_000

# Policy with NatureCNN
policy_kwargs = dict(
    features_extractor_class=NatureCNN,
    features_extractor_kwargs=dict(features_dim=512),
    net_arch=[256, 256],
)
```

---

## 🎯 다음 단계

### Step 1: Environment 테스트 (오늘)
**파일**: `scripts/train/train_vision_sac.py`

**기반**: `VISION_SAC_GUIDE.md` 예시

**주요 컴포넌트**:
```python
from stable_baselines3 import SAC
from models.cnn_extractor import NatureCNN

# Environment
env = RoArmPickPlaceEnvVision(...)

# SAC Model
policy_kwargs = dict(
    features_extractor_class=NatureCNN,
    features_extractor_kwargs=dict(features_dim=512),
    net_arch=[256, 256]
)

model = SAC(
    "CnnPolicy",
    env,
    policy_kwargs=policy_kwargs,
    buffer_size=100_000,
    batch_size=256,
    learning_rate=3e-4,
    tau=0.005,
    gamma=0.99,
    train_freq=1,
    gradient_steps=1,
    tensorboard_log="./tensorboard_logs/",
    verbose=1
)

# Training
model.learn(total_timesteps=500_000)
model.save("roarm_sac_vision")
```

**명령어**:
```bash
/home/roarm_m3/isaacsim/python.sh scripts/train/train_vision_sac.py
```

**예상**:
- Target: 500K steps
- Time: 5-10 hours
- 모니터링: TensorBoard (매 10K steps)
- Checkpoint: 매 10K steps 저장
- Evaluation: 매 10K steps (10 episodes)

---

## 📊 예상 학습 곡선

| Steps | Success Rate | Reward | 비고 |
|-------|-------------|--------|------|
| 0-50K | 0% | -300 | Random exploration |
| 50K-100K | 5-10% | -200 | 첫 번째 성공 |
| 100K-200K | 20-30% | -100 | 일부 성공 |
| 200K-300K | 40-50% | -50 | 안정적 향상 |
| 300K-500K | 60-70% | 0-50 | 목표 달성 |

---

## 🗂️ 파일 구조

```
roarm_isaac_clean/
├── docs/
│   ├── VISION_RL_PLAN.md       # 전체 프로젝트 계획 (SAC로 업데이트됨)
│   ├── VISION_SAC_GUIDE.md     # ✅ SAC 구현 가이드 (~500 lines)
│   └── PROGRESS_SUMMARY.md     # ✅ 이 파일
├── models/
│   ├── __init__.py             # 🔧 수정 필요 (lint 오류)
│   └── cnn_extractor.py        # ✅ CNN Feature Extractors
├── scripts/
│   ├── test/
│   │   └── test_vision_quick.py  # ✅ Vision 전처리 검증
│   └── train/
│       └── train_vision_sac.py   # ⏳ 다음 구현
└── envs/
    └── roarm_pick_place_env_vision.py  # 🔧 완성 필요
```

---

## 📚 참고 문서

1. **VISION_RL_PLAN.md** - 전체 프로젝트 로드맵
2. **VISION_SAC_GUIDE.md** - SAC 구현 완전 가이드
3. **models/cnn_extractor.py** - CNN Feature Extractor 구현
4. **scripts/test/test_vision_quick.py** - Vision 전처리 예시

---

## 💡 Key Insights

### SAC vs PPO
| 항목 | SAC | PPO |
|------|-----|-----|
| Sample Efficiency | **500K steps** | 2-5M steps |
| Off-Policy | ✅ Yes | ❌ No |
| Replay Buffer | ✅ 100K transitions | ❌ None |
| Vision 특화 | ✅ "SAC for Pixels" | ✅ DrQv2 |
| 학습 시간 | **5-10 hours** | 10-20 hours |
| 구현 난이도 | Medium | Easy |

### CNN Architecture Choice
- **NatureCNN (512)**: 대부분의 task에 충분
- **LargerCNN (1024)**: 복잡한 환경, 다양한 물체

---

## ⚠️ 주의사항

1. **Replay Buffer**: SAC는 메모리 사용량이 큼 (100K transitions × 4 images)
2. **학습 초반**: 50K steps까지는 random exploration
3. **TensorBoard**: 항상 모니터링 (reward, success rate 확인)
4. **Checkpoint**: 매 10K steps마다 저장 (최고 성능 모델 보존)
