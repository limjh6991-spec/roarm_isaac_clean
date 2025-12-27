# VRL 알고리즘 및 프레임워크 권장사항

> **작성일**: 2025-12-27  
> **대상**: RoArm-M3 Vision-based Manipulation

---

## 🎯 현재 프로젝트 방향 검토

### 현재 설정
- **알고리즘**: SAC (Soft Actor-Critic)
- **프레임워크**: Stable Baselines3
- **Policy**: CnnPolicy (Vision 입력)
- **환경**: 커스텀 Gymnasium 환경

### 결론: ✅ 적합한 방향

SAC + Stable Baselines3 조합은 **Vision-based Manipulation**에 적합합니다:
- SAC는 **sample efficiency**가 높아 vision 기반 학습에 효과적
- 연속 액션 공간(continuous action space)에 최적화
- 실제 로봇 fine-tuning에 용이

---

## 📊 알고리즘 비교

| 항목 | SAC | PPO |
|------|-----|-----|
| **Sample Efficiency** | ⭐⭐⭐⭐⭐ 높음 | ⭐⭐⭐ 보통 |
| **병렬 환경 확장성** | ⭐⭐⭐ 보통 | ⭐⭐⭐⭐⭐ 우수 |
| **학습 안정성** | ⭐⭐⭐⭐ 좋음 | ⭐⭐⭐⭐⭐ 매우 좋음 |
| **섬세한 조작** | ⭐⭐⭐⭐⭐ 우수 | ⭐⭐⭐⭐ 좋음 |
| **Sim-to-Real** | ⭐⭐⭐⭐⭐ 우수 | ⭐⭐⭐⭐ 좋음 |

### 권장 사항

1. **시뮬레이터 내 학습**: SAC 또는 PPO 모두 가능
   - 대규모 병렬 학습 → PPO
   - 단일 환경 또는 적은 병렬화 → SAC

2. **실제 로봇 Fine-tuning**: SAC 권장
   - 적은 데이터로 학습 가능
   - 안전한 exploration (soft 방식)

3. **Vision 기반 Manipulation**: SAC 권장
   - 이미지 처리 시간으로 인해 sample efficiency 중요
   - CnnPolicy와 잘 호환

---

## 🔧 프레임워크 선택

### Option 1: Stable Baselines3 (현재 사용중) ✅

```python
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv

model = SAC(
    policy="CnnPolicy",
    env=env,
    learning_rate=3e-4,
    buffer_size=100000,
    batch_size=256,
    ent_coef="auto",
    verbose=1,
)
```

**장점**:
- 문서화 잘 됨
- 커뮤니티 지원 활발
- 다양한 알고리즘 지원

**단점**:
- Isaac Lab과의 통합 최적화 부족
- 대규모 병렬화 제한적

### Option 2: Isaac Lab 내장 RL

Isaac Lab 2.3에서는 다음 라이브러리 지원:
- **rl-games**: NVIDIA 최적화
- **rsl-rl**: 로봇 학습 특화
- **skrl**: 경량화된 RL 라이브러리

```bash
# Isaac Lab 설치 시 포함
./isaaclab.sh --install
```

**장점**:
- Isaac Sim과 최적화된 통합
- 대규모 병렬 학습 지원
- GPU 가속 최적화

**단점**:
- Isaac Lab 설치 필요
- 학습 곡선

### 권장: Stable Baselines3 유지

현재 프로젝트 규모와 목표에서는 Stable Baselines3이 적합합니다.
Isaac Lab은 수천 개의 병렬 환경이 필요한 대규모 학습에 더 적합합니다.

---

## 📝 개선 권장사항

### 1. 하이퍼파라미터 튜닝

```python
# 권장 SAC 설정 (Vision Manipulation)
SAC(
    learning_rate=3e-4,
    buffer_size=100_000,
    learning_starts=1000,
    batch_size=256,
    tau=0.005,
    gamma=0.99,
    train_freq=1,
    gradient_steps=1,
    ent_coef="auto",
    target_entropy="auto",
)
```

### 2. Observation Space 개선

현재 RGB-D (4, 84, 84)는 좋지만, 추가 고려사항:
- **Proprioception** 추가: 로봇 관절 상태 포함
- **Frame Stacking**: 여러 프레임 연결로 시간 정보 제공

```python
from gymnasium.wrappers import FrameStack
env = FrameStack(env, num_stack=4)
```

### 3. Reward 설계

현재 reward는 기본적이며, 다음 추가 권장:
- **Shaping reward**: 점진적인 목표 달성 보상
- **Sparse + Dense 혼합**: 최종 성공 보너스 + 진행 보상

---

## 🚀 다음 단계

1. **환경 수정**: Isaac Sim 5.1 API에 맞게 업데이트
2. **테스트 환경 검증**: 로봇 제어 및 카메라 동작 확인
3. **학습 시작**: SAC로 초기 학습 진행
4. **하이퍼파라미터 튜닝**: 결과에 따라 조정
5. **Sim-to-Real**: 실제 로봇에 정책 배포

---

*이 문서는 2025-12-27 기준으로 작성되었습니다.*
