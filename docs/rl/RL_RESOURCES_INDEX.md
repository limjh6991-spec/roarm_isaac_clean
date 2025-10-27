# 강화학습 리소스 모음

**생성일**: 2025-10-20  
**목적**: RoArm-M3 Pick & Place 강화학습 성공을 위한 참고 자료

---

## 📚 목차

1. [핵심 논문](#1-핵심-논문)
2. [Isaac Sim 강화학습 튜토리얼](#2-isaac-sim-강화학습-튜토리얼)
3. [로봇 조작 RL 성공 사례](#3-로봇-조작-rl-성공-사례)
4. [PPO 알고리즘 가이드](#4-ppo-알고리즘-가이드)
5. [보상 함수 설계](#5-보상-함수-설계)
6. [하이퍼파라미터 튜닝](#6-하이퍼파라미터-튜닝)
7. [디버깅 및 문제 해결](#7-디버깅-및-문제-해결)
8. [실전 팁](#8-실전-팁)

---

## 1. 핵심 논문

### 1.1 PPO (Proximal Policy Optimization)
**논문**: "Proximal Policy Optimization Algorithms" (Schulman et al., 2017)
- **링크**: https://arxiv.org/abs/1707.06347
- **핵심 내용**:
  - Trust Region 방법의 단순화
  - Clipped Surrogate Objective
  - 안정적이고 샘플 효율적인 학습
  
**주요 하이퍼파라미터**:
```python
learning_rate = 3e-4
n_steps = 2048
batch_size = 64
n_epochs = 10
gamma = 0.99
gae_lambda = 0.95
clip_range = 0.2
ent_coef = 0.01  # Entropy coefficient
vf_coef = 0.5    # Value function coefficient
max_grad_norm = 0.5
```

**성공 포인트**:
- ✅ 작은 clip_range (0.1~0.2): 안정적 학습
- ✅ 적절한 entropy bonus: 탐험 유도
- ✅ GAE (Generalized Advantage Estimation) 사용

---

### 1.2 Sparse Reward 환경
**논문**: "Hindsight Experience Replay" (Andrychowicz et al., 2017)
- **링크**: https://arxiv.org/abs/1707.01495
- **핵심 아이디어**: 실패한 에피소드도 학습에 활용
- **적용**: Sparse reward 환경에서 필수

**논문**: "Learning Dexterous In-Hand Manipulation" (OpenAI, 2019)
- **링크**: https://arxiv.org/abs/1808.00177
- **핵심 내용**:
  - Curriculum Learning
  - Domain Randomization
  - Asymmetric Actor-Critic

---

### 1.3 로봇 조작 (Manipulation)
**논문**: "Deep Reinforcement Learning for Robotic Manipulation" (Gu et al., 2017)
- **링크**: https://arxiv.org/abs/1610.00633
- **핵심 내용**:
  - Sample-efficient 학습
  - Simulation-to-Reality Transfer
  - Shaped Rewards 설계

**논문**: "QT-Opt: Scalable Deep Reinforcement Learning for Vision-Based Robotic Manipulation" (Kalashnikov et al., 2018)
- **링크**: https://arxiv.org/abs/1806.10293
- **구글 로보틱스**: 대규모 로봇 데이터로 학습

---

## 2. Isaac Sim 강화학습 튜토리얼

### 2.1 공식 문서
**NVIDIA Isaac Sim RL Guide**
- **링크**: https://docs.omniverse.nvidia.com/isaacsim/latest/reinforcement_learning_tutorials.html
- **핵심 내용**:
  - Gym 환경 생성
  - VecEnv 사용법
  - Stable-Baselines3 통합

**주요 예제**:
1. Cartpole (기본)
2. Ant (사족보행)
3. Franka (로봇팔 조작)
4. Shadow Hand (손가락 조작)

---

### 2.2 Isaac Gym (Isaac Sim 전신)
**논문**: "Isaac Gym: High Performance GPU-Based Physics Simulation" (Makoviychuk et al., 2021)
- **링크**: https://arxiv.org/abs/2108.10470
- **핵심 기술**:
  - GPU-accelerated 물리 시뮬레이션
  - Massively parallel training
  - Domain Randomization

**성공 사례**:
- 로봇 손 조작 (Rubik's Cube)
- 사족보행 로봇
- 드론 제어

---

## 3. 로봇 조작 RL 성공 사례

### 3.1 Pick and Place 작업
**프로젝트**: UR5 Pick and Place with Isaac Sim
- **GitHub**: https://github.com/NVIDIA-Omniverse/IsaacGymEnvs
- **핵심 전략**:
  ```python
  # Shaped Reward 예제
  reward = 0.0
  
  # 1. Reaching reward (거리 기반)
  reward += -0.1 * ee_to_cube_dist
  
  # 2. Grasping reward (sparse)
  if grasped:
      reward += 10.0
  
  # 3. Lifting reward (높이 기반)
  reward += 5.0 * max(0, cube_height - initial_height)
  
  # 4. Goal reaching (sparse)
  if cube_to_goal_dist < 0.05:
      reward += 50.0
  
  # 5. Time penalty
  reward -= 0.01
  ```

**학습 시간**: 500K~1M steps (약 6~12시간, GPU 기준)

---

### 3.2 Franka Panda 조작
**논문**: "Learning to Manipulate Deformable Objects without Demonstrations" (Lin et al., 2022)
- **핵심 전략**:
  - Curriculum Learning: 쉬운 → 어려운 작업
  - Shaped Rewards: Dense guidance
  - Domain Randomization: 물체 크기/질량 변경

**성공 요인**:
- ✅ 적절한 observation space (25~30 dim)
- ✅ Normalized inputs (VecNormalize)
- ✅ Action smoothing (low-pass filter)

---

## 4. PPO 알고리즘 가이드

### 4.1 Stable-Baselines3 PPO
**공식 문서**: https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html

**기본 사용법**:
```python
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize

# 환경 생성
env = VecNormalize(env, norm_obs=True, norm_reward=True)

# PPO 모델 생성
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

# 학습
model.learn(total_timesteps=1000000)
```

---

### 4.2 하이퍼파라미터 권장값

| 파라미터 | 일반적 범위 | 로봇 조작 권장 | 설명 |
|---------|-----------|--------------|------|
| **learning_rate** | 1e-5 ~ 1e-3 | **3e-4** | Adam optimizer |
| **n_steps** | 128 ~ 4096 | **2048** | Rollout buffer size |
| **batch_size** | 32 ~ 256 | **64** | Minibatch size |
| **n_epochs** | 3 ~ 15 | **10** | Update epochs |
| **gamma** | 0.9 ~ 0.999 | **0.99** | Discount factor |
| **gae_lambda** | 0.9 ~ 0.99 | **0.95** | GAE parameter |
| **clip_range** | 0.1 ~ 0.3 | **0.2** | PPO clip range |
| **ent_coef** | 0.0 ~ 0.1 | **0.01** | Entropy bonus |
| **vf_coef** | 0.5 ~ 1.0 | **0.5** | Value function loss |
| **max_grad_norm** | 0.3 ~ 1.0 | **0.5** | Gradient clipping |

---

## 5. 보상 함수 설계

### 5.1 Shaped Reward 설계 원칙

**1. Dense Guidance (밀집 안내)**
```python
# ❌ Sparse만 사용 (학습 어려움)
reward = 100 if success else 0

# ✅ Shaped + Sparse (학습 용이)
reward = -0.1 * distance  # Dense guidance
if success:
    reward += 100  # Sparse bonus
```

**2. 적절한 스케일**
```python
# Reward 범위: -1 ~ +100 정도 권장
reaching_reward = -0.1 * ee_to_cube_dist  # [-0.5, 0]
grasp_reward = 10.0 if grasped else 0     # [0, 10]
success_reward = 100.0 if success else 0  # [0, 100]
```

**3. 1회성 보상 (One-time)**
```python
# ❌ 매 step마다 지급 (over-rewarding)
if grasped:
    reward += 10.0

# ✅ 첫 달성 시만 지급
if grasped and not self.first_grasp_achieved:
    reward += 10.0
    self.first_grasp_achieved = True
```

---

### 5.2 RoArm M3 최적 보상 구조 (현재 구현)

```python
# 1. REACH (+5): EE → 큐브 5cm 이내
if ee_to_cube_dist < 0.05 and not self.first_reach:
    reward += 5.0
    self.first_reach = True

# 2. GRIP (+10): 유효 그립 3프레임 연속
grasp_valid = (
    ee_to_cube_dist < 0.08 and
    gripper_width < 0.02 and
    cube_pos[2] > 0.03
)
if grasp_valid:
    self.grip_frames += 1
    if self.grip_frames >= 3 and not self.valid_grip:
        reward += 10.0
        self.valid_grip = True

# 3. LIFT (+15): 큐브 5cm 상승
if cube_height > 0.05 and not self.lifted:
    reward += 15.0
    self.lifted = True

# 4. GOAL (+20): 큐브 → 타겟 8cm 이내
if cube_to_goal_dist < 0.08 and not self.goal_near:
    reward += 20.0
    self.goal_near = True

# 5. SUCCESS (+100): 타겟 5cm, 5프레임 연속
if cube_to_goal_dist < 0.05:
    self.success_frames += 1
    if self.success_frames >= 5:
        reward += 100.0
        done = True

# 6. Time penalty
reward -= 0.01
```

---

## 6. 하이퍼파라미터 튜닝

### 6.1 학습 불안정 해결

**증상**: Loss 발산, Reward 진동
**해결책**:
- ✅ Learning rate 감소: 3e-4 → 1e-4
- ✅ Clip range 감소: 0.2 → 0.1
- ✅ Batch size 증가: 64 → 128
- ✅ Entropy coefficient 증가: 0.01 → 0.02

---

### 6.2 학습 느림 해결

**증상**: 1M steps 후에도 성능 미개선
**해결책**:
- ✅ Shaped reward 추가
- ✅ Curriculum learning 도입
- ✅ Observation normalization 확인
- ✅ 초기 자세 랜덤화

---

### 6.3 Over-fitting 방지

**증상**: Training reward는 높지만 test 실패
**해결책**:
- ✅ Domain randomization:
  ```python
  cube_size = np.random.uniform(0.025, 0.035)
  cube_mass = np.random.uniform(0.01, 0.05)
  cube_friction = np.random.uniform(0.5, 1.5)
  ```
- ✅ Observation noise 추가
- ✅ Action noise 추가

---

## 7. 디버깅 및 문제 해결

### 7.1 GRIP 마일스톤 미달성 (현재 문제!)

**증상**: 100K steps, REACH 12회, GRIP 0회

**원인 분석**:
1. ✅ **URDF 그리퍼**: 이미 Prismatic (정상)
2. ⚠️ **Grasp 조건 너무 엄격**:
   ```python
   # 현재 (너무 엄격)
   grasp_valid = (
       ee_to_cube_dist < 0.08 and  # 8cm
       gripper_width < 0.02 and    # 2cm (거의 닫힘)
       cube_pos[2] > 0.03          # 3cm 높이
   )
   ```
3. ⚠️ **그리퍼 제어 문제**: Action → 그리퍼 동작 확인 필요

**해결책**:
- [ ] Grasp 조건 완화: gripper_width < 0.03
- [ ] Contact force 감지 추가
- [ ] 그리퍼 action 디버깅

---

### 7.2 일반적인 문제들

**문제 1: Reward가 음수에서 증가 안함**
- 원인: Time penalty가 너무 큼
- 해결: Time penalty 감소 (-0.1 → -0.01)

**문제 2: 로봇이 이상한 자세**
- 원인: Joint limit 초과 또는 Self-collision
- 해결: Joint limit 확인, Collision geometry 개선

**문제 3: 큐브가 튕겨나감**
- 원인: 물리 파라미터 (restitution, friction)
- 해결:
  ```python
  cube_restitution = 0.0  # 탄성 계수 낮춤
  cube_friction = 1.0     # 마찰 계수 높임
  ```

---

## 8. 실전 팁

### 8.1 학습 시작 전 체크리스트

- [ ] **URDF 검증**: 그리퍼 동작 확인
- [ ] **Observation 확인**: 25~30 dim, normalized
- [ ] **Action 범위**: [-1, 1] scaled
- [ ] **보상 함수**: Shaped + Sparse
- [ ] **초기 자세**: 안정적인가?
- [ ] **물리 파라미터**: Friction, Damping 적절한가?
- [ ] **VecNormalize**: obs + reward 정규화

---

### 8.2 학습 중 모니터링

**필수 지표**:
```python
# TensorBoard에서 확인
- ep_rew_mean: 평균 에피소드 보상 (증가 추세?)
- ep_len_mean: 평균 에피소드 길이 (감소 추세?)
- value_loss: Value function loss (안정적?)
- policy_loss: Policy loss (안정적?)
- entropy_loss: Entropy (탐험 유지?)
```

**마일스톤 추적**:
```python
# 로그 파일에서 추출
grep "REACH\|GRIP\|LIFT\|GOAL\|SUCCESS" training.log
```

---

### 8.3 성공 기준

**RoArm M3 Pick & Place**:
- **Phase 0 (Easy)**: 성공률 ≥60% (200 에피소드)
- **Phase 1 (Medium)**: 성공률 ≥40%
- **Phase 2 (Hard)**: 성공률 ≥20%

**학습 시간 예상**:
- Phase 0: 500K steps (약 6시간, RTX 5090)
- Phase 1: 1M steps (약 12시간)
- Phase 2: 2M steps (약 24시간)

---

### 8.4 Curriculum Learning 전략

**자동 승급**:
```python
if success_rate >= 0.6 and curriculum_phase == 0:
    curriculum_phase = 1
    print("🎓 Phase 1 진입!")
```

**수동 조정** (권장):
```python
# 충분히 학습 후 수동 승급
# 예: 500K steps에서 Phase 0 → Phase 1
```

---

## 9. 참고 자료

### 9.1 GitHub 리포지토리
- **IsaacGymEnvs**: https://github.com/NVIDIA-Omniverse/IsaacGymEnvs
- **Stable-Baselines3**: https://github.com/DLR-RM/stable-baselines3
- **rl-baselines3-zoo**: https://github.com/DLR-RM/rl-baselines3-zoo

### 9.2 블로그 & 튜토리얼
- **OpenAI Spinning Up**: https://spinningup.openai.com/
- **Deep RL Course**: https://huggingface.co/deep-rl-course/unit0/introduction
- **CleanRL**: https://github.com/vwxyzjn/cleanrl

### 9.3 비디오 강의
- **DeepMind x UCL RL Course**: https://www.deepmind.com/learning-resources/reinforcement-learning-lecture-series-2021
- **Berkeley CS 285**: http://rail.eecs.berkeley.edu/deeprlcourse/

---

## 10. 다음 단계 (RoArm M3)

### 10.1 즉시 실행 (우선순위 높음)
1. **50K 테스트 학습**
   - 목표: GRIP 마일스톤 1회 이상
   - 시간: 약 30분~1시간
   - 명령어:
     ```bash
     cd ~/roarm_isaac_clean/scripts/rl
     PYTHONUNBUFFERED=1 ~/isaacsim/python.sh train_dense_reward.py --timesteps 50000
     ```

2. **GRIP 조건 디버깅** (50K 실패 시)
   - Grasp 조건 완화
   - Contact force 확인
   - 그리퍼 action 디버깅

---

### 10.2 중기 목표 (1주일)
1. **Phase 0 완료**: 성공률 60% 달성
2. **Curriculum Phase 1 진입**
3. **Domain Randomization 추가**

---

### 10.3 장기 목표 (1개월)
1. **Phase 2 완료**: 다양한 물체 파지
2. **Real Robot Transfer**: Sim-to-Real
3. **Multi-task Learning**: 다양한 조작 작업

---

**작성자**: GitHub Copilot  
**최종 업데이트**: 2025-10-20  
**다음 갱신**: 50K 학습 완료 후
