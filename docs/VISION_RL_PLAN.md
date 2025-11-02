# Vision-Based RL 프로젝트 계획

**작성일**: 2025-11-02  
**목표**: RoArm-M3 + D405 카메라를 활용한 Vision-based Pick & Place RL

---

## 📊 현재 상태 (Phase 1 완료)

### ✅ 완료된 작업
1. **Hardware Integration**
   - Intel RealSense D405 카메라 추가 (72g, 191KB STL)
   - URDF에 카메라 통합 (gripper_link에 20mm 앞 장착)
   - 카메라 무게 보상 (Stiffness 5000, Damping 100, Effort 200)

2. **Camera Sensor**
   - IsaacLab Camera 센서 구현
   - RGB + Depth 이미지 캡처 기능
   - 8개 동작 시퀀스 테스트 완료
   - 출력: `output/camera_images/YYYYMMDD_HHMMSS/*.png`

3. **Simulation Stability**
   - Joint Limit 강제 적용 (torch.clamp)
   - PD Controller 튜닝
   - 중력 보상 확인

### 📁 주요 파일
- `scripts/test/test_roarm_with_camera_isaaclab.py` (606 lines) - 카메라 테스트
- `assets/roarm_m3/urdf/roarm_m3_with_camera_correct.urdf` - 카메라 통합 URDF
- `assets/roarm_m3/meshes/d405/d405.stl` (191KB) - Intel 공식 파일
- `envs/roarm_pick_place_env_vision.py` (360 lines) - Vision Env (초안)

---

## 🎯 Phase 2: Vision-Based RL 구현

### 목표
RGB-D 이미지를 입력으로 받아 Pick & Place 작업을 학습하는 RL Agent 개발

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Vision RL Pipeline                       │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Camera (D405)                                               │
│    │                                                          │
│    ├─> RGB (640×480)  ──┐                                   │
│    └─> Depth (640×480) ─┤                                   │
│                          │                                    │
│                          ▼                                    │
│                   Preprocessing                               │
│                   - Resize: 84×84                            │
│                   - Normalize: [0, 1]                        │
│                   - Stack: (4, 84, 84)                       │
│                          │                                    │
│                          ▼                                    │
│                   CNN Feature Extractor                       │
│                   - Conv1: 32 filters (8×8, stride 4)       │
│                   - Conv2: 64 filters (4×4, stride 2)       │
│                   - Conv3: 64 filters (3×3, stride 1)       │
│                   - Flatten: 3136 → FC: 512                 │
│                          │                                    │
│                          ▼                                    │
│                   Policy Network (PPO)                        │
│                   - Actor: 512 → 256 → 7 (actions)          │
│                   - Critic: 512 → 256 → 1 (value)           │
│                          │                                    │
│                          ▼                                    │
│                   Action (7 DOF)                             │
│                   [j1, j2, j3, j4, j5, j6, gripper]         │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

---

## 📋 구현 계획

### Step 1: Environment Setup (2-3일)

**목표**: Vision Env 완성 및 테스트

**작업**:
1. `roarm_pick_place_env_vision.py` 완성
   - [x] Camera 센서 통합
   - [ ] RGB-D 전처리 (resize, normalize)
   - [ ] Observation space 정의 (4, 84, 84)
   - [ ] Action space 정의 (7 DOF)
   - [ ] Reset 로직 (랜덤 object/goal 위치)

2. Reward Function 구현
   - [ ] Dense reward (distance-based)
   - [ ] Vision bonus (object in view)
   - [ ] Grasp detection (contact sensor)
   - [ ] Task completion (lift + place)

3. 테스트
   - [ ] `gym.check_env()` 통과
   - [ ] Random policy로 rollout
   - [ ] 이미지 시각화 확인

**출력**:
- `envs/roarm_pick_place_env_vision.py` (완성)
- `scripts/test/test_vision_env.py` (환경 테스트)

---

### Step 2: CNN Feature Extractor (1-2일)

**목표**: Custom CNN 구현

**작업**:
1. NatureCNN 구현 (Stable-Baselines3 스타일)
   ```python
   class NatureCNN(nn.Module):
       def __init__(self):
           super().__init__()
           self.cnn = nn.Sequential(
               nn.Conv2d(4, 32, kernel_size=8, stride=4),
               nn.ReLU(),
               nn.Conv2d(32, 64, kernel_size=4, stride=2),
               nn.ReLU(),
               nn.Conv2d(64, 64, kernel_size=3, stride=1),
               nn.ReLU(),
               nn.Flatten(),
           )
           self.linear = nn.Linear(3136, 512)
   ```

2. Feature visualization
   - [ ] Gradient-CAM for attention
   - [ ] Feature map 시각화

**출력**:
- `models/cnn_extractor.py`
- `scripts/test/test_cnn.py` (Feature 추출 테스트)

---

### Step 3: SAC Training (2주) ⭐ **변경됨!**

**목표**: Vision-based SAC 학습 (Sample Efficient!)

**작업**:
1. Training script 작성
   ```python
   from stable_baselines3 import SAC
   from stable_baselines3.common.callbacks import CheckpointCallback
   
   model = SAC(
       "CnnPolicy",
       env,
       policy_kwargs=dict(
           features_extractor_class=NatureCNN,
           features_extractor_kwargs=dict(features_dim=512),
       ),
       buffer_size=100_000,  # Replay buffer
       learning_rate=3e-4,
       batch_size=256,
       tau=0.005,
       gamma=0.99,
       train_freq=1,
       gradient_steps=1,
       verbose=1,
       tensorboard_log="./logs/vision_sac"
   )
   ```

2. Hyperparameter Tuning
   - [ ] Learning rate: [1e-4, 3e-4, 1e-3]
   - [ ] Batch size: [128, 256, 512]
   - [ ] Buffer size: [50K, 100K, 200K]
   - [ ] Network size: [256, 512, 1024]

3. Monitoring
   - [ ] TensorBoard logging
   - [ ] Episode reward tracking
   - [ ] Success rate tracking
   - [ ] Replay buffer utilization

**장점 (PPO 대비)**:
- ✅ **3-5배 더 적은 샘플**: 500K steps vs 2-5M steps
- ✅ **Off-Policy**: 데이터 재사용 가능
- ✅ **안정성**: Entropy regularization
- ✅ **Vision 특화**: SAC for Pixels 검증됨

**출력**:
- `scripts/train/train_vision_sac.py`
- `logs/vision_sac/` (TensorBoard logs)
- `checkpoints/vision_sac/` (모델 체크포인트)

---

### Step 4: Evaluation & Visualization (2일)

**목표**: 학습된 Policy 평가

**작업**:
1. Evaluation script
   - [ ] 100 episodes 평가
   - [ ] Success rate 계산
   - [ ] Video recording

2. Visualization
   - [ ] Episode rollout video
   - [ ] Attention map overlay
   - [ ] Trajectory plotting

**출력**:
- `scripts/eval/eval_vision_ppo.py`
- `videos/vision_ppo/` (에피소드 영상)
- `docs/VISION_RL_RESULTS.md` (결과 보고서)

---

## 🔧 기술 스택

### Core Libraries
- **Isaac Lab**: Simulation environment
- **Stable-Baselines3**: RL algorithms (PPO)
- **PyTorch**: CNN implementation
- **OpenCV**: Image preprocessing
- **TensorBoard**: Training monitoring

### Camera
- **Intel RealSense D405**
- **Resolution**: RGB 640×480, Depth 640×480
- **FOV**: 87° (Horizontal)
- **Range**: 0.07m ~ 10m

### RL Configuration
- **Algorithm**: SAC (Soft Actor-Critic) ⭐ **변경됨!**
- **Observation**: (4, 84, 84) - RGB-D
- **Action**: (7,) - Continuous joint control
- **Reward**: Dense + Vision bonus
- **Horizon**: 1000 steps (16.7 seconds @ 60Hz)
- **Buffer Size**: 100K transitions
- **Sample Efficiency**: 500K steps (PPO 대비 3-5배 적음)

---

## 📊 예상 성능

### Baseline (Random Policy)
- Success Rate: ~0%
- Avg Episode Reward: -500

### Target (Trained Policy - SAC)
- Success Rate: >70% (100 episodes)
- Avg Episode Reward: >100
- Training Time: ~5-10 hours (RTX 3090)
- **Sample Efficiency**: 500K steps (PPO: 2-5M steps)

---

## 🚀 다음 단계

### Immediate (오늘)
1. ✅ Vision RL 계획 작성
2. ⏳ Vision Env 완성 (`roarm_pick_place_env_vision.py`)
3. ⏳ 환경 테스트 스크립트 작성

### This Week
4. CNN Feature Extractor 구현
5. PPO Training script 작성
6. 첫 학습 실행 (Baseline)

### Next Week
7. Hyperparameter tuning
8. 평가 및 시각화
9. 문서화 및 최종 보고서

---

## 📚 참고 자료

### Papers
- **Playing Atari with Deep RL** (Mnih et al., 2013) - DQN + CNN
- **Proximal Policy Optimization** (Schulman et al., 2017) - PPO
- **QT-Opt**: Scalable Deep RL for Vision-Based Robotic Manipulation (Kalashnikov et al., 2018)

### Code References
- Stable-Baselines3 CnnPolicy: https://github.com/DLR-RM/stable-baselines3
- Isaac Lab Camera: https://isaac-sim.github.io/IsaacLab/source/api/lab/omni.isaac.lab.sensors.html#camera

### Documentation
- `resources/vision_rl/README.md` - Vision RL 자료 모음
- `docs/CAMERA_TEST_SUMMARY.md` - Phase 1 카메라 테스트 결과

---

## ✅ Success Criteria

1. **Environment**
   - [ ] `gym.check_env()` 통과
   - [ ] Random policy rollout 성공
   - [ ] RGB-D 이미지 올바르게 출력

2. **Training**
   - [ ] PPO 학습 시작
   - [ ] TensorBoard 정상 동작
   - [ ] Episode reward 증가 확인

3. **Evaluation**
   - [ ] Success rate >70%
   - [ ] Video recording 성공
   - [ ] Attention map 시각화

4. **Documentation**
   - [ ] 코드 주석 완비
   - [ ] README 작성
   - [ ] 결과 보고서 작성

---

**Last Updated**: 2025-11-02  
**Status**: Planning Complete ✅  
**Next Action**: Vision Env 완성
