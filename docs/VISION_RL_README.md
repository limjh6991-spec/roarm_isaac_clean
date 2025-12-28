# RoArm-M3 Vision RL Implementation

Intel RealSense D405 카메라를 RoArm-M3에 통합하고 Vision-based Reinforcement Learning을 구현합니다.

## 📦 구조

```
assets/
├── cameras/
│   └── realsense_d405.urdf              # D405 카메라 URDF
└── roarm_m3/
    ├── meshes/roarm_m3/                 # STL 메시 (8개)
    └── urdf/
        ├── roarm_m3.generated.urdf      # 기본 로봇 URDF
        └── roarm_m3_with_d405.urdf      # 카메라 통합 URDF ✨

envs/
└── roarm_pick_place_env_vision.py       # Vision RL Environment

scripts/
├── test/
│   ├── test_camera_urdf.py              # URDF 테스트
│   └── test_camera_capture.py           # RGB-D 캡처 테스트
└── train_vision_rl.py                   # Vision RL 학습

logs/
├── camera_test/                         # 캡처 이미지
└── vision_rl/                           # 학습 로그
    ├── models/                          # 체크포인트
    └── tensorboard/                     # TensorBoard 로그
```

## 🎥 Intel RealSense D405 스펙

- **크기**: 42mm × 42mm × 23mm
- **무게**: 60g
- **FOV**: 87° × 58°
- **작동 거리**: 7-50cm (최적: 7cm)
- **정밀도**: Sub-millimeter @ 7cm
- **출력**: RGB (1280×720) + Depth (1280×720)

## 🤖 카메라 마운트

```xml
<joint name="camera_mount_joint" type="fixed">
  <parent link="gripper_link"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.02" rpy="0 0.5236 0"/>
</joint>
```

- **위치**: 그리퍼 앞 5cm, 위 2cm
- **방향**: 30도 하향 (작업 공간 관측)
- **타입**: Fixed joint (그리퍼와 함께 이동)

## 🚀 사용법

### 1️⃣ URDF 테스트

통합 URDF가 Isaac Sim에서 올바르게 로드되는지 확인:

```bash
~/isaacsim/python.sh scripts/test/test_camera_urdf.py
```

**확인 사항**:
- ✅ URDF 로딩 성공
- ✅ 카메라 프레임 5개 존재
- ✅ 카메라가 그리퍼에 부착
- ✅ 로봇 모션 시 카메라 추종

### 2️⃣ RGB-D 캡처 테스트

카메라에서 RGB + Depth 이미지 캡처:

```bash
~/isaacsim/python.sh scripts/test/test_camera_capture.py
```

**조작법**:
- `s` 키: RGB-D 이미지 캡처 및 저장
- `q` 키: 종료

**출력**:
- `logs/camera_test/rgb_XXX.png`: RGB 이미지 (256×256)
- `logs/camera_test/depth_XXX.png`: Depth 시각화 (256×256)
- `logs/camera_test/depth_raw_XXX.npy`: Raw Depth 데이터

### 3️⃣ Vision RL 학습

RGB-D 이미지를 관측으로 사용하는 Pick and Place 학습:

```bash
~/isaacsim/python.sh scripts/train_vision_rl.py
```

**학습 설정**:
- **알고리즘**: PPO (Stable Baselines3)
- **Policy**: CnnPolicy (Nature CNN)
- **Observation**: RGB-D (4, 84, 84)
- **Action**: Joint positions (7,)
- **총 Steps**: 100,000
- **체크포인트**: 10,000 steps마다

**TensorBoard 모니터링**:
```bash
tensorboard --logdir logs/vision_rl/tensorboard
```

## 📊 Observation Space

```python
# Shape: (4, 84, 84)
# Channels: [R, G, B, Depth]
# Range: [0, 1]
# Source: D405 camera on gripper

observation = {
    "rgb": (3, 84, 84),      # RGB 이미지
    "depth": (1, 84, 84),    # Depth 이미지 (10cm ~ 2m)
}
```

## 🎯 Reward 구조

```python
# Phase 0: Reach object
reach_reward = -distance_to_object * 10.0
vision_bonus = +0.1 if object_in_view else 0.0
phase_transition = +10.0 if distance < 5cm

# Phase 1: Grasp object
grasp_reward = +20.0 if grasped else -0.1

# Phase 2: Lift object
lift_reward = object_height * 50.0
phase_transition = +30.0 if height > 20cm

# Phase 3: Place object
place_reward = -distance_to_goal * 20.0
success_bonus = +100.0 if distance < 5cm

# Penalty
motion_penalty = -0.001 * ||joint_velocity||
```

## 🧠 Policy Network

**Nature CNN** (Mnih et al., 2015):

```
Input: (4, 84, 84) RGB-D
  ↓
Conv1: 32 filters, 8×8, stride 4 → ReLU
  ↓
Conv2: 64 filters, 4×4, stride 2 → ReLU
  ↓
Conv3: 64 filters, 3×3, stride 1 → ReLU
  ↓
Flatten → 3136 features
  ↓
FC1: 512 → ReLU
  ↓
Actor Head: 256 → 256 → 7 (actions)
Critic Head: 256 → 256 → 1 (value)
```

## 📈 학습 프로세스

```
Episode 시작
  ↓
1. Reset: 로봇 홈 포지션, 랜덤 물체/목표 위치
  ↓
2. RGB-D 캡처 (256×256 → 84×84)
  ↓
3. Policy 실행: CNN → Action (7 joint positions)
  ↓
4. Simulation Step (1/60 s)
  ↓
5. Reward 계산 (phase-based dense reward)
  ↓
6. Phase 전환 확인 (reach → grasp → lift → place)
  ↓
7. Termination 체크 (성공 or 실패 or 타임아웃)
  ↓
8. PPO 업데이트 (2048 steps마다)
  ↓
반복 (100,000 steps)
```

## 🔧 파라미터 튜닝

### Environment
```python
episode_length_s = 10.0        # 에피소드 길이
camera_resolution = (256, 256)  # 카메라 해상도
camera_frequency = 20           # 카메라 FPS
observation_size = (4, 84, 84)  # 관측 크기
```

### PPO
```python
learning_rate = 3e-4           # 학습률
n_steps = 2048                 # Steps per update
batch_size = 64                # Batch size
n_epochs = 10                  # Epochs per update
gamma = 0.99                   # Discount factor
gae_lambda = 0.95              # GAE parameter
clip_range = 0.2               # PPO clip range
ent_coef = 0.01                # Entropy coefficient
```

## 📁 로그 구조

```
logs/
├── camera_test/
│   ├── rgb_001.png            # RGB 캡처
│   ├── depth_001.png          # Depth 시각화
│   └── depth_raw_001.npy      # Raw Depth
│
└── vision_rl/
    ├── models/
    │   ├── roarm_vision_ppo_10000_steps.zip
    │   ├── roarm_vision_ppo_20000_steps.zip
    │   └── roarm_vision_ppo_final.zip
    │
    └── tensorboard/
        └── PPO_1/
            └── events.out.tfevents.*
```

## 🎯 성공 기준

- ✅ **Reach**: 그리퍼가 물체 5cm 이내 도달
- ✅ **Grasp**: 물체를 성공적으로 파지
- ✅ **Lift**: 물체를 20cm 이상 들어올림
- ✅ **Place**: 물체를 목표 위치 5cm 이내 배치

## 🔍 디버깅

### 1. URDF 로딩 실패
```bash
# 메시 파일 확인
ls -lh assets/roarm_m3/meshes/roarm_m3/

# URDF 경로 확인
grep "filename" assets/roarm_m3/urdf/roarm_m3_with_d405.urdf
```

### 2. 카메라 프레임 없음
```bash
# 카메라 프레임 확인
grep "camera_" assets/roarm_m3/urdf/roarm_m3_with_d405.urdf
```

### 3. RGB-D 캡처 실패
```python
# Camera.update() 호출 확인
camera.update(dt)

# 데이터 타입 확인
print(type(camera.data.rgb))    # torch.Tensor or numpy.ndarray
print(camera.data.rgb.shape)    # (1, 256, 256, 4)
```

### 4. 학습 안정성
```python
# Reward scale 조정
reach_reward *= 0.1  # 너무 큰 reward는 학습 불안정

# Observation normalization
rgb = rgb / 255.0
depth = (depth - 0.1) / 1.9

# Action clipping
actions = torch.clamp(actions, -1.0, 1.0)
```

## 📚 참고 자료

### Papers
- [Playing Atari with Deep RL (Mnih et al., 2013)](https://arxiv.org/abs/1312.5602)
- [Human-level control through deep RL (Mnih et al., 2015)](https://www.nature.com/articles/nature14236)
- [Mastering Diverse Domains through World Models (Hafner et al., 2023)](https://arxiv.org/abs/2301.04104)

### Documentation
- [Isaac Sim Camera Sensor](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_camera.html)
- [Stable Baselines3 CnnPolicy](https://stable-baselines3.readthedocs.io/en/master/guide/custom_policy.html)
- [Intel RealSense D405](https://www.intelrealsense.com/depth-camera-d405/)

## 🆘 Troubleshooting

### Import 에러
```bash
# Isaac Sim Python 사용
~/isaacsim/python.sh your_script.py

# 일반 Python 사용 불가 ❌
python your_script.py
```

### Out of Memory
```python
# Batch size 줄이기
batch_size = 32  # 64 → 32

# Observation size 줄이기
observation_size = (4, 64, 64)  # (4, 84, 84) → (4, 64, 64)
```

### 학습 느림
```python
# Camera FPS 줄이기
camera_frequency = 10  # 20 → 10

# Simulation substeps 줄이기
substeps = 1  # 2 → 1
```

## 📝 TODO

- [ ] Data augmentation (random crop, color jitter)
- [ ] DrQ-v2 implementation
- [ ] Multi-task learning (다양한 물체)
- [ ] Domain randomization (조명, 배경)
- [ ] Real robot transfer

---

**Created**: 2024-11-02  
**Author**: RoArm Isaac Clean Team  
**Status**: Phase 2 - Vision RL Implementation
