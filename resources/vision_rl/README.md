# Vision-Based RL Resources

Phase 2를 위한 Vision-Based Reinforcement Learning 자료 모음입니다.

## 목표
- RGB-D 입력을 활용한 RL Policy 학습
- CNN 기반 Policy Network 구조 이해
- Isaac Sim Camera Sensor 통합

---

## 1. Stable-Baselines3 CnnPolicy

### GitHub
- **Repository**: https://github.com/DLR-RM/stable-baselines3
- **Documentation**: https://stable-baselines3.readthedocs.io/en/master/

### CnnPolicy 개요
- **지원 알고리즘**: PPO, A2C, DQN, SAC
- **입력 형식**: (channels, height, width) - PyTorch 형식
- **기본 CNN 구조**:
  ```python
  # NatureCNN (Atari 스타일)
  Conv2d(in_channels, 32, kernel=8, stride=4)
  Conv2d(32, 64, kernel=4, stride=2)
  Conv2d(64, 64, kernel=3, stride=1)
  Flatten()
  Linear(64 * 7 * 7, 512)
  ```

### 사용 예시
```python
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

# Vision-based Environment
env = VisionEnv(obs_type="rgb")  # (84, 84, 3)

# CnnPolicy로 PPO 학습
model = PPO(
    "CnnPolicy",
    env,
    policy_kwargs=dict(
        features_extractor_class=CustomCNN,
        features_extractor_kwargs=dict(features_dim=256),
    ),
    n_steps=2048,
    learning_rate=3e-4,
    verbose=1
)
model.learn(total_timesteps=1_000_000)
```

### Custom CNN 예시
```python
import torch.nn as nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

class CustomCNN(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=256):
        super().__init__(observation_space, features_dim)
        n_input_channels = observation_space.shape[0]  # RGB: 3, RGBD: 4
        
        self.cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.Flatten(),
        )
        
        # Compute shape after CNN
        with torch.no_grad():
            n_flatten = self.cnn(
                torch.zeros(1, *observation_space.shape)
            ).shape[1]
        
        self.linear = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU(),
        )

    def forward(self, observations):
        return self.linear(self.cnn(observations))
```

---

## 2. Isaac Sim Camera Sensor

### 공식 문서
- **URL**: https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/ext_omni_isaac_sensor.html

### Camera Sensor 종류
1. **RGB Camera**: 색상 이미지 (H x W x 3)
2. **Depth Camera**: 깊이 이미지 (H x W x 1)
3. **Segmentation Camera**: 시멘틱 세그멘테이션
4. **Instance Segmentation Camera**: 인스턴스 세그멘테이션
5. **Bounding Box Camera**: 2D/3D Bounding Box

### Camera 생성 예시 (Python)
```python
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.prims as prim_utils

# 1. Create Camera Prim
camera_prim = prim_utils.create_prim(
    "/World/Camera",
    "Camera",
    position=np.array([0.5, 0, 0.3]),
    orientation=euler_angles_to_quat(np.array([0, 30, 0]), degrees=True)
)

# 2. Create Camera Sensor
camera = Camera(
    prim_path="/World/Camera",
    frequency=20,  # 20 Hz
    resolution=(256, 256),
    projection_type="pinhole",
)

# 3. Initialize
camera.initialize()
camera.add_motion_vectors_to_frame()

# 4. Get RGBD data
rgb_image = camera.get_rgba()[:, :, :3]  # (256, 256, 3)
depth_image = camera.get_depth()  # (256, 256, 1)
```

### Camera Attachment (URDF)
```xml
<!-- RoArm-M3 URDF에 카메라 추가 -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.025 0.09 0.025"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.025 0.09 0.025"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.072"/>
    <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="gripper_base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.02" rpy="0 0.5 0"/>  <!-- 앞쪽 아래 30도 -->
</joint>
```

### Isaac Sim에서 Camera USD 생성
```python
# USD Stage에서 Camera 추가
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()
camera_prim_path = "/World/RoArm_M3/camera_link/Camera"

# Create Camera
camera = UsdGeom.Camera.Define(stage, camera_prim_path)
camera.CreateHorizontalApertureAttr(20.955)  # 센서 크기
camera.CreateVerticalApertureAttr(15.2908)
camera.CreateFocalLengthAttr(24)  # 초점 거리
camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10000))  # near, far plane

# Set resolution
camera.CreateProjectionAttr("perspective")
```

---

## 3. Vision-Based RL 논문 및 코드

### 추천 논문 (arXiv)
1. **"Learning to Manipulate Deformable Objects without Demonstrations"** (RSS 2020)
   - URL: https://arxiv.org/abs/1910.13439
   - 핵심: RGB-D + PPO, rope manipulation

2. **"Deep Reinforcement Learning for Vision-Based Robotic Grasping"** (ICRA 2018)
   - URL: https://arxiv.org/abs/1802.10264
   - 핵심: QT-Opt, CNN + Bellman backup

3. **"Learning Dexterous In-Hand Manipulation"** (OpenAI, IJRR 2020)
   - URL: https://arxiv.org/abs/1808.00177
   - 핵심: PPO + Domain Randomization (vision)

4. **"Visual Foresight: Model-Based Deep Reinforcement Learning"** (2018)
   - URL: https://arxiv.org/abs/1812.00568
   - 핵심: Video prediction + action planning

5. **"SAC-X: Soft Actor-Critic for Pixels"** (2019)
   - URL: https://arxiv.org/abs/1910.01741
   - 핵심: SAC + image observations

### GitHub 코드 저장소
1. **DLR-RM/stable-baselines3**
   - URL: https://github.com/DLR-RM/stable-baselines3
   - 내용: CnnPolicy, RecurrentPPO, Atari wrappers

2. **thu-ml/tianshou**
   - URL: https://github.com/thu-ml/tianshou
   - 내용: Modular RL library, Vision support

3. **google-research/vision_transformer**
   - URL: https://github.com/google-research/vision_transformer
   - 내용: ViT (Transformer for images)

4. **openai/baselines**
   - URL: https://github.com/openai/baselines
   - 내용: PPO, A2C with Atari CNN

---

## 4. RGB-D 전처리 및 Data Pipeline

### Preprocessing Pipeline
```python
import numpy as np
import cv2

class VisionPreprocessor:
    def __init__(self, image_size=(84, 84)):
        self.image_size = image_size
    
    def preprocess_rgb(self, rgb):
        """RGB 이미지 전처리"""
        # (H, W, 3) → (84, 84, 3)
        rgb = cv2.resize(rgb, self.image_size)
        # Normalize to [0, 1]
        rgb = rgb.astype(np.float32) / 255.0
        # (H, W, C) → (C, H, W) for PyTorch
        rgb = np.transpose(rgb, (2, 0, 1))
        return rgb
    
    def preprocess_depth(self, depth, max_depth=2.0):
        """Depth 이미지 전처리"""
        # Clip and normalize
        depth = np.clip(depth, 0, max_depth) / max_depth
        # Resize
        depth = cv2.resize(depth, self.image_size)
        # Add channel dimension
        depth = np.expand_dims(depth, axis=0)  # (1, H, W)
        return depth
    
    def combine_rgbd(self, rgb, depth):
        """RGB + Depth → RGBD (4 channels)"""
        rgb = self.preprocess_rgb(rgb)  # (3, H, W)
        depth = self.preprocess_depth(depth)  # (1, H, W)
        rgbd = np.concatenate([rgb, depth], axis=0)  # (4, H, W)
        return rgbd
```

### Frame Stacking (Temporal Context)
```python
from collections import deque

class FrameStack:
    def __init__(self, num_stack=4):
        self.num_stack = num_stack
        self.frames = deque(maxlen=num_stack)
    
    def reset(self, initial_frame):
        for _ in range(self.num_stack):
            self.frames.append(initial_frame)
        return self.get_obs()
    
    def step(self, frame):
        self.frames.append(frame)
        return self.get_obs()
    
    def get_obs(self):
        # Stack along channel dimension
        # (4, 84, 84) x 4 → (16, 84, 84)
        return np.concatenate(list(self.frames), axis=0)
```

---

## 5. Isaac Sim Vision Environment 구현 예시

### ObservationBuilder 수정 (v3.10)
```python
# envs/observation/observation_builder.py
class ObservationBuilder:
    def __init__(self, env, obs_mode="vector"):
        self.env = env
        self.obs_mode = obs_mode  # "vector", "rgb", "rgbd"
        
        if obs_mode in ["rgb", "rgbd"]:
            self.camera = self._setup_camera()
            self.preprocessor = VisionPreprocessor()
    
    def _setup_camera(self):
        from omni.isaac.sensor import Camera
        camera = Camera(
            prim_path="/World/RoArm_M3/camera_link/Camera",
            frequency=20,
            resolution=(256, 256),
        )
        camera.initialize()
        return camera
    
    def get_observation(self, observations):
        if self.obs_mode == "vector":
            # 기존 28-dim vector
            return self._build_vector_obs(observations)
        
        elif self.obs_mode == "rgb":
            # RGB only (3, 84, 84)
            rgb = self.camera.get_rgba()[:, :, :3]
            return self.preprocessor.preprocess_rgb(rgb)
        
        elif self.obs_mode == "rgbd":
            # RGB + Depth (4, 84, 84)
            rgb = self.camera.get_rgba()[:, :, :3]
            depth = self.camera.get_depth()
            return self.preprocessor.combine_rgbd(rgb, depth)
```

### Environment 수정
```python
# envs/roarm_pick_place_env.py
from gymnasium import spaces

class RoArmPickPlaceEnv(VecEnvBase):
    def __init__(self, ..., obs_mode="vector"):
        super().__init__(...)
        self.obs_mode = obs_mode
        
        # Observation space
        if obs_mode == "vector":
            self.observation_space = spaces.Box(
                low=-np.inf, high=np.inf, shape=(28,), dtype=np.float32
            )
        elif obs_mode == "rgb":
            self.observation_space = spaces.Box(
                low=0, high=1, shape=(3, 84, 84), dtype=np.float32
            )
        elif obs_mode == "rgbd":
            self.observation_space = spaces.Box(
                low=0, high=1, shape=(4, 84, 84), dtype=np.float32
            )
        
        self.obs_builder = ObservationBuilder(self, obs_mode)
```

### Training Script 수정
```python
# scripts/rl/train_vision_rl.py
from stable_baselines3 import PPO
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

env = RoArmPickPlaceEnv(obs_mode="rgbd")

model = PPO(
    "CnnPolicy",
    env,
    policy_kwargs=dict(
        features_extractor_class=CustomCNN,
        features_extractor_kwargs=dict(features_dim=256),
    ),
    n_steps=2048,
    batch_size=64,
    learning_rate=3e-4,
    verbose=1
)

model.learn(total_timesteps=5_000_000)
```

---

## 6. Domain Randomization (Vision)

### Texture Randomization
```python
# envs/utils/domain_randomization.py
import omni.isaac.core.utils.prims as prim_utils
from pxr import UsdShade

def randomize_texture(prim_path, textures_list):
    """랜덤 텍스처 적용"""
    texture = np.random.choice(textures_list)
    
    material = UsdShade.Material.Get(stage, f"{prim_path}/Material")
    shader = UsdShade.Shader(material.GetPrim().GetChild("Shader"))
    
    diffuse_texture = shader.GetInput("diffuse_texture")
    diffuse_texture.Set(texture)
```

### Lighting Randomization
```python
def randomize_lighting(light_prim_path):
    """조명 강도/색상 랜덤화"""
    light = prim_utils.get_prim_at_path(light_prim_path)
    
    intensity = np.random.uniform(500, 2000)
    color = np.random.uniform([0.8, 0.8, 0.8], [1.0, 1.0, 1.0])
    
    light.GetAttribute("inputs:intensity").Set(intensity)
    light.GetAttribute("inputs:color").Set(tuple(color))
```

### Camera Pose Randomization
```python
def randomize_camera_pose(camera_joint_path):
    """카메라 위치/각도 랜덤화"""
    # ±5cm, ±10도
    pos_noise = np.random.uniform([-0.05, -0.05, -0.05], [0.05, 0.05, 0.05])
    rot_noise = np.random.uniform([-10, -10, -10], [10, 10, 10])
    
    # Apply to camera joint
    # ...
```

---

## 7. 학습 로드맵 (Week 3-4)

### Week 3: CNN Policy 통합
**Day 1-2**: ObservationBuilder 수정
- RGB/RGBD observation 모드 추가
- Camera Sensor 통합
- Preprocessing pipeline 구현

**Day 3-4**: Custom CNN 구현
- SB3 CnnPolicy 기반 Custom CNN
- Feature extractor 테스트
- Observation space 검증

**Day 5**: Training Script 수정
- `train_vision_rl.py` 생성
- Hyperparameter 조정 (batch_size, n_steps)
- 초기 학습 실험 (100K steps)

### Week 4: Vision RL 학습
**Day 1-2**: Domain Randomization
- Texture randomization
- Lighting randomization
- Camera pose randomization

**Day 3-5**: Vision RL 학습 (1M steps)
- RGB-only baseline
- RGBD comparison
- 성능 비교 (reach_rate, grip_rate)

---

## 8. 추가 자료

### Books
- **"Reinforcement Learning: An Introduction"** (Sutton & Barto)
  - Chapter 9: On-policy Prediction with Function Approximation
- **"Deep Learning"** (Goodfellow et al.)
  - Chapter 9: Convolutional Networks

### Online Courses
- **Stanford CS231n**: Convolutional Neural Networks for Visual Recognition
- **DeepMind RL Course**: https://www.deepmind.com/learning-resources/reinforcement-learning-lecture-series-2021

### Blogs
- **OpenAI Spinning Up**: https://spinningup.openai.com/en/latest/
- **Lil'Log - Policy Gradient Algorithms**: https://lilianweng.github.io/posts/2018-04-08-policy-gradient/

---

## 다운로드 체크리스트

- [ ] Stable-Baselines3 (pip install)
- [ ] Isaac Sim Camera Sensor Docs (북마크)
- [ ] Vision-based RL 논문 5편 (arXiv)
- [ ] Custom CNN 코드 예시 (GitHub)
- [ ] Domain Randomization 예시

**다음 단계**:
1. SB3 CnnPolicy 튜토리얼 실습
2. Isaac Sim Camera API 테스트
3. RGB/RGBD observation 모드 구현
4. Custom CNN Feature Extractor 작성

---

**작성일**: 2025-10-28  
**상태**: 리소스 수집 완료, 구현 대기
