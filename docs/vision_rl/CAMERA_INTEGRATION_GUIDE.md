# Intel RealSense D405/D455 카메라 통합 가이드

**작성일**: 2025-11-02  
**목적**: Phase 2 Vision RL을 위한 RealSense 카메라 자료 정리

---

## 📋 목차

1. [카메라 스펙 비교](#1-카메라-스펙-비교)
2. [Intel RealSense ROS2 패키지](#2-intel-realsense-ros2-패키지)
3. [URDF 생성 및 통합](#3-urdf-생성-및-통합)
4. [Isaac Sim 카메라 통합](#4-isaac-sim-카메라-통합)
5. [Vision-Based RL 구현](#5-vision-based-rl-구현)
6. [다음 단계](#6-다음-단계)

---

## 1. 카메라 스펙 비교

### 1.1 Intel RealSense D405 (근거리)

**용도**: 7cm ~ 50cm 근거리 고정밀 작업

| 항목 | 스펙 |
|------|------|
| **FOV** | 87° × 58° |
| **이상적인 범위** | 7 cm ~ 50 cm |
| **최소 물체 감지** | < 1mm @ 7cm |
| **정확도** | Sub-millimeter (7cm에서) |
| **셔터** | Global Shutter |
| **크기** | 42mm × 42mm × 23mm |
| **무게** | 60g |
| **RGB 해상도** | ISP 기반 (depth 센서에서 생성) |
| **Depth 해상도** | 640×480 |

**추천 사용 사례**:
- ✅ **Pick and Place** (작은 물체)
- ✅ Defect Inspection (0.1mm 정밀도)
- ✅ Wound Measurement (의료)
- ✅ 그리퍼 정밀 제어

### 1.2 Intel RealSense D455 (중거리)

**용도**: 0.6m ~ 6m 중거리 범용 작업

| 항목 | 스펙 |
|------|------|
| **FOV** | 87° × 58° |
| **이상적인 범위** | 0.6 m ~ 6 m |
| **Depth 오차** | < 2% @ 4m |
| **셔터** | Global Shutter (RGB + Depth 모두) |
| **IMU** | 내장 (gyro + accel) |
| **RGB 해상도** | 1920×1080 (전용 센서) |
| **Depth 해상도** | 1280×720 |
| **베이스라인** | 95mm (D435i: 50mm) |

**추천 사용 사례**:
- ✅ 충돌 회피 (AMR, 드론)
- ✅ 3D 스캐닝
- ✅ Digital Signage (터치리스 제스처)
- ✅ Volumetric Capture

### 1.3 RoArm-M3 프로젝트 선택

**Phase 2 목표**: Pick and Place (고추 수확)

| 기준 | D405 | D455 | 선택 |
|------|------|------|------|
| 작업 거리 | 7-50cm | 60-600cm | ✅ **D405** |
| 정밀도 | Sub-mm | <2% @ 4m | ✅ **D405** |
| 크기/무게 | 60g | 72g | ✅ **D405** |
| 가격 | 저렴 | 중간 | ✅ **D405** |
| RGB 품질 | ISP 기반 | 전용 센서 | ⚠️ D455 우세 |

**최종 결정**: 
- **D405**를 1차 목표로 선택 (Pick and Place 최적)
- D455도 자료 수집 (향후 확장용)

---

## 2. Intel RealSense ROS2 패키지

### 2.1 GitHub 저장소

**URL**: https://github.com/IntelRealSense/realsense-ros

**지원 ROS2 버전**:
- ROS2 Humble ✅
- ROS2 Iron
- ROS2 Jazzy
- ROS2 Rolling

### 2.2 패키지 구조

```
realsense-ros/
├── realsense2_camera/          # ROS2 노드
├── realsense2_camera_msgs/     # 메시지 정의
├── realsense2_description/     # URDF + Mesh
│   ├── meshes/
│   │   ├── d405.dae            # D405 메시
│   │   ├── d455.dae            # D455 메시
│   │   └── ...
│   └── urdf/
│       ├── d405.urdf.xacro     # D405 URDF
│       ├── d455.urdf.xacro     # D455 URDF
│       └── ...
└── scripts/
```

### 2.3 주요 기능

1. **RGB-D 동기화**:
   - `enable_sync: true` → 프레임 타임스탬프 일치
   - `/camera/camera/rgbd` 토픽 제공

2. **Extrinsics 제공**:
   - Depth → Color 변환 행렬
   - TF (static + dynamic) 퍼블리시

3. **IMU 통합** (D455만):
   - Gyro: Angular velocity
   - Accel: Linear acceleration
   - 통합 `/imu` 토픽 (unite_imu_method)

4. **QoS 설정**:
   - `depth_qos`, `color_qos`, `pointcloud_qos`
   - SENSOR_DATA, SYSTEM_DEFAULT 등

### 2.4 설치 방법

```bash
# ROS2 Workspace에 클론
cd ~/ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git

# librealsense2 SDK 설치
sudo apt install ros-humble-librealsense2*

# 빌드
cd ~/ros2_ws
colcon build --packages-select realsense2_camera realsense2_description

# 실행
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

### 2.5 ROS2 Topics (예시)

```
/camera/camera/color/image_raw        # RGB (1920x1080)
/camera/camera/color/camera_info
/camera/camera/depth/image_rect_raw   # Depth (1280x720)
/camera/camera/depth/camera_info
/camera/camera/depth/color/points     # PointCloud2
/camera/camera/rgbd                   # RGB+Depth 동기화
/camera/camera/extrinsics/depth_to_color
/tf_static                            # Camera TF
```

---

## 3. URDF 생성 및 통합

### 3.1 D405 URDF 템플릿

```xml
<?xml version="1.0"?>
<robot name="realsense_d405" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Base Link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.042 0.042 0.023"/>  <!-- 42x42x23mm -->
      </geometry>
      <material name="camera_black">
        <color rgba="0.15 0.15 0.15 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.042 0.042 0.023"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.060"/>  <!-- 60g -->
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" 
               iyy="0.00001" iyz="0" 
               izz="0.00001"/>
    </inertial>
  </link>

  <!-- Depth Frame -->
  <link name="camera_depth_frame"/>
  <joint name="camera_depth_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_depth_frame"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Depth Optical Frame -->
  <link name="camera_depth_optical_frame"/>
  <joint name="camera_depth_optical_joint" type="fixed">
    <parent link="camera_depth_frame"/>
    <child link="camera_depth_optical_frame"/>
    <!-- ROS (X:forward, Y:left, Z:up) → Optical (X:right, Y:down, Z:forward) -->
    <origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/>
  </joint>

  <!-- Color Frame -->
  <link name="camera_color_frame"/>
  <joint name="camera_color_joint" type="fixed">
    <parent link="camera_depth_frame"/>
    <child link="camera_color_frame"/>
    <!-- Depth에서 Color로의 Extrinsic (D405는 거의 동일) -->
    <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
  </joint>

  <!-- Color Optical Frame -->
  <link name="camera_color_optical_frame"/>
  <joint name="camera_color_optical_joint" type="fixed">
    <parent link="camera_color_frame"/>
    <child link="camera_color_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/>
  </joint>

</robot>
```

### 3.2 D455 URDF 템플릿

```xml
<?xml version="1.0"?>
<robot name="realsense_d455" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Base Link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.124 0.029 0.026"/>  <!-- 124x29x26mm -->
      </geometry>
      <material name="camera_black">
        <color rgba="0.15 0.15 0.15 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.124 0.029 0.026"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.072"/>  <!-- 72g -->
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" 
               iyy="0.00001" iyz="0" 
               izz="0.00001"/>
    </inertial>
  </link>

  <!-- Depth Frame (Left IR 기준) -->
  <link name="camera_depth_frame"/>
  <joint name="camera_depth_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_depth_frame"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Depth Optical Frame -->
  <link name="camera_depth_optical_frame"/>
  <joint name="camera_depth_optical_joint" type="fixed">
    <parent link="camera_depth_frame"/>
    <child link="camera_depth_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/>
  </joint>

  <!-- Color Frame (RGB Sensor) -->
  <link name="camera_color_frame"/>
  <joint name="camera_color_joint" type="fixed">
    <parent link="camera_depth_frame"/>
    <child link="camera_color_frame"/>
    <!-- Depth → Color Extrinsic (95mm baseline, 약간 오른쪽) -->
    <origin xyz="0.0148 0.001 0.0005" rpy="0 0 0"/>
  </joint>

  <!-- Color Optical Frame -->
  <link name="camera_color_optical_frame"/>
  <joint name="camera_color_optical_joint" type="fixed">
    <parent link="camera_color_frame"/>
    <child link="camera_color_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/>
  </joint>

  <!-- IMU Link (Optional) -->
  <link name="camera_imu_frame"/>
  <joint name="camera_imu_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_imu_frame"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

</robot>
```

### 3.3 RoArm-M3 통합

**목표**: 카메라를 RoArm-M3 URDF에 부착

**옵션 1: 그리퍼 베이스에 부착** (추천)

```xml
<!-- roarm_m3_with_camera.urdf -->
<robot name="roarm_m3_with_d405">
  
  <!-- 기존 RoArm-M3 URDF 내용 -->
  <xacro:include filename="roarm_m3_enhanced.urdf"/>
  
  <!-- D405 URDF 포함 -->
  <xacro:include filename="realsense_d405.urdf"/>
  
  <!-- 카메라 부착 -->
  <joint name="camera_mount_joint" type="fixed">
    <parent link="gripper_base_link"/>
    <child link="camera_link"/>
    <!-- 그리퍼 앞쪽 5cm, 위쪽 2cm, 30도 아래 -->
    <origin xyz="0.05 0 0.02" rpy="0 0.5236 0"/>
  </joint>

</robot>
```

**옵션 2: 별도 링크 추가**

```xml
<!-- 카메라 전용 마운트 -->
<link name="camera_mount_link">
  <visual>
    <geometry>
      <box size="0.03 0.05 0.01"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="0.02"/>
    <inertia ixx="0.00001" ixy="0" ixz="0" 
             iyy="0.00001" iyz="0" 
             izz="0.00001"/>
  </inertial>
</link>

<joint name="camera_mount_joint" type="fixed">
  <parent link="link6"/>
  <child link="camera_mount_link"/>
  <origin xyz="0.06 0 0.03" rpy="0 0 0"/>
</joint>

<joint name="camera_attachment_joint" type="fixed">
  <parent link="camera_mount_link"/>
  <child link="camera_link"/>
  <origin xyz="0 0 0.01" rpy="0 0.5236 0"/>
</joint>
```

### 3.4 좌표계 설명

**ROS vs Optical 좌표계**:

| 좌표계 | X | Y | Z |
|--------|---|---|---|
| **ROS** (Robot) | Forward | Left | Up |
| **Optical** (Camera) | Right | Down | Forward |

**변환**: ROS → Optical
```
rpy="-1.5707963 0 -1.5707963"
# Roll -90°: Y축 회전 (Z down)
# Yaw -90°: Z축 회전 (X right)
```

---

## 4. Isaac Sim 카메라 통합

### 4.1 URDF → USD 변환

```python
#!/usr/bin/env python3
"""
RoArm-M3 + D405 URDF to USD Converter
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from isaacsim.asset.importer.urdf import _urdf

# URDF 경로
urdf_path = "/path/to/roarm_m3_with_d405.urdf"

# World 생성
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Import Config
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False  # 카메라 프레임 유지!
import_config.import_inertia_tensor = True
import_config.fix_base = False
import_config.self_collision = True
import_config.distance_scale = 1.0

# URDF Interface
urdf_interface = _urdf.acquire_urdf_interface()

# Parse
success = urdf_interface.parse_urdf(
    str(urdf_path.parent),
    str(urdf_path.name),
    import_config
)

if not success:
    print("❌ URDF Parse 실패")
    simulation_app.close()
    exit(1)

# Import
prim_path = urdf_interface.import_robot(
    str(urdf_path.parent),
    str(urdf_path.name),
    import_config,
    "/World/RoArm_M3"
)

print(f"✅ Robot imported at: {prim_path}")

# USD 저장
from pxr import Usd
stage = world.stage
output_path = "/path/to/roarm_m3_with_camera.usd"
stage.Export(output_path)
print(f"✅ USD saved: {output_path}")

# GUI 유지
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

### 4.2 Isaac Sim Camera Sensor 추가

**방법 1: Python API (코드)**

```python
from omni.isaac.sensor import Camera
import numpy as np

# 1. Camera Prim 생성 (URDF에서 이미 생성됨)
camera_prim_path = "/World/RoArm_M3/camera_link"

# 2. Camera Sensor 생성
camera = Camera(
    prim_path=f"{camera_prim_path}/Camera",
    frequency=20,  # 20 FPS
    resolution=(256, 256),  # 84x84는 전처리 후
    projection_type="pinhole",
)

# 3. Initialize
camera.initialize()

# 4. RGB + Depth 데이터 가져오기
def get_rgbd_observation():
    # RGB (H, W, 3)
    rgb = camera.get_rgba()[:, :, :3]
    rgb = rgb.cpu().numpy() if hasattr(rgb, 'cpu') else rgb
    
    # Depth (H, W)
    depth = camera.get_depth()
    depth = depth.cpu().numpy() if hasattr(depth, 'cpu') else depth
    
    # Normalize depth (0-1)
    depth = np.clip(depth, 0.1, 2.0)  # 10cm ~ 2m
    depth = (depth - 0.1) / 1.9
    
    # Stack (H, W, 4)
    rgbd = np.concatenate([rgb, depth[:, :, None]], axis=-1)
    
    return rgbd

# 5. 학습 루프에서 사용
obs = get_rgbd_observation()  # (256, 256, 4)
# Resize to 84x84 (CnnPolicy 입력)
from torchvision.transforms import Resize
resize = Resize((84, 84))
obs_tensor = resize(torch.from_numpy(obs).permute(2, 0, 1))  # (4, 84, 84)
```

**방법 2: USD Stage에서 직접 생성**

```python
from pxr import UsdGeom, Gf

stage = world.stage
camera_prim_path = "/World/RoArm_M3/camera_link/Camera"

# Create Camera Prim
camera = UsdGeom.Camera.Define(stage, camera_prim_path)

# Camera Parameters
camera.CreateHorizontalApertureAttr(20.955)  # Sensor width (mm)
camera.CreateVerticalApertureAttr(15.2908)   # Sensor height (mm)
camera.CreateFocalLengthAttr(1.93)           # Focal length (mm)
camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))  # Near/Far

# FOV 설정 (D405: 87° H, 58° V)
# FOV = 2 * arctan(aperture / (2 * focal_length))
# focal_length = aperture / (2 * tan(FOV/2))
```

### 4.3 Camera Calibration

**실제 D405 스펙을 Isaac Sim에 반영**:

| 파라미터 | D405 실제값 | Isaac Sim 설정 |
|----------|-------------|----------------|
| RGB 해상도 | 640×480 | `resolution=(256, 256)` (학습용) |
| Depth 해상도 | 640×480 | `resolution=(256, 256)` |
| H-FOV | 87° | `horizontal_aperture / focal_length` |
| V-FOV | 58° | `vertical_aperture / focal_length` |
| 작동 거리 | 7-50cm | Clipping range (0.05, 0.6) |

---

## 5. Vision-Based RL 구현

### 5.1 Observation Space 변경

**기존 (State-based)**:
```python
# v4.2: 28-dim state
obs = [
    joint_positions (6),      # Joint angles
    joint_velocities (6),     # Joint velocities
    ee_position (3),          # End-effector XYZ
    ee_orientation (4),       # Quaternion
    gripper_position (2),     # Left/Right finger
    object_position (3),      # Target XYZ
    object_orientation (4),   # Quaternion
]
# Total: 28-dim
```

**Vision-based (RGB만)**:
```python
# v5.0: RGB-only
obs = camera.get_rgba()[:, :, :3]  # (256, 256, 3)
# Resize to (84, 84, 3)
# Stack 3 frames: (3, 84, 84) → CnnPolicy
```

**Vision-based (RGB-D)**:
```python
# v5.1: RGB + Depth
rgb = camera.get_rgba()[:, :, :3]    # (256, 256, 3)
depth = camera.get_depth()           # (256, 256, 1)
rgbd = np.concatenate([rgb, depth[:, :, None]], axis=-1)  # (256, 256, 4)
# Resize to (84, 84, 4)
# Stack 3 frames: (4, 84, 84) → CnnPolicy
```

### 5.2 CnnPolicy 설정 (Stable-Baselines3)

```python
from stable_baselines3 import PPO
from stable_baselines3.common.torch_layers import NatureCNN

# Custom CNN Feature Extractor
class CustomCNN(NatureCNN):
    def __init__(self, observation_space, features_dim=256):
        super().__init__(observation_space, features_dim)
        # NatureCNN 구조:
        # Conv1: (4, 84, 84) → (32, 20, 20)
        # Conv2: (32, 20, 20) → (64, 9, 9)
        # Conv3: (64, 9, 9) → (64, 7, 7)
        # Flatten: 64*7*7 = 3136
        # Linear: 3136 → 256 (features_dim)

# PPO with CnnPolicy
model = PPO(
    "CnnPolicy",
    env,
    policy_kwargs={
        "features_extractor_class": CustomCNN,
        "features_extractor_kwargs": {"features_dim": 256},
        "net_arch": [dict(pi=[128, 128], vf=[128, 128])],
    },
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.01,
    verbose=1,
    tensorboard_log="./logs/tensorboard/",
)

model.learn(total_timesteps=10_000_000)
```

### 5.3 DrQ-v2 구현 (추천 알고리즘)

**참고**: `docs/vision_rl/ALGORITHM_COMPARISON.md`에서 DrQ-v2 선택

```python
# DrQ-v2 핵심 구성 요소
class DrQv2Agent:
    def __init__(self):
        self.actor = Actor(obs_shape=(4, 84, 84), action_dim=8)
        self.critic = Critic(obs_shape=(4, 84, 84), action_dim=8)
        self.critic_target = copy.deepcopy(self.critic)
        
        # Data Augmentation
        self.aug = RandomShiftsAug(pad=4)  # Random crop
        
        # Hyperparameters
        self.discount = 0.99
        self.tau = 0.005  # Target network update rate
        self.critic_target_update_freq = 2
        
    def update(self, replay_buffer, step):
        # Sample batch
        obs, action, reward, next_obs, done = replay_buffer.sample(256)
        
        # Augment observations
        obs = self.aug(obs)
        next_obs = self.aug(next_obs)
        
        # Update critic (Q-function)
        with torch.no_grad():
            next_action = self.actor(next_obs)
            target_Q = self.critic_target(next_obs, next_action)
            target_Q = reward + (1 - done) * self.discount * target_Q
        
        current_Q = self.critic(obs, action)
        critic_loss = F.mse_loss(current_Q, target_Q)
        
        # Update actor (Policy)
        actor_loss = -self.critic(obs, self.actor(obs)).mean()
        
        # Soft update target network
        if step % self.critic_target_update_freq == 0:
            soft_update(self.critic_target, self.critic, self.tau)
```

### 5.4 Isaac Sim Environment 수정

```python
# envs/roarm_pick_place_env_vision.py
class RoArmPickPlaceEnvVision(RLTask):
    def __init__(self, name, env_config):
        super().__init__(name, env_config)
        
        # Camera 설정
        self.obs_mode = env_config.get("obs_mode", "rgbd")  # rgb or rgbd
        self.camera_resolution = (256, 256)
        self.obs_resolution = (84, 84)
        
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        
        # Robot + Camera URDF
        self.robot_usd = "/path/to/roarm_m3_with_camera.usd"
        
        # Camera 추가
        for i in range(self._num_envs):
            camera_prim_path = f"/World/envs/env_{i}/RoArm_M3/camera_link/Camera"
            self.cameras[i] = Camera(
                prim_path=camera_prim_path,
                frequency=20,
                resolution=self.camera_resolution,
            )
    
    def get_observations(self):
        # RGB or RGB-D
        obs_list = []
        for i in range(self._num_envs):
            if self.obs_mode == "rgb":
                obs = self.cameras[i].get_rgba()[:, :, :3]
            else:  # rgbd
                rgb = self.cameras[i].get_rgba()[:, :, :3]
                depth = self.cameras[i].get_depth()
                obs = np.concatenate([rgb, depth[:, :, None]], axis=-1)
            
            # Resize to 84x84
            obs = cv2.resize(obs, self.obs_resolution)
            obs_list.append(obs)
        
        return {"obs": torch.stack(obs_list)}
```

---

## 6. 다음 단계

### 6.1 즉시 실행 가능 (Week 1-2)

**Task 1: D405 URDF 생성** (1일)
```bash
□ Intel 공식 realsense-ros 클론
  git clone https://github.com/IntelRealSense/realsense-ros.git
□ realsense2_description/urdf/d405.urdf.xacro 확인
□ RoArm-M3에 맞게 수정
  - 크기: 42x42x23mm
  - 무게: 60g
  - FOV: 87° x 58°
□ 저장: assets/cameras/realsense_d405.urdf
```

**Task 2: RoArm-M3 통합** (1일)
```bash
□ roarm_m3_with_d405.urdf 생성
□ 카메라 부착 위치 결정
  - gripper_base_link에 fixed joint
  - xyz="0.05 0 0.02" rpy="0 0.5236 0"
□ URDF Viewer로 확인
  ~/isaacsim/python.sh scripts/urdf/visualize_urdf_simple.py
```

**Task 3: Isaac Sim 변환 및 테스트** (2일)
```bash
□ URDF → USD 변환
  scripts/utils/urdf_to_usd_camera.py 작성
□ Camera Sensor 추가
  - Camera Prim 생성
  - Resolution: 256x256
  - Frequency: 20 Hz
□ RGB 캡처 테스트
  scripts/test/test_camera_capture.py 작성
□ Depth 캡처 테스트
```

### 6.2 Vision RL 구현 (Week 3-4)

**Task 4: Environment 수정** (3일)
```bash
□ envs/roarm_pick_place_env_vision.py 생성
□ Observation space 변경: 28-dim → (4, 84, 84)
□ Camera 통합
□ 테스트 학습 (100K steps)
```

**Task 5: DrQ-v2 구현** (5일)
```bash
□ algorithms/drqv2/ 디렉토리 생성
□ Actor, Critic, ReplayBuffer 구현
□ Data Augmentation (RandomShift) 추가
□ 학습 스크립트 작성
□ 1M steps 테스트
```

**Task 6: 성능 비교** (2일)
```bash
□ State-based (v4.2) vs Vision-based (v5.0)
□ Sample efficiency 분석
□ REACH/ATTACH 달성률 비교
```

### 6.3 리소스 체크리스트

- [ ] Intel RealSense ROS2 패키지 클론 완료
- [ ] D405 URDF 템플릿 작성 완료
- [ ] RoArm-M3 통합 URDF 생성 완료
- [ ] Isaac Sim Camera Sensor 테스트 완료
- [ ] RGB 캡처 스크립트 작동 확인
- [ ] Depth 캡처 스크립트 작동 확인
- [ ] CnnPolicy 테스트 학습 완료
- [ ] DrQ-v2 구현 시작

---

## 7. 참고 자료

### 7.1 공식 문서

1. **Intel RealSense**:
   - ROS2 Wrapper: https://github.com/IntelRealSense/realsense-ros
   - SDK 2.0: https://github.com/IntelRealSense/librealsense
   - D405 제품 페이지: https://www.intelrealsense.com/depth-camera-d405/
   - D455 제품 페이지: https://www.intelrealsense.com/depth-camera-d455/

2. **Isaac Sim**:
   - Camera Sensor API: `omni.isaac.sensor.Camera`
   - URDF Import: `isaacsim.asset.importer.urdf`

3. **Stable-Baselines3**:
   - CnnPolicy: https://stable-baselines3.readthedocs.io/en/master/guide/custom_policy.html
   - NatureCNN: `stable_baselines3.common.torch_layers.NatureCNN`

### 7.2 프로젝트 내부 문서

1. `docs/vision_rl/ALGORITHM_COMPARISON.md` - Vision RL 알고리즘 비교
2. `docs/PHASE2_VISION_BASED_RL_PLAN.md` - Phase 2 로드맵
3. `resources/vision_rl/README.md` - Vision RL 기술 자료
4. `resources/grippers/README.md` - Gripper + Camera 통합 자료

### 7.3 코드 예시

1. `scripts/utils/urdf_to_usd.py` - URDF → USD 변환
2. `scripts/urdf/visualize_urdf_simple.py` - URDF 시각화
3. `envs/roarm_pick_place_env.py` - 기존 State-based 환경

---

## 8. FAQ

### Q1: D405와 D455 중 어떤 것을 선택해야 하나요?

**A**: Pick and Place 작업이라면 **D405**를 추천합니다.
- RoArm-M3 작업 공간: ~50cm
- D405 이상적 범위: 7-50cm ✅
- Sub-millimeter 정밀도 필요

### Q2: URDF에서 카메라 좌표계가 복잡한 이유는?

**A**: ROS와 Camera의 좌표계가 다르기 때문입니다.
- ROS: X(전방), Y(좌), Z(상)
- Camera Optical: X(우), Y(하), Z(전)
- 변환: `rpy="-1.5708 0 -1.5708"` (Roll -90°, Yaw -90°)

### Q3: Isaac Sim에서 카메라가 데이터를 안 주면?

**A**: 체크리스트:
1. `camera.initialize()` 호출했나?
2. `world.step()` + `world.render()` 호출 중인가?
3. Prim path가 정확한가? (`/World/RoArm_M3/camera_link/Camera`)
4. URDF에서 `merge_fixed_joints=False`로 설정했나?

### Q4: Vision RL이 State-based보다 느린 이유는?

**A**: 정상입니다.
- Sample efficiency: Vision < State (10배 이상 차이)
- 이유: CNN으로 feature 추출해야 함
- 해결: Data augmentation (DrQ-v2), Model-based RL (Dreamer)

### Q5: 학습 속도를 높이려면?

**A**: 
1. 해상도 낮추기 (256 → 84)
2. GPU 사용 (CnnPolicy는 GPU 필수)
3. 병렬 환경 늘리기 (num_envs=128 → 256)
4. Data augmentation으로 sample efficiency 향상

---

**문서 버전**: v1.0  
**마지막 업데이트**: 2025-11-02  
**작성자**: GitHub Copilot + 사용자  
**다음 업데이트**: D405 URDF 생성 완료 후
