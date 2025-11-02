# Isaac Sim 5.0 Vision RL - Extension & API 변경사항

## 🔍 주요 발견 사항

### 1. **Camera API 사용법** (IsaacLab 기반)

Isaac Sim 5.0에서는 **IsaacLab의 Camera API**를 사용하는 것이 권장됩니다:

```python
from isaaclab.sensors import Camera, CameraCfg

# Camera 생성
camera_cfg = CameraCfg(
    prim_path="/World/robot/camera_link",
    update_period=0,  # 매 step마다 업데이트
    height=256,
    width=256,
    data_types=["rgb", "depth", "distance_to_camera"],
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=24.0,
        focus_distance=400.0,
        horizontal_aperture=20.955,
    ),
)

camera = Camera(camera_cfg)
camera.initialize()

# 데이터 캡처
camera.update(dt)
rgb = camera.data.output["rgb"]    # (1, H, W, 3) torch.uint8
depth = camera.data.output["distance_to_camera"]  # (1, H, W, 1) torch.float
```

### 2. **URDF Import 방식 변경**

Isaac Sim 5.0에서 URDF import 방식이 크게 변경되었습니다:

**❌ 이전 방식 (작동 안 함)**:
```python
success, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=omni.isaac.core.utils.extensions.get_extension_path_from_name(...)  # 제거됨!
)
```

**✅ 새로운 방식 (Isaac Sim 5.0)**:
```python
# 방법 1: 최소 파라미터 (추천)
success, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
)

# 방법 2: IsaacLab의 UrdfConverter 사용 (더 안정적)
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

cfg = UrdfConverterCfg(
    asset_path=urdf_path,
    fix_base=False,
    merge_fixed_joints=False,
    make_instanceable=True,
)
converter = UrdfConverter(cfg)
usd_path = converter.usd_path  # 변환된 USD 파일 경로
```

### 3. **Deprecated 모듈 매핑**

| 이전 (Isaac Sim 4.x) | 현재 (Isaac Sim 5.0) |
|---------------------|---------------------|
| `omni.isaac.core` | `isaacsim.core.api` |
| `omni.isaac.sensor` | `isaacsim.sensors.camera`, `isaacsim.sensors.physics`, `isaacsim.sensors.rtx` |
| `omni.isaac.cloner` | `isaacsim.core.cloner` |
| `omni.importer.urdf` | `isaacsim.asset.importer.urdf` |

### 4. **RGB-D 캡처 Best Practice**

IsaacLab 예제에서 발견한 최적 방식:

```python
# Camera 초기화
camera = Camera(camera_cfg)
camera.initialize()

# Simulation loop
while simulation_app.is_running():
    sim.step()
    camera.update(dt)
    
    # RGB 데이터 (uint8)
    rgb = camera.data.output["rgb"]  # (B, H, W, 3)
    rgb_normalized = rgb.float() / 255.0  # [0, 1]
    
    # Depth 데이터 (float)
    depth = camera.data.output["distance_to_camera"]  # (B, H, W, 1)
    
    # Resize for RL (256x256 → 84x84)
    rgb_small = cv2.resize(rgb_normalized[0].cpu().numpy(), (84, 84))
    depth_small = cv2.resize(depth[0].cpu().numpy(), (84, 84))
    
    # Stack RGB-D (4, 84, 84)
    rgbd = np.concatenate([rgb_small, depth_small], axis=-1)
    rgbd = np.transpose(rgbd, (2, 0, 1))  # HWC → CHW
```

### 5. **Camera Data Types**

지원되는 data_types:
- `"rgb"`: (B, H, W, 3) torch.uint8
- `"rgba"`: (B, H, W, 4) torch.uint8
- `"depth"`: Alias for `"distance_to_image_plane"`
- `"distance_to_camera"`: 카메라 중심까지의 거리
- `"distance_to_image_plane"`: 카메라 평면까지의 거리
- `"normals"`: (B, H, W, 3) torch.float
- `"semantic_segmentation"`: Semantic ID
- `"instance_segmentation_fast"`: Instance ID

### 6. **TiledCamera for Multiple Cameras**

여러 카메라를 효율적으로 처리:

```python
from isaaclab.sensors import TiledCamera, TiledCameraCfg

camera_cfg = TiledCameraCfg(
    prim_path="/World/Origin_.*/CameraSensor",  # 여러 카메라
    update_period=0,
    height=256,
    width=256,
    data_types=["rgb", "distance_to_camera"],
)

camera = TiledCamera(camera_cfg)
camera.initialize()

# 모든 카메라의 이미지를 한번에 렌더링
camera.update(dt)
images = camera.data.output["rgb"]  # (num_cameras, H, W, 3)
```

## 🔧 해결된 에러들

### Error 1: `AttributeError: module 'omni.isaac.core' has no attribute 'utils'`

**원인**: Isaac Sim 5.0에서 `omni.isaac.core.utils` 제거됨

**해결**:
```python
# ❌ 이전
import_config = omni.isaac.core.utils.extensions.get_extension_path_from_name(...)

# ✅ 현재
success, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
)
```

### Error 2: `TypeError: URDFParseAndImportFile.__init__() got an unexpected keyword argument`

**원인**: URDF Importer API 파라미터 변경

**해결**: 최소 파라미터만 사용하거나 IsaacLab UrdfConverter 사용

## 📚 참고 자료

### IsaacLab 공식 예제
- Camera 예제: `scripts/tutorials/04_sensors/run_usd_camera.py`
- Cartpole Camera 환경: `isaaclab_tasks/direct/cartpole/cartpole_camera_env.py`
- Vision RL: `isaaclab_tasks/manager_based/classic/cartpole/cartpole_camera_env_cfg.py`

### Key Files 분석

#### 1. **Camera 초기화** (`isaaclab/sensors/camera/camera.py`)
```python
class Camera(SensorBase):
    def __init__(self, cfg: CameraCfg):
        # RGB-D 센서 자동 설정
        # Replicator 통합
        # RTX 렌더링 활성화
```

#### 2. **데이터 접근** (`camera.data.output`)
```python
{
    "rgb": torch.Tensor,      # (B, H, W, 3) uint8
    "depth": torch.Tensor,    # (B, H, W, 1) float32
    "distance_to_camera": torch.Tensor,  # (B, H, W, 1) float32
}
```

#### 3. **RL Environment 통합** (`cartpole_camera_env.py`)
```python
def _get_observations(self) -> dict:
    camera_data = self._tiled_camera.data.output["rgb"] / 255.0
    # Normalize
    mean_tensor = torch.mean(camera_data, dim=(1, 2), keepdim=True)
    camera_data -= mean_tensor
    return {"policy": camera_data.clone()}
```

## 🎯 권장 구현 방식

### 1. **Simple Camera Test**
```python
from isaaclab.sensors import Camera, CameraCfg
import isaaclab.sim as sim_utils

camera_cfg = CameraCfg(
    prim_path="/World/camera",
    height=256,
    width=256,
    data_types=["rgb", "distance_to_camera"],
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=24.0,
        horizontal_aperture=20.955,
    ),
)

camera = Camera(camera_cfg)
```

### 2. **Robot + Camera**
```python
# Robot URDF import (IsaacLab 방식)
from isaaclab.sim.spawners import spawn_from_urdf

robot = spawn_from_urdf(
    prim_path="/World/robot",
    cfg=UrdfFileCfg(
        asset_path="path/to/robot.urdf",
        fix_base=False,
    )
)

# Camera 추가
camera_cfg = CameraCfg(
    prim_path="/World/robot/camera_link",  # URDF의 link와 매칭
    height=256,
    width=256,
    data_types=["rgb", "distance_to_camera"],
)
camera = Camera(camera_cfg)
```

### 3. **Vision RL Training**
```python
# CnnPolicy 사용
from stable_baselines3 import PPO

model = PPO(
    policy="CnnPolicy",
    env=env,
    policy_kwargs=dict(
        features_extractor_kwargs=dict(features_dim=512),
    ),
)
```

## ⚠️ 주의사항

1. **Extension 로딩 순서**:
   ```python
   # URDF import 전에 extension 활성화
   from isaacsim.core.utils.extensions import enable_extension
   enable_extension("isaacsim.asset.importer.urdf")
   ```

2. **RTX Sensors 설정**:
   ```python
   # Camera 사용 시 자동으로 활성화되지만 명시적으로:
   carb_settings_iface = carb.settings.get_settings()
   carb_settings_iface.set_bool("/isaaclab/render/rtx_sensors", True)
   ```

3. **Texture Loading**:
   ```python
   # 처음 몇 프레임은 텍스처 로딩 대기
   for _ in range(5):
       sim.step()
   camera.update(dt)
   ```

4. **Device 설정**:
   ```python
   camera = Camera(camera_cfg)
   # camera.device는 자동으로 SimulationContext의 device 사용
   print(f"Camera device: {camera.device}")  # cuda:0 또는 cpu
   ```

## 🚀 Next Steps

1. ✅ IsaacLab Camera API로 전환
2. ✅ URDF import 방식 수정
3. ✅ TiledCamera 활용 (병렬 렌더링)
4. ⏳ Vision-based RL Environment 구현
5. ⏳ DrQ-v2 알고리즘 적용

---

**생성일**: 2024-11-02  
**기반**: Isaac Sim 5.0 + IsaacLab  
**출처**: isaac-sim/IsaacLab GitHub Repository
