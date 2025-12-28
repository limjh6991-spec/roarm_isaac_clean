# Isaac Sim 5.1.0 API Migration Guide

## 🔄 API 변경 사항 (5.0 → 5.1)

### 핵심 변경사항
Isaac Sim 5.1.0부터 `omni.isaac.*` 네임스페이스가 deprecated되고 `isaacsim.*`로 통합되었습니다.

---

## 📋 API 매핑 테이블

### Core APIs

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.isaac.kit.SimulationApp` | `isaacsim.SimulationApp` |
| `omni.isaac.core.World` | `isaacsim.core.api.World` |
| `omni.isaac.core.simulation_context.SimulationContext` | `isaacsim.core.api.SimulationContext` |
| `omni.isaac.core.physics_context.PhysicsContext` | `isaacsim.core.api.PhysicsContext` |

### Objects & Prims

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.isaac.core.objects.DynamicCuboid` | `isaacsim.core.api.prims.XFormPrim` |
| `omni.isaac.core.objects.RigidPrim` | `isaacsim.core.api.prims.RigidPrim` |
| `omni.isaac.core.prims.XFormPrim` | `isaacsim.core.api.prims.XFormPrim` |

### Articulations

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.isaac.core.articulations.Articulation` | `isaacsim.core.api.articulations.Articulation` |
| `omni.isaac.manipulators.*` | `isaacsim.robot.manipulators.*` |

### Sensors

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.isaac.sensor.Camera` | `isaacsim.sensors.camera.Camera` |
| `omni.isaac.sensor.camera` | `isaacsim.sensors.camera.camera` |
| `omni.isaac.sensor.camera_view` | `isaacsim.sensors.camera.camera_view` |
| `omni.isaac.sensor.contact_sensor` | `isaacsim.sensors.physics.contact_sensor` |
| `omni.isaac.sensor.imu_sensor` | `isaacsim.sensors.physics.imu_sensor` |
| `omni.isaac.sensor.lidar_rtx` | `isaacsim.sensors.rtx.lidar_rtx` |
| `omni.isaac.range_sensor.*` | `isaacsim.sensors.physx.*` |

### Robot & Control

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.isaac.wheeled_robots.*` | `isaacsim.robot.wheeled_robots.*` |
| `omni.isaac.quadruped.*` | `isaacsim.robot.policy.examples.*` |
| `omni.isaac.surface_gripper.*` | `isaacsim.robot.surface_gripper.*` |
| `omni.isaac.motion_generation.*` | `isaacsim.robot_motion.motion_generation.*` |
| `omni.isaac.lula.*` | `isaacsim.robot_motion.lula.*` |

### Utilities

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.isaac.core.utils.stage` | `isaacsim.core.api.utils.stage` |
| `omni.isaac.core.utils.extensions` | `isaacsim.core.api.utils.extensions` |
| `omni.isaac.nucleus.*` | `isaacsim.storage.native.*` |
| `omni.isaac.cloner.*` | `isaacsim.core.cloner.*` |
| `omni.isaac.version.*` | `isaacsim.core.version.*` |

### Assets & Import

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.isaac.asset_browser.*` | `isaacsim.asset.browser.*` |
| `omni.isaac.occupancy_map.*` | `isaacsim.asset.gen.omap.*` |
| `omni.isaac.block_world.*` | `isaacsim.asset.importer.heightmap.*` |

### Cortex (Behavior)

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.isaac.cortex.*` | `isaacsim.cortex.framework.*` |
| `omni.isaac.cortex.sample_behaviors.*` | `isaacsim.cortex.behaviors.*` |

### Replicator

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.replicator.isaac.*` | `isaacsim.replicator.domain_randomization.*` |
| `omni.isaac.synthetic_recorder.*` | `isaacsim.replicator.synthetic_recorder.*` |
| `omni.isaac.scene_blox.*` | `isaacsim.replicator.scene_blox.*` |

### UI & GUI

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.isaac.menu.*` | `isaacsim.gui.menu.*` |
| `omni.isaac.window.about.*` | `isaacsim.app.about.*` |
| `omni.kit.property.isaac.*` | `isaacsim.gui.property.*` |

### Core Nodes

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.isaac.core_nodes.*` | `isaacsim.core.nodes.*` |

### Other

| Isaac Sim 5.0 (Old) | Isaac Sim 5.1 (New) |
|---------------------|---------------------|
| `omni.isaac.throttling.*` | `isaacsim.core.throttling.*` |
| `omni.isaac.dynamic_control.*` | Deprecated (use Articulation API) |

---

## 📝 코드 변경 예시

### Example 1: Basic SimulationApp

**Before (5.0):**
```python
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

world = World()
cube = DynamicCuboid("/World/Cube", size=0.1)
```

**After (5.1):**
```python
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import World
from isaacsim.core.api.prims import RigidPrim

world = World()
# Note: DynamicCuboid is replaced with RigidPrim or use USD prims
```

### Example 2: Camera Sensor

**Before (5.0):**
```python
from omni.isaac.sensor import Camera

camera = Camera(
    prim_path="/World/Camera",
    frequency=30,
    resolution=(640, 480)
)
```

**After (5.1):**
```python
from isaacsim.sensors.camera import Camera

camera = Camera(
    prim_path="/World/Camera",
    frequency=30,
    resolution=(640, 480)
)
```

### Example 3: Articulation

**Before (5.0):**
```python
from omni.isaac.core.articulations import Articulation

robot = Articulation(prim_path="/World/Robot")
```

**After (5.1):**
```python
from isaacsim.core.api.articulations import Articulation

robot = Articulation(prim_path="/World/Robot")
```

### Example 4: URDF Import (with Isaac Lab)

**Before (Isaac Lab 1.0):**
```python
from omni.isaac.lab.sim.converters import UrdfConverter, UrdfConverterCfg

urdf_cfg = UrdfConverterCfg(
    asset_path="robot.urdf",
    usd_dir="./usd",
)
```

**After (Use native Isaac Sim 5.1):**
```python
from isaacsim.asset.importer.urdf import UrdfImporter

urdf_importer = UrdfImporter()
result = urdf_importer.import_urdf(
    urdf_path="robot.urdf",
    output_path="./usd/robot.usd"
)
```

---

## 🚨 주의사항

### 1. Isaac Lab Compatibility
- Isaac Lab은 별도의 프로젝트로, Isaac Sim 5.1에서는 `omni.isaac.lab` 대신 native `isaacsim.*` API를 사용하는 것을 권장합니다.
- Isaac Lab을 계속 사용하려면 별도의 호환성 계층이 필요할 수 있습니다.

### 2. Deprecated APIs
- `omni.isaac.dynamic_control`: 완전히 제거됨. Articulation API 사용 필요
- `omni.isaac.sensor`: 여러 센서 모듈로 분리됨
  - `isaacsim.sensors.camera`
  - `isaacsim.sensors.physics`
  - `isaacsim.sensors.physx`
  - `isaacsim.sensors.rtx`

### 3. Import Order
SimulationApp은 항상 다른 Isaac Sim import보다 먼저 생성되어야 합니다:

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

# 이후에 다른 isaacsim 모듈 import
from isaacsim.core.api import World
```

---

## 🔧 자동 변환 스크립트

프로젝트 전체를 자동으로 변환하려면:

```bash
# 기본 패턴 변경
find . -name "*.py" -type f -exec sed -i 's/omni\.isaac\.kit/isaacsim/g' {} +
find . -name "*.py" -type f -exec sed -i 's/omni\.isaac\.core\./isaacsim.core.api./g' {} +
find . -name "*.py" -type f -exec sed -i 's/omni\.isaac\.sensor\./isaacsim.sensors.camera./g' {} +

# 더 세밀한 변경은 수동으로 진행 필요
```

---

## 📚 참고 자료

- [Isaac Sim 5.1 Release Notes](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/release_notes.html)
- [Isaac Sim 5.1 API Documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/api.html)
- [Migration Guide](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/migration.html)

---

**작성일**: 2025-11-17  
**대상 버전**: Isaac Sim 5.1.0  
