# Isaac Sim 5.1 Cheatsheet

## Quick Import Reference

```python
# ===== 초기화 (항상 먼저!) =====
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

# ===== Core API =====
from isaacsim.core.api.world import World
from isaacsim.core.api.simulation_context import SimulationContext
from isaacsim.core.api.physics_context import PhysicsContext

# ===== Prims =====
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.prims import XFormPrim, RigidPrim

# ===== Objects =====
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.objects import DynamicSphere, VisualSphere

# ===== Sensors =====
from isaacsim.sensors.camera import Camera

# ===== USD =====
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics
```

## World & Scene 생성

```python
world = World(stage_units_in_meters=1.0)
world.scene.add_ground_plane()

# DynamicCuboid 추가
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        size=0.05,
        color=np.array([1.0, 0.0, 0.0]),
        translation=np.array([0.3, 0.0, 0.5]),
    )
)

world.reset()
```

## USD로 로봇 로드

```python
import omni.usd
from pxr import Sdf, UsdGeom

stage = omni.usd.get_context().get_stage()

# USD 파일 참조
prim = stage.DefinePrim("/World/Robot", "Xform")
prim.GetReferences().AddReference("/path/to/robot.usd")

# SingleArticulation으로 래핑
from isaacsim.core.prims import SingleArticulation
robot = SingleArticulation(prim_path="/World/Robot", name="robot")
world.scene.add(robot)
```

## 로봇 제어

```python
# 초기화 후
robot.initialize()

# 관절 상태 읽기
positions = robot.get_joint_positions()
velocities = robot.get_joint_velocities()

# 관절 제어
robot.set_joint_position_targets(target_positions)
robot.set_joint_velocity_targets(target_velocities)

# DOF 정보
num_dof = robot.num_dof
dof_names = robot.dof_names
```

## 카메라

```python
camera = Camera(
    prim_path="/World/Camera",
    name="camera",
    translation=np.array([1.0, 0.0, 0.8]),
    resolution=(256, 256),
)
camera.initialize()

# 이미지 획득
rgba = camera.get_rgba()  # (H, W, 4) uint8
depth = camera.get_depth()  # (H, W) float32
```

## 시뮬레이션 루프

```python
world.reset()

for i in range(1000):
    # 액션 적용
    robot.set_joint_position_targets(actions)
    
    # 스텝
    world.step(render=True)
    
    # 관찰
    obs = camera.get_rgba()
    positions = robot.get_joint_positions()

# 종료
app.close()
```

## 변경된 파라미터

| 이전 | 현재 |
|------|------|
| `use_flatcache=True` | `use_fabric=True` |
| `omni.isaac.core` | `isaacsim.core.api` |
| `Articulation` | `SingleArticulation` |

---
*Last updated: 2025-12-27*
