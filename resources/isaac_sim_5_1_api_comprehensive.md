# Isaac Sim 5.1 API 종합 가이드 (2025-12-29 업데이트)

> **버전**: Isaac Sim 5.1.0 GA  
> **검증일**: 2025-12-29 (Husky + RoArm 테스트 통과)

---

## 📋 핵심 변경 사항 요약

### 🚨 Critical Breaking Change: SingleArticulation

**❌ 구버전 (Isaac Sim 5.0 이전)**:
```python
# 이 방식은 5.1에서 작동하지 않음!
robot = SingleArticulation(
    prim_path="/World/Robot",
    name="robot",
    usd_path="/path/to/robot.usd",  # ❌ 지원 안됨!
    translation=np.array([0.0, 0.0, 0.0]),  # ❌ 지원 안됨!
)
```

**✅ 신버전 (Isaac Sim 5.1)**:
```python
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation

# Step 1: USD 파일을 먼저 stage에 추가
add_reference_to_stage(
    usd_path='/path/to/robot.usd',
    prim_path='/World/Robot'
)

# Step 2: SingleArticulation으로 래핑 (prim_path와 name만!)
robot = SingleArticulation(prim_path='/World/Robot', name='robot')
world.scene.add(robot)
world.reset()
```

---

## 🔧 정확한 Import 패턴

### 필수 순서: SimulationApp 먼저!

```python
#!/usr/bin/env python3
# ===== 1. SimulationApp 초기화 (반드시 첫 번째!) =====
from isaacsim import SimulationApp
simulation_app = SimulationApp({
    'headless': False,  # GUI 모드
    'width': 1280,
    'height': 720,
})

# ===== 2. Isaac Sim 모듈 import (SimulationApp 이후!) =====
from isaacsim.core.api import World
from isaacsim.core.api.world import World  # 동일
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.sensors.camera import Camera

# ===== 3. 기타 =====
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics, UsdLux
import numpy as np
```

---

## 🤖 로봇 로딩: 3가지 방법

### 방법 1: add_reference_to_stage (권장 ✅)

```python
from isaacsim.core.utils.stage import add_reference_to_stage

# USD 파일 로드
add_reference_to_stage(
    usd_path='/home/user/robot.usd',
    prim_path='/World/Robot'
)

# Articulation 생성
robot = SingleArticulation(prim_path='/World/Robot', name='robot')
world.scene.add(robot)
```

### 방법 2: 직접 USD 참조 추가

```python
import omni.usd
from pxr import Sdf

stage = omni.usd.get_context().get_stage()

# Prim 생성 및 참조 추가
prim = stage.DefinePrim("/World/Robot", "Xform")
prim.GetReferences().AddReference("/path/to/robot.usd")

# 이후 SingleArticulation으로 래핑
robot = SingleArticulation(prim_path="/World/Robot", name="robot")
```

### 방법 3: World.scene.add() 직접 (Objects용)

```python
# DynamicCuboid 같은 simple objects만 가능
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        size=0.05,
        color=np.array([1.0, 0.0, 0.0]),
        translation=np.array([0.3, 0.0, 0.5]),
    )
)
```

---

## 📷 카메라 설정

```python
from isaacsim.sensors.camera import Camera

camera = Camera(
    prim_path="/World/Camera",
    name="vision_camera",
    translation=np.array([1.0, 0.0, 0.8]),
    orientation=np.array([0.0, 0.0, 0.707, 0.707]),  # 쿼터니언
    resolution=(256, 256),
)

# 초기화 필수!
camera.initialize()

# 이미지 획득 (몇 스텝 후)
for _ in range(5):
    world.step(render=True)
    
rgba = camera.get_rgba()  # (H, W, 4) uint8
depth = camera.get_depth()  # (H, W) float32 (if depth enabled)
```

---

## 🔄 API 마이그레이션 표

| 구버전 (4.x / 5.0) | 신버전 (5.1) |
|-------------------|-------------|
| `omni.isaac.core` | `isaacsim.core.api` |
| `omni.isaac.core.world.World` | `isaacsim.core.api.world.World` |
| `omni.isaac.core.articulations.Articulation` | `isaacsim.core.prims.SingleArticulation` |
| `omni.isaac.core.objects.*` | `isaacsim.core.api.objects.*` |
| `omni.isaac.sensor` | `isaacsim.sensors.*` |
| `use_flatcache=True` | `use_fabric=True` |
| `Articulation(..., usd_path=...)` | `add_reference_to_stage()` 사용 |

---

## ✅ 검증된 작동 예제

### test_husky_roarm.py 패턴 (2025-12-29 검증)

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({'headless': False})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
import numpy as np

# World 초기화
world = World(stage_units_in_meters=1.0, physics_dt=1.0/120.0)
world.scene.add_default_ground_plane()
world.reset()

# 로봇 로드 (2단계)
add_reference_to_stage(
    usd_path='/path/to/robot.usd',
    prim_path='/World/Robot'
)
world.reset()

# SingleArticulation 생성
robot = SingleArticulation(prim_path='/World/Robot', name='robot')
world.scene.add(robot)
world.reset()

# 로봇 제어
print(f"DOF: {robot.num_dof}")
print(f"Joints: {robot.dof_names}")

# 관절 제어
robot.set_joint_positions(np.zeros(robot.num_dof))
robot.set_joint_position_targets(target_positions)

# 시뮬레이션 루프
for i in range(1000):
    world.step(render=True)

simulation_app.close()
```

---

## ⚠️ 일반적인 에러와 해결책

### 에러 1: `SingleArticulation.__init__() got an unexpected keyword argument 'usd_path'`

**원인**: 5.1에서 `usd_path` 파라미터 제거됨

**해결**:
```python
# 잘못된 방식
robot = SingleArticulation(prim_path=..., usd_path=...)  # ❌

# 올바른 방식
add_reference_to_stage(usd_path=..., prim_path=...)
robot = SingleArticulation(prim_path=..., name=...)  # ✅
```

### 에러 2: `SingleArticulation has no attribute 'set_joint_position_targets'`

**원인**: 5.1에서 메서드명 변경됨

**해결**:
```python
# 잘못된 방식
robot.set_joint_position_targets(target)  # ❌

# 올바른 방식
robot.set_joint_positions(target)  # ✅
```

### 에러 3: `Translation/orientation not supported`

**원인**: `SingleArticulation`은 `translation`, `orientation` 파라미터 지원 안함

**해결**: USD 파일 내에서 위치 설정하거나, `XFormPrim`으로 래핑

### 에러 3: `Camera image is None`

**원인**: 카메라 warm-up 시간 필요

**해결**:
```python
camera.initialize()
for _ in range(5):  # warm-up 스텝
    world.step(render=True)
rgba = camera.get_rgba()
```

---

## 📚 참고 자료

- [Isaac Sim 5.1 Release Notes](https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html)
- [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/)
- 프로젝트 리소스: `resources/isaac_sim_5_1_cheatsheet.md`

---

*마지막 업데이트: 2025-12-29 17:50 KST*
