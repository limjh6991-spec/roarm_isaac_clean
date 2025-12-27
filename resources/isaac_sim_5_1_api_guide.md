# Isaac Sim 5.1 API 변경 사항 및 VRL 가이드

> **작성일**: 2025-12-27  
> **Isaac Sim 버전**: 5.1.0 GA  
> **Isaac Lab 버전**: 2.3.0 (권장)

---

## 📋 요약

Isaac Sim 5.1은 **Core Experimental API**를 도입하여 기존 API를 대체합니다. 이전 `omni.isaac.*` 네임스페이스는 `isaacsim.*`로 변경되었으며, 6.0에서 완전히 제거될 예정입니다.

**중요한 변경점**:
1. `omni.isaac.core` → `isaacsim.core.api`
2. `use_flatcache` → `use_fabric` (PhysX 파라미터)
3. Isaac Lab이 RL 워크플로우의 공식 프레임워크로 권장됨
4. URDF importer의 "merge joints" 플래그 지원 중단

---

## 🔄 API 마이그레이션 매핑

### Core API 변경

| 이전 (4.x / 5.0) | 최신 (5.1) |
|------------------|------------|
| `omni.isaac.core` | `isaacsim.core.api` |
| `omni.isaac.core.world.World` | `isaacsim.core.api.world.World` |
| `omni.isaac.core.articulations.Articulation` | `isaacsim.core.prims.SingleArticulation` |
| `omni.isaac.core.objects.DynamicCuboid` | `isaacsim.core.api.objects.DynamicCuboid` |
| `omni.isaac.sensor` | `isaacsim.sensors.*` |
| `omni.isaac.core_nodes` | `isaacsim.core.nodes` |
| `omni.isaac.surface_gripper` | `isaacsim.robot.surface_gripper` |

### SingleArticulation API 변경

```python
# ❌ 이전 방식 (deprecated)
from omni.isaac.core.articulations import Articulation
robot = Articulation(prim_path="/World/Robot", usd_path="robot.usd")

# ✅ 5.1 방식 - World.scene.add() 사용
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation

world = World()
robot = world.scene.add(
    SingleArticulation(
        prim_path="/World/Robot",
        name="robot",
        # usd_path는 사전에 stage에 로드해야 함
    )
)
```

### USD 로드 방식 변경

```python
# USD 파일을 먼저 stage에 추가
import omni.usd
from pxr import UsdGeom

stage = omni.usd.get_context().get_stage()

# USD 파일 참조 추가
prim = stage.DefinePrim("/World/Robot", "Xform")
prim.GetReferences().AddReference("/path/to/robot.usd")

# 그 후 SingleArticulation으로 래핑
robot = SingleArticulation(prim_path="/World/Robot", name="robot")
```

### Camera API 변경

```python
# ✅ 5.1 방식
from isaacsim.sensors.camera import Camera

camera = Camera(
    prim_path="/World/Camera",
    name="vision_camera",
    translation=np.array([1.0, 0.0, 0.8]),
    orientation=np.array([0.0, 0.0, 0.707, 0.707]),
    resolution=(256, 256),
)
camera.initialize()

# RGB 이미지 획득
rgb = camera.get_rgba()  # (H, W, 4) numpy array
```

---

## 🤖 VRL 알고리즘 권장사항

### 알고리즘 선택

| 알고리즘 | 장점 | 단점 | 추천 용도 |
|---------|------|------|----------|
| **PPO** | 병렬 환경에서 빠른 학습, 안정적 | Sample 효율성 낮음 | 시뮬레이터 내 대규모 학습 |
| **SAC** | 높은 Sample 효율성, 섬세한 제어 | 병렬화 최적화 필요 | 실제 로봇 fine-tuning |

### 권장 설정

**Vision-based Manipulation (RoArm-M3 스타일)**:
- **알고리즘**: SAC (Sample 효율성이 중요한 경우) 또는 PPO (대규모 병렬 학습)
- **Policy**: CnnPolicy (vision input)
- **Framework**: Stable Baselines3 또는 Isaac Lab의 내장 RL

### Isaac Lab 사용 권장

Isaac Sim 5.1에서 RL 워크플로우는 **Isaac Lab**을 통해 수행하는 것이 권장됩니다:

```bash
# Isaac Lab 설치
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
./isaaclab.sh --install
```

Isaac Lab 주요 클래스:
- `isaaclab.app.AppLauncher` - 시뮬레이션 초기화
- `isaaclab.sim.SimulationContext` - 시뮬레이션 컨텍스트
- `isaaclab.assets.Articulation` - 로봇 제어
- `isaaclab.scene.InteractiveScene` - 씬 관리

---

## ⚠️ 주의사항

### 1. SimulationApp 초기화 순서
```python
# ✅ 반드시 SimulationApp을 먼저 초기화
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

# 그 후 다른 Isaac 모듈 import
from isaacsim.core.api.world import World
```

### 2. ForceField API 변경
```python
# ❌ 이전
# ForceFieldAPI 사용

# ✅ 5.1
# PhysxForceFieldAPI 사용
```

### 3. use_flatcache → use_fabric
```python
# ❌ 이전
SimulationContext(use_flatcache=True)

# ✅ 5.1
SimulationContext(use_fabric=True)
```

---

## 📚 참고 자료

1. [Isaac Sim 5.1 Release Notes](https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html)
2. [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/)
3. [Isaac Lab 2.3 Release Notes](https://isaac-sim.github.io/IsaacLab/main/source/changelog.html)
4. [NVIDIA Developer Blog - Isaac Sim 5.1](https://developer.nvidia.com/blog/)

---

## 🔧 프로젝트 스크립트 업데이트 필요 항목

### 1. `envs/simple_vision_env.py`
- [ ] `from isaacsim.core.api.articulations import Articulation` → 수정 필요
- [ ] Isaac Lab의 `ArticulationCfg` 대신 native API 사용
- [ ] Camera 초기화 로직 수정

### 2. `scripts/test/test_vision_env.py`
- [ ] AppLauncher 대신 SimulationApp 사용
- [ ] import 경로 업데이트

### 3. `scripts/train/train_vision_sac.py`
- [ ] Environment wrapper 업데이트
- [ ] Isaac Lab 호환성 확인

---

*이 문서는 2025-12-27 기준으로 작성되었습니다.*
