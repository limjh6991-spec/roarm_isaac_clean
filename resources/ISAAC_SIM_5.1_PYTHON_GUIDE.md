# Isaac Sim 5.1.0 Python 스크립팅 가이드

**최종 업데이트**: 2025년 12월 8일

---

## 📖 SimulationApp 사용법

### 기본 사용 예시

```python
from isaacsim import SimulationApp

# Headless 모드로 시작 (GUI 없음)
simulation_app = SimulationApp({"headless": True})

### 중요: 모든 Omniverse import는 SimulationApp 초기화 후에 해야 함 ###
from isaacsim.core.api import World
import omni.isaac.core.utils.prims as prim_utils

# 시뮬레이션 코드...
world = World()
world.scene.add_default_ground_plane()
world.reset()

for i in range(100):
    world.step(render=False)

# 정리
simulation_app.close()
```

### SimulationApp 설정 옵션

```python
CONFIG = {
    "headless": True,           # GUI 없이 실행
    "width": 1280,              # 뷰포트 너비
    "height": 720,              # 뷰포트 높이
    "anti_aliasing": 0,         # 안티앨리어싱 (0=Off, 1-4=레벨)
    "renderer": "RayTracedLighting",  # 렌더러 (PathTracing, RayTracedLighting)
    "max_gpu_memory_fraction": 0.8,   # GPU 메모리 사용 제한
    "physics_gpu": 0,           # Physics용 GPU ID
    "active_gpu": 0,            # 렌더링용 GPU ID
}

simulation_app = SimulationApp(CONFIG)
```

### 5.1.0 새 기능: skip_cleanup

```python
# 종료 시 cleanup 건너뛰기 (빠른 종료가 필요할 때)
simulation_app.close(skip_cleanup=True)
```

---

## 🏗️ Experience Files

Isaac Sim 5.1.0은 용도별 다양한 experience 파일 제공:

### 파일 목록

| 파일 | 용도 | 특징 |
|------|------|------|
| `isaacsim.exp.base.python.kit` | Python 스크립팅 | 최소 로딩, 빠른 시작 |
| `isaacsim.exp.base.kit` | 기본 시뮬레이션 | 중간 무게 |
| `isaacsim.exp.full.kit` | 전체 GUI | 모든 기능 |
| `isaacsim.exp.full.streaming.kit` | WebRTC 스트리밍 | Docker 기본값 |
| `isaacsim.exp.base.zero_delay.kit` | Zero delay | 지연 최소화 |

### Experience 파일 지정 방법

```python
from isaacsim import SimulationApp

# 특정 experience 파일 사용
simulation_app = SimulationApp({
    "headless": True,
    "experience": "isaacsim.exp.base.python.kit"
})
```

---

## 🐳 Docker에서 실행

### 기본 실행 (느림 - streaming 모드)

```bash
docker run --rm --gpus all \
  -e ACCEPT_EULA=Y \
  -v $(pwd):/workspace \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  /isaac-sim/python.sh /workspace/script.py
```

### 경량 모드 실행 (권장)

```bash
docker run --rm --gpus all \
  -e ACCEPT_EULA=Y \
  --entrypoint "" \
  -v $(pwd):/workspace \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  /isaac-sim/kit/kit \
  /isaac-sim/apps/isaacsim.exp.base.python.kit \
  --no-window \
  --exec "/workspace/script.py"
```

### python.sh 직접 사용 (experience 지정)

```bash
docker run --rm --gpus all \
  -e ACCEPT_EULA=Y \
  -v $(pwd):/workspace \
  --entrypoint "" \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  bash -c "cd /isaac-sim && ./python.sh /workspace/script.py"
```

---

## 📦 확장 기능 활성화

### 코드에서 확장 활성화

```python
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension

# UI 확장 활성화
enable_extension("omni.kit.widget.stage")
enable_extension("omni.kit.widget.layers")

# ROS 2 Bridge 활성화
enable_extension("isaacsim.ros2.bridge")

simulation_app.update()
```

### Experience 파일에서 확장 추가

```toml
# apps/my_custom.kit
[dependencies]
"omni.kit.window.stage" = {}
"omni.kit.widget.layers" = {}
"isaacsim.ros2.bridge" = {}
```

---

## 🔄 5.1.0 API 마이그레이션

### 이전 (4.x) → 새 API (5.1)

```python
# === 이전 방식 (deprecated, 6.0에서 제거 예정) ===
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.sensor import Camera
from omni.isaac.manipulators import SingleManipulator

# === 새 방식 (5.1+) ===
from isaacsim import SimulationApp
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
from isaacsim.robot.manipulators import SingleManipulator
```

### World 생성

```python
from isaacsim.core.api import World

# 기본 생성
world = World()

# 상세 설정
world = World(
    stage_units_in_meters=1.0,
    physics_dt=1.0/60.0,
    rendering_dt=1.0/60.0,
    backend="torch",  # numpy, torch, warp
    device="cuda:0",
)

# 물리 시뮬레이션 루프
world.reset()
for i in range(1000):
    world.step(render=True)

# 정리
world.clear()
```

### 카메라 생성

```python
from isaacsim.sensors.camera import Camera

camera = Camera(
    prim_path="/World/camera",
    position=[2.0, 2.0, 1.5],
    frequency=30,
    resolution=(640, 480),
)

world.reset()
camera.initialize()

# 렌더링 후 데이터 가져오기
world.step(render=True)
rgba = camera.get_rgba()
depth = camera.get_depth()
```

---

## ⚠️ 중요 주의사항

### 1. Import 순서
```python
# ❌ 잘못된 순서 - 오류 발생
from isaacsim.core.api import World
from isaacsim import SimulationApp  # 너무 늦음!

# ✅ 올바른 순서
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
# 이제 다른 import 가능
from isaacsim.core.api import World
```

### 2. Headless 모드에서 matplotlib
```python
# headless 모드에서 matplotlib window 호출 주의
import matplotlib
matplotlib.use('Agg')  # 백엔드 변경 필요
import matplotlib.pyplot as plt
```

### 3. 종료 처리
```python
import os

try:
    # 시뮬레이션 코드
    pass
finally:
    simulation_app.close()
    # hang 방지를 위한 강제 종료
    os._exit(0)
```

### 4. GPU 메모리 관리
```python
# GPU 메모리 사용량 제한
CONFIG = {
    "headless": True,
    "max_gpu_memory_fraction": 0.5,  # 50%만 사용
}
```

---

## 🔗 참고 문서

- [Python Environment 공식 문서](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/manual_standalone_python.html)
- [Core API Overview](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/core_api_overview.html)
- [API Reference](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_python_api.html)
