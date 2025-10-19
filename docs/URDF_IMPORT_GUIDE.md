# Isaac Sim 5.0 URDF Import 공식 가이드
> 공식 문서 및 예제 기반 정리 (2025-10-19)

## 📚 공식 문서 위치

```bash
# Extension 문서
~/isaacsim/exts/isaacsim.asset.importer.urdf/docs/README.md
~/isaacsim/exts/isaacsim.asset.importer.urdf/docs/api.rst

# 공식 예제
~/isaacsim/standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py
~/isaacsim/standalone_examples/tutorials/getting_started_robot.py
```

---

## 🔧 방법 1: 직접 URDF Import (추천)

### **패턴: Isaac Core World 없이 사용**

```python
from isaacsim import SimulationApp
kit = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import omni.kit.commands
from isaacsim.core.prims import Articulation
from pxr import Gf, Sdf, UsdLux, UsdPhysics

# 1. Import Config 생성
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = True  # ← 베이스 고정
import_config.distance_scale = 1.0

# 2. URDF Import 실행
#    ✅ dest_path 파라미터 없음!
#    ✅ get_articulation_root=True 사용
success, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="/path/to/robot.urdf",
    import_config=import_config,
    get_articulation_root=True,  # ← 중요!
)

# 3. Stage 가져오기
stage = omni.usd.get_context().get_stage()

# 4. Physics Scene 설정
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)

# 5. Ground Plane 추가
from pxr import PhysicsSchemaTools
PhysicsSchemaTools.addGroundPlane(
    stage, "/groundPlane", "Z", 1500, 
    Gf.Vec3f(0, 0, -0.25), Gf.Vec3f(0.5)
)

# 6. Lighting
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(500)

# 7. Simulation 시작
omni.timeline.get_timeline_interface().play()
kit.update()

# 8. Articulation 초기화
art = Articulation(prim_path)
art.initialize()

# 9. Simulation Loop
for frame in range(1000):
    kit.update()

# 10. 종료
omni.timeline.get_timeline_interface().stop()
kit.close()
```

---

## 🌍 방법 2: Isaac Core World 사용

### **패턴: World + USD Reference**

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage

# 1. World 생성
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# 2. 방법 A: USD 파일 사용 (추천)
add_reference_to_stage(
    usd_path="/path/to/robot.usd",
    prim_path="/World/Robot"
)

# 3. Articulation 생성
robot = Articulation(prim_paths_expr="/World/Robot", name="my_robot")

# 4. World 초기화 (중요!)
my_world.reset()

# 5. Simulation Loop
for i in range(100):
    my_world.step(render=True)
    
    # Joint 제어
    robot.set_joint_positions([[0.0, 0.5, -0.5, 0.0]])

simulation_app.close()
```

---

## 🔥 핵심 발견

### 1. **`dest_path` 파라미터는 없음!**
```python
# ❌ 잘못된 방법:
omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="...",
    dest_path="/tmp/robot.usda"  # ← 존재하지 않는 파라미터!
)

# ✅ 올바른 방법:
omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="...",
    import_config=import_config,
    get_articulation_root=True  # ← 이것 사용
)
```

### 2. **`get_articulation_root=True` 필수**
- 이것이 없으면 `prim_path`가 제대로 반환되지 않음
- Articulation root 경로를 반환함

### 3. **Isaac Core World 사용 시 주의사항**
- URDF import **전에** `World()` 생성 필요
- `my_world.reset()` 호출 필수
- Physics scene은 World가 자동 생성

### 4. **URDF → USD 변환 워크플로우**
```bash
# 방법 1: 직접 import (매번 파싱)
URDFParseAndImportFile → Stage에 직접 추가

# 방법 2: USD 저장 후 재사용 (추천)
1. URDF import 실행
2. Stage를 USD로 저장
3. 이후 add_reference_to_stage()로 빠르게 로딩
```

---

## 📋 ImportConfig 주요 옵션

```python
import_config.merge_fixed_joints = False     # Fixed joint 병합 여부
import_config.convex_decomp = False          # Convex decomposition
import_config.import_inertia_tensor = True   # Inertia tensor import
import_config.fix_base = True                # 베이스 링크 고정
import_config.self_collision = False         # Self collision
import_config.distance_scale = 1.0           # 스케일 (미터)
import_config.default_drive_type = 0         # Drive type
import_config.default_drive_strength = 1e7   # Drive strength
import_config.default_position_drive_damping = 1e5  # Damping
```

---

## 🐛 일반적인 문제 해결

### 문제 1: "로봇 로딩" 후 조용히 종료
**원인:** `dest_path` 파라미터 사용
**해결:** `get_articulation_root=True` 사용

### 문제 2: Physics가 동작하지 않음
**원인:** Physics scene이 없거나 timeline이 시작되지 않음
**해결:**
```python
# 방법 1: 수동 physics scene
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))

# 방법 2: World 사용
my_world = World()  # 자동으로 physics scene 생성
```

### 문제 3: World.reset() 호출 시 오류
**원인:** URDF가 World 생성 후에 import됨
**해결:** World 생성 → URDF import → World.reset() 순서 유지

### 문제 4: Print 출력이 안 보임
**해결:**
```python
import sys
sys.stdout = sys.stderr
```

---

## 🎯 Best Practices

### 1. **개발 단계**: 직접 URDF Import
```python
# 빠른 테스트용
success, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path="robot.urdf",
    import_config=import_config,
    get_articulation_root=True
)
```

### 2. **프로덕션 단계**: USD 변환
```python
# 1. URDF → USD 변환 (1회만)
# Isaac Sim GUI에서: File → Import → URDF

# 2. USD 파일 재사용
add_reference_to_stage(
    usd_path="robot.usd",
    prim_path="/World/Robot"
)
```

### 3. **강화학습 환경**: World + USD
```python
class MyEnv:
    def __init__(self):
        self.world = World()
        self.world.scene.add_default_ground_plane()
        
        # USD 사용 (빠름)
        add_reference_to_stage(
            usd_path=self.cfg.robot_usd,
            prim_path="/World/Robot"
        )
        
        self.robot = self.world.scene.add(
            Articulation(prim_path="/World/Robot", name="robot")
        )
        
    def reset(self):
        self.world.reset()  # Physics도 리셋됨
```

---

## 📖 참고 자료

1. **공식 Extension 문서**
   ```bash
   cat ~/isaacsim/exts/isaacsim.asset.importer.urdf/docs/README.md
   ```

2. **공식 예제 코드**
   ```bash
   cat ~/isaacsim/standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py
   ```

3. **Tutorial**
   ```bash
   cat ~/isaacsim/standalone_examples/tutorials/getting_started_robot.py
   ```

4. **API Reference**
   ```bash
   cat ~/isaacsim/exts/isaacsim.asset.importer.urdf/docs/api.rst
   ```

---

## 🔧 디버깅 체크리스트

- [ ] `SimulationApp` 생성 완료
- [ ] `URDFCreateImportConfig` 실행 완료
- [ ] `get_articulation_root=True` 사용
- [ ] `dest_path` 파라미터 **사용하지 않음**
- [ ] Physics scene 생성 또는 World 사용
- [ ] Timeline 시작 (`play()` 또는 `world.reset()`)
- [ ] `sys.stdout = sys.stderr` 설정 (출력 확인)
- [ ] `kit.update()` 호출 (시뮬레이션 진행)

---

**작성일:** 2025-10-19  
**버전:** Isaac Sim 5.0.0-rc.45  
**참고:** 공식 문서 및 예제 기반
