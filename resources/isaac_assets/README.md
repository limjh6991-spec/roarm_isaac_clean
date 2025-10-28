# Isaac Assets Resources

Phase 2를 위한 Isaac Sim/Lab Assets 활용 및 생성 자료 모음입니다.

## 목표
- Isaac Sim에서 USD Scene 구성 방법 학습
- 농업 환경 (고추밭) 3D 모델 생성 또는 수집
- Isaac Lab Scene API 활용

---

## 1. NVIDIA Isaac Sim 공식 문서

### Isaac Sim Assets 개요
- **공식 문서**: https://docs.omniverse.nvidia.com/isaacsim/latest/
- **Asset 경로**: `omniverse://localhost/NVIDIA/Assets/Isaac/`

### 기본 Asset 카테고리
1. **Robots**: UR10, Franka, Ant, Quadruped
2. **Environments**: Warehouse, Office, Hospital
3. **Props**: Boxes, Bins, Shelves, Tables
4. **People**: Human characters (animated)

### Asset 경로 예시
```python
# Isaac Sim에서 기본 Asset 로드
from omni.isaac.core.utils.stage import add_reference_to_stage

# UR10 로봇
ur10_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
add_reference_to_stage(usd_path=ur10_path, prim_path="/World/UR10")

# Warehouse 환경
warehouse_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"
add_reference_to_stage(usd_path=warehouse_path, prim_path="/World/Warehouse")
```

---

## 2. USD (Universal Scene Description) 기초

### USD란?
- **개발**: Pixar (오픈소스)
- **용도**: 3D 씬 구성, 애니메이션, 물리 시뮬레이션
- **형식**: `.usd`, `.usda` (ASCII), `.usdc` (binary)

### USD Scene 구조
```
/World (Stage Root)
├── /Environment
│   ├── /Ground
│   ├── /Lighting
│   └── /Background
├── /Robot
│   ├── /Base
│   ├── /Arm
│   └── /Gripper
└── /Objects
    ├── /Cube
    ├── /Target
    └── /Obstacles
```

### USD Python API 기초
```python
from pxr import Usd, UsdGeom, Gf

# Stage 열기
stage = Usd.Stage.Open("scene.usd")

# Prim 생성 (Cube)
cube_prim = UsdGeom.Cube.Define(stage, "/World/Cube")
cube_prim.CreateSizeAttr(0.5)  # 0.5m 크기
cube_prim.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.5))  # 위치

# Color 설정
cube_prim.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])  # Red

# 저장
stage.GetRootLayer().Save()
```

---

## 3. Isaac Lab Scene API

### Isaac Lab이란?
- **Repository**: https://github.com/isaac-sim/IsaacLab
- **설명**: Isaac Sim 위에 구축된 RL 프레임워크
- **Scene API**: Modular scene composition

### Scene 구성 예시
```python
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.utils import configclass

@configclass
class MySceneCfg(InteractiveSceneCfg):
    """농업 환경 Scene Configuration"""
    
    # Ground plane
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    
    # Dome light
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=2000),
    )
    
    # 고추 식물 (예시)
    pepper_plant = AssetBaseCfg(
        prim_path="/World/PepperPlant",
        spawn=UsdFileCfg(usd_path="path/to/pepper_plant.usd"),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.5, 0, 0)),
    )
    
    # RoArm-M3 로봇
    robot = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path="path/to/roarm_m3.usd"),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, 0),
            joint_pos={".*": 0.0},
        ),
    )

# Scene 생성
scene = InteractiveScene(MySceneCfg())
```

---

## 4. 농업 환경 (고추밭) 생성 가이드

### 4.1. 3D 모델 소스

#### 무료 3D 모델 웹사이트
1. **Sketchfab**
   - URL: https://sketchfab.com/
   - 검색: "pepper plant", "chili plant", "vegetable"
   - 라이선스: CC BY (상업 이용 가능)
   - 형식: FBX, OBJ, GLB

2. **TurboSquid**
   - URL: https://www.turbosquid.com/
   - 카테고리: Plants → Vegetables
   - 필터: "Free" 또는 Low-cost

3. **Free3D**
   - URL: https://free3d.com/
   - 검색: "farm", "crop", "plant"

4. **Poly Haven**
   - URL: https://polyhaven.com/models
   - CC0 라이선스 (완전 무료)
   - HDR 환경 맵 포함

#### Procedural Generation (절차적 생성)
- **Blender SpeedTree**: 식물 생성 애드온
- **Houdini**: 농업 환경 procedural 생성

### 4.2. Blender에서 USD로 변환

```bash
# Blender 3.6+ (USD 지원)
# 1. FBX/OBJ 임포트
# 2. Material 설정 (Principled BSDF)
# 3. Export → Universal Scene Description (.usdc)

# 또는 Command Line
blender --background --python convert_to_usd.py -- input.fbx output.usd
```

**convert_to_usd.py** 예시:
```python
import bpy
import sys

# Load FBX
argv = sys.argv[sys.argv.index("--") + 1:]
input_file = argv[0]
output_file = argv[1]

bpy.ops.import_scene.fbx(filepath=input_file)

# Export USD
bpy.ops.wm.usd_export(
    filepath=output_file,
    selected_objects_only=False,
    export_materials=True,
)

print(f"Converted {input_file} → {output_file}")
```

### 4.3. Isaac Sim에서 USD 최적화

#### PhysX Material 추가
```python
from pxr import UsdPhysics, PhysxSchema

# Rigid Body 추가
stage = omni.usd.get_context().get_stage()
pepper_prim = stage.GetPrimAtPath("/World/PepperPlant")

# Collision Shape
collision_api = UsdPhysics.CollisionAPI.Apply(pepper_prim)
collision_api.CreateCollisionEnabledAttr(True)

# PhysX Material (마찰, 반발)
physx_material_path = "/World/PhysicsMaterial/PepperMaterial"
physx_material = UsdPhysics.Material.Define(stage, physx_material_path)
physx_material.CreateStaticFrictionAttr(0.8)
physx_material.CreateDynamicFrictionAttr(0.6)
physx_material.CreateRestitutionAttr(0.1)

# Bind Material
collision_api.CreatePhysicsMaterialRel().SetTargets([physx_material_path])
```

#### LOD (Level of Detail) 설정
```python
# 고추 식물이 멀리 있을 때 간단한 메시 사용
from pxr import UsdGeom

model = UsdGeom.Xform.Define(stage, "/World/PepperPlant")
model.GetPrim().CreateAttribute("kind", Sdf.ValueTypeNames.Token).Set("component")

# LOD variants
variant_set = model.GetPrim().GetVariantSets().AddVariantSet("LOD")
variant_set.AddVariant("high")
variant_set.AddVariant("medium")
variant_set.AddVariant("low")
```

---

## 5. 고추밭 Scene 구성

### 5.1. Ground + Rows (밭 구조)

```python
import numpy as np

def create_pepper_field(stage, num_rows=5, num_plants_per_row=10):
    """고추밭 생성 (5 rows x 10 plants)"""
    
    # Ground
    ground_prim = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground_prim.CreateSizeAttr(1.0)
    ground_prim.AddScaleOp().Set(Gf.Vec3d(20, 10, 0.1))  # 20m x 10m x 0.1m
    ground_prim.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    
    # Soil texture
    ground_prim.CreateDisplayColorAttr([(0.4, 0.3, 0.2)])  # Brown
    
    # Pepper plants
    row_spacing = 2.0  # 2m between rows
    plant_spacing = 1.5  # 1.5m between plants
    
    for i in range(num_rows):
        for j in range(num_plants_per_row):
            x = (i - num_rows / 2) * row_spacing
            y = (j - num_plants_per_row / 2) * plant_spacing
            z = 0.0
            
            # Reference pepper plant USD
            plant_path = f"/World/PepperField/Row_{i}/Plant_{j}"
            add_reference_to_stage(
                usd_path="path/to/pepper_plant.usd",
                prim_path=plant_path
            )
            
            # Set position
            plant_prim = stage.GetPrimAtPath(plant_path)
            xformable = UsdGeom.Xformable(plant_prim)
            xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
            
            # Random rotation (±15도)
            rot_z = np.random.uniform(-15, 15)
            xformable.AddRotateZOp().Set(rot_z)
```

### 5.2. Task-specific Objects

```python
def add_harvest_targets(stage, num_peppers=20):
    """수확 대상 고추 추가"""
    
    for i in range(num_peppers):
        # Random position within field
        x = np.random.uniform(-8, 8)
        y = np.random.uniform(-4, 4)
        z = np.random.uniform(0.3, 0.8)  # 30~80cm 높이
        
        # Red pepper (수확 대상)
        pepper_path = f"/World/Peppers/Pepper_{i}"
        pepper = UsdGeom.Capsule.Define(stage, pepper_path)
        pepper.CreateHeightAttr(0.08)  # 8cm
        pepper.CreateRadiusAttr(0.015)  # 1.5cm
        pepper.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        
        # Red color
        pepper.CreateDisplayColorAttr([(0.8, 0.1, 0.1)])
        
        # Physics
        UsdPhysics.CollisionAPI.Apply(pepper.GetPrim())
        rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(pepper.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(pepper.GetPrim())
        mass_api.CreateMassAttr(0.02)  # 20g
```

---

## 6. Domain Randomization (Environment)

### Lighting Randomization
```python
def randomize_lighting(stage):
    """조명 변화 (시간대 시뮬레이션)"""
    
    # Dome Light
    dome_light = stage.GetPrimAtPath("/World/DomeLight")
    intensity = np.random.uniform(800, 2500)  # 800~2500 lux
    color_temp = np.random.uniform(5000, 6500)  # 5000~6500K (daylight)
    
    dome_light.GetAttribute("inputs:intensity").Set(intensity)
    dome_light.GetAttribute("inputs:color_temperature").Set(color_temp)
```

### Weather Effects (날씨 효과)
```python
def add_weather_effects(stage, weather="sunny"):
    """날씨 효과 추가 (흐림, 비 등)"""
    
    if weather == "cloudy":
        # Reduce lighting intensity
        dome_light = stage.GetPrimAtPath("/World/DomeLight")
        dome_light.GetAttribute("inputs:intensity").Set(1200)
    
    elif weather == "rainy":
        # Add particle system (비)
        # Note: Isaac Sim에서는 제한적 지원
        pass
```

### Plant Variations (식물 변화)
```python
def randomize_plant_appearance(plant_prim):
    """식물 외형 랜덤화 (크기, 색상)"""
    
    xformable = UsdGeom.Xformable(plant_prim)
    
    # Scale (±20%)
    scale = np.random.uniform(0.8, 1.2)
    xformable.AddScaleOp().Set(Gf.Vec3d(scale, scale, scale))
    
    # Color variation (녹색 농도)
    green_value = np.random.uniform(0.3, 0.6)
    plant_prim.GetAttribute("primvars:displayColor").Set(
        [(0.1, green_value, 0.1)]
    )
```

---

## 7. Performance Optimization

### Instancing (인스턴싱)
동일한 오브젝트(고추 식물)를 여러 개 생성할 때 메모리 절약:

```python
from pxr import Usd, UsdGeom

# Master instance (원본)
master_path = "/World/PepperMaster"
master = UsdGeom.Xform.Define(stage, master_path)
master.GetPrim().SetInstanceable(True)

# Create instances
for i in range(100):
    instance_path = f"/World/Peppers/Pepper_{i}"
    instance = stage.DefinePrim(instance_path)
    instance.GetReferences().AddReference("", master_path)
    
    # Set position (instance-specific)
    xformable = UsdGeom.Xformable(instance)
    xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
```

### Collision Simplification
고추 식물의 collision mesh를 단순화:

```python
# Convex Hull Approximation
from pxr import UsdPhysics, PhysxSchema

collision_api = UsdPhysics.MeshCollisionAPI.Apply(plant_prim)
collision_api.CreateApproximationAttr("convexHull")  # 볼록 껍질
```

---

## 8. 학습 로드맵 (Week 5-6)

### Week 5: 농업 환경 구성
**Day 1-2**: 3D 모델 수집
- Sketchfab/TurboSquid에서 고추 식물 모델 다운로드
- Blender에서 USD로 변환
- Isaac Sim에서 임포트 테스트

**Day 3-4**: Scene 구성
- 고추밭 Scene 생성 (5 rows x 10 plants)
- 수확 대상 고추 배치
- Lighting 설정

**Day 5**: Physics & Collision
- PhysX Material 설정
- Collision mesh 최적화
- 로봇-환경 상호작용 테스트

### Week 6: Task 통합 및 학습
**Day 1-2**: Task 정의
- 수확 Task (고추 따기)
- 보상 함수 설계 (harvest_reward)
- Episode 종료 조건

**Day 3-5**: 학습 실험
- Vector observation + 농업 환경
- Vision-based RL + 농업 환경
- 성능 비교 (success_rate)

---

## 9. 추가 자료

### USD 튜토리얼
- **Pixar USD Tutorials**: https://graphics.pixar.com/usd/release/tut_helloworld.html
- **NVIDIA Omniverse USD Guide**: https://docs.omniverse.nvidia.com/materials-and-rendering/latest/usd.html

### Blender → USD Workflow
- **Blender USD Export**: https://docs.blender.org/manual/en/latest/files/import_export/usd.html

### Isaac Lab Examples
- **Isaac Lab GitHub**: https://github.com/isaac-sim/IsaacLab
- **Examples**: `source/standalone/workflows/rl_games/`

### 3D 모델 편집
- **Blender**: https://www.blender.org/ (무료, 오픈소스)
- **MeshLab**: http://www.meshlab.net/ (메시 정리)
- **CloudCompare**: https://www.cloudcompare.org/ (Point cloud)

---

## 10. Isaac Sim Built-in Assets

### 경로별 Asset 리스트
```python
# Isaac Sim에서 사용 가능한 Asset 탐색
import omni.client

def list_isaac_assets(base_path="omniverse://localhost/NVIDIA/Assets/Isaac/"):
    result, entries = omni.client.list(base_path)
    if result == omni.client.Result.OK:
        for entry in entries:
            print(f"- {entry.relative_path}")
    
list_isaac_assets("omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Props/")
```

**주요 Props**:
- Bins (상자)
- Shelves (선반)
- Tables (테이블)
- Pallets (팔레트)

**Environment**:
- Simple_Warehouse (창고)
- Hospital (병원)
- Simple_Room (방)

---

## 다운로드 체크리스트

- [ ] 고추 식물 3D 모델 (Sketchfab/TurboSquid)
- [ ] USD Tutorial (Pixar 공식 문서)
- [ ] Isaac Lab Scene API (GitHub)
- [ ] Blender 3.6+ (USD Export 플러그인)
- [ ] Domain Randomization 예시 코드

**다음 단계**:
1. Sketchfab에서 고추/농작물 모델 검색
2. Blender에서 USD 변환 테스트
3. Isaac Sim에서 고추밭 Scene 생성
4. 로봇 + 농업 환경 통합 테스트

---

**작성일**: 2025-10-28  
**상태**: 리소스 수집 완료, 3D 모델 다운로드 대기
