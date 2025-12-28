# Isaac Sim 5.0 + IsaacLab API Guide

**작성일**: 2025-11-02  
**대상**: RoArm-M3 + D405 Vision RL 프로젝트

## 📋 목차

1. [IsaacLab 개요](#1-isaaclab-개요)
2. [Articulation API 변경사항](#2-articulation-api-변경사항)
3. [URDF Import 방법](#3-urdf-import-방법)
4. [Robot 초기화 및 제어](#4-robot-초기화-및-제어)
5. [예제 코드](#5-예제-코드)

---

## 1. IsaacLab 개요

### IsaacLab이란?

IsaacLab은 Isaac Sim 5.0을 위한 **공식 Python 라이브러리**입니다:
- Isaac Sim의 복잡한 API를 간소화
- Articulation, Sensor, Controller 등 통합 인터페이스 제공
- RL 학습을 위한 Environment 구조 제공

### 설치 방법

```bash
# 1. GitHub 저장소 클론
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git

# 2. Isaac Sim 링크 생성
cd ~/IsaacLab
ln -s ~/isaacsim _isaac_sim

# 3. IsaacLab 설치
./isaaclab.sh --install
```

### 주요 변경사항

| Isaac Sim 4.x | Isaac Sim 5.0 + IsaacLab |
|---------------|--------------------------|
| `omni.isaac.core.articulations.Articulation` | `isaaclab.assets.Articulation` |
| `omni.isaac.core.World` | `isaaclab.sim.SimulationContext` |
| `get_world_pose()` | `data.root_pose_w` |
| `set_joint_positions()` | `write_joint_position_to_sim()` |

---

## 2. Articulation API 변경사항

### 2.1 Articulation 클래스

**이전 방식 (omni.isaac.core)**:
```python
from omni.isaac.core.articulations import Articulation

robot = Articulation(prim_path="/World/robot")
robot.initialize()

# 속성 접근
pos = robot.get_world_pose()
joint_pos = robot.get_joint_positions()
```

**IsaacLab 방식**:
```python
from isaaclab.assets import Articulation, ArticulationCfg

# Configuration 기반 생성
robot_cfg = ArticulationCfg(
    prim_path="/World/robot",
    spawn=...,  # USD/URDF 로딩 설정
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1),
        joint_pos={"joint1": 0.0, ...}
    ),
)

robot = Articulation(cfg=robot_cfg)

# 속성 접근 (data 속성을 통해)
pos = robot.data.root_pos_w
joint_pos = robot.data.joint_pos
```

### 2.2 핵심 속성

#### ArticulationData 주요 속성

```python
# Root 상태
robot.data.root_pos_w          # (N, 3) - 월드 좌표계 위치
robot.data.root_quat_w         # (N, 4) - 월드 좌표계 회전 (w,x,y,z)
robot.data.root_vel_w          # (N, 6) - 선속도 + 각속도
robot.data.root_state_w        # (N, 13) - pos + quat + vel

# Joint 상태
robot.data.joint_pos           # (N, DOF) - Joint 위치
robot.data.joint_vel           # (N, DOF) - Joint 속도
robot.data.joint_acc           # (N, DOF) - Joint 가속도

# Default 상태
robot.data.default_root_state  # 초기 root 상태
robot.data.default_joint_pos   # 초기 joint 위치
robot.data.default_joint_vel   # 초기 joint 속도

# Body 상태
robot.data.body_pos_w          # (N, B, 3) - 각 링크 위치
robot.data.body_quat_w         # (N, B, 4) - 각 링크 회전
robot.data.body_vel_w          # (N, B, 6) - 각 링크 속도

# Jacobian
robot.data.jacobian_w          # (N, B, 6, DOF) - 월드 좌표계 Jacobian
```

### 2.3 데이터 쓰기 메서드

```python
# Root Pose 설정
robot.write_root_pose_to_sim(root_pose)  # (N, 7): pos + quat
robot.write_root_velocity_to_sim(root_vel)  # (N, 6): lin_vel + ang_vel
robot.write_root_state_to_sim(root_state)  # (N, 13): pose + vel

# Joint 설정
robot.write_joint_position_to_sim(joint_pos)  # (N, DOF)
robot.write_joint_velocity_to_sim(joint_vel)  # (N, DOF)
robot.write_joint_state_to_sim(joint_pos, joint_vel)  # 둘 다

# 액추에이터 명령 설정
robot.set_joint_position_target(target)
robot.set_joint_velocity_target(target)
robot.set_joint_effort_target(target)

# Simulation에 반영
robot.write_data_to_sim()  # 액추에이터 명령 적용
```

### 2.4 업데이트 및 리셋

```python
# 매 시뮬레이션 스텝마다 호출
robot.update(dt)  # 내부 버퍼 업데이트

# 리셋
robot.reset()  # 내부 버퍼 초기화 (Simulation 상태는 변경 안 함)
```

---

## 3. URDF Import 방법

### 3.1 IsaacLab UrdfConverter 사용 (권장)

```python
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
import isaaclab.sim as sim_utils

# 설정
urdf_cfg = UrdfConverterCfg(
    asset_path="/path/to/robot.urdf",
    usd_dir="/path/to/output",
    usd_file_name="robot.usd",
    fix_base=False,  # True면 베이스 고정
    merge_fixed_joints=True,
    self_collision=False,
    make_instanceable=True,
)

# 변환 실행
converter = UrdfConverter(urdf_cfg)
converter.convert()

# USD 파일 경로
usd_path = converter.usd_path
print(f"USD created at: {usd_path}")
```

### 3.2 ArticulationCfg에서 URDF 사용

```python
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
import isaaclab.sim as sim_utils

robot_cfg = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path="/path/to/robot.usd",  # 변환된 USD 파일
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1),  # 초기 위치 (z 높이 설정 중요!)
        joint_pos={
            "joint1": 0.0,
            "joint2": -1.57,
            "joint3": 1.57,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint.*"],
            effort_limit=100.0,
            velocity_limit=2.0,
            stiffness=1000.0,
            damping=100.0,
        ),
    },
)
```

### 3.3 직접 URDF Import (비권장)

```python
from isaacsim.asset.importer.urdf import _urdf
import omni.kit.commands

# ImportConfig 생성
import_config = _urdf.ImportConfig()
import_config.set_fix_base(False)
import_config.set_merge_fixed_joints(True)

# Parse
success, robot_model = omni.kit.commands.execute(
    "URDFParseFile",
    urdf_path="/path/to/robot.urdf",
    import_config=import_config,
)

# Import
success, prim_path = omni.kit.commands.execute(
    "URDFImportRobot",
    urdf_path="/path/to/robot.urdf",
    urdf_robot=robot_model,
    import_config=import_config,
)
```

**⚠️ 주의사항**:
- `dest_path` 파라미터는 사용하지 않음 (에러 발생)
- 로봇은 `/World` 루트 아래에 자동 생성됨
- `prim_path`는 생성된 경로를 반환함

---

## 4. Robot 초기화 및 제어

### 4.1 SimulationContext 사용

```python
from isaaclab.sim import SimulationContext, SimulationCfg

# Simulation 설정
sim_cfg = SimulationCfg(
    dt=1.0/60.0,  # 60 Hz
    device="cuda:0",
    physics_material=...,
)

sim = SimulationContext(sim_cfg)

# 카메라 설정
sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

# 시뮬레이션 시작
sim.reset()
```

### 4.2 Robot 추가 및 초기화

```python
from isaaclab.assets import Articulation

# Robot 생성
robot = Articulation(cfg=robot_cfg)

# Ground plane 추가
from isaaclab.sim.spawners.ground_plane import GroundPlaneCfg, spawn_ground_plane

ground_cfg = GroundPlaneCfg()
spawn_ground_plane("/World/ground", ground_cfg)

# Simulation reset
sim.reset()

# Robot 초기화 (Simulation reset 후 필수!)
robot.update(sim.cfg.dt)
```

### 4.3 Robot 제어 루프

```python
import torch

# Simulation loop
while simulation_app.is_running():
    # Reset 주기
    if count % 500 == 0:
        # Root 상태 리셋
        root_state = robot.data.default_root_state.clone()
        robot.write_root_pose_to_sim(root_state[:, :7])
        robot.write_root_velocity_to_sim(root_state[:, 7:])
        
        # Joint 상태 리셋
        joint_pos = robot.data.default_joint_pos.clone()
        joint_vel = robot.data.default_joint_vel.clone()
        robot.write_joint_state_to_sim(joint_pos, joint_vel)
        
        # 내부 버퍼 리셋
        robot.reset()
    
    # 액추에이터 명령 생성
    target_pos = torch.randn_like(robot.data.joint_pos) * 0.1
    robot.set_joint_position_target(target_pos)
    
    # Simulation에 명령 적용
    robot.write_data_to_sim()
    
    # Simulation step
    sim.step()
    
    # Robot 상태 업데이트
    robot.update(sim.cfg.dt)
    
    count += 1
```

---

## 5. 예제 코드

### 5.1 RoArm-M3 Configuration

```python
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
import isaaclab.sim as sim_utils

ROARM_M3_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path="/path/to/roarm_m3_with_d405.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1),  # 바닥에서 10cm 위
        joint_pos={
            "joint1": 0.0,       # 정면
            "joint2": -1.57,     # -90도 (수평)
            "joint3": 1.57,      # +90도 (수직) - 'ㄱ'자
            "joint4": 0.0,
            "joint5": 0.0,
            "gripper": 0.01,     # 약간 열림
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-5]"],
            effort_limit=10.0,
            velocity_limit=2.0,
            stiffness=100.0,
            damping=10.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["gripper"],
            effort_limit=5.0,
            velocity_limit=1.0,
            stiffness=50.0,
            damping=5.0,
        ),
    },
)
```

### 5.2 완전한 예제 스크립트

```python
#!/usr/bin/env python
"""RoArm-M3 with D405 Camera - IsaacLab Example"""

import argparse
from isaaclab.app import AppLauncher

# Argument parser
parser = argparse.ArgumentParser(description="RoArm-M3 + D405 Test")
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows"""

import torch
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab.sim.spawners.ground_plane import GroundPlaneCfg, spawn_ground_plane

def main():
    # Simulation 설정
    sim_cfg = SimulationCfg(dt=1.0/60.0, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.0, 2.0, 2.0], [0.0, 0.0, 0.0])
    
    # Ground plane 생성
    spawn_ground_plane("/World/ground", GroundPlaneCfg())
    
    # Robot 생성
    robot = Articulation(cfg=ROARM_M3_CFG)
    
    # Simulation 시작
    sim.reset()
    robot.update(sim.cfg.dt)
    
    print("✅ Setup complete!")
    
    # Simulation loop
    count = 0
    while simulation_app.is_running():
        # Reset
        if count % 500 == 0:
            root_state = robot.data.default_root_state.clone()
            robot.write_root_pose_to_sim(root_state[:, :7])
            robot.write_root_velocity_to_sim(root_state[:, 7:])
            
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            
            robot.reset()
            print(f"[{count}] Reset")
        
        # Control (여기에 RL 액션 추가 가능)
        target_pos = robot.data.default_joint_pos.clone()
        robot.set_joint_position_target(target_pos)
        robot.write_data_to_sim()
        
        # Step
        sim.step()
        robot.update(sim.cfg.dt)
        count += 1
    
    simulation_app.close()

if __name__ == "__main__":
    main()
```

---

## 📚 참고 자료

### IsaacLab GitHub
- **저장소**: https://github.com/isaac-sim/IsaacLab
- **문서**: https://isaac-sim.github.io/IsaacLab/
- **예제**: `scripts/tutorials/` 디렉토리
- **Assets**: `source/isaaclab_assets/` 디렉토리

### 핵심 튜토리얼
1. `scripts/tutorials/01_assets/run_articulation.py` - Articulation 기본
2. `scripts/tutorials/05_controllers/run_diff_ik.py` - IK Controller
3. `scripts/demos/arms.py` - 다양한 로봇 팔 예제

### API 문서
- **Articulation**: `source/isaaclab/isaaclab/assets/articulation/`
- **Simulation**: `source/isaaclab/isaaclab/sim/`
- **Converters**: `source/isaaclab/isaaclab/sim/converters/`

---

## ⚠️ 주의사항

### 1. 초기 위치 설정
```python
# ❌ 잘못된 방법 - 로봇이 바닥에 묻힘
init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0))

# ✅ 올바른 방법 - 로봇 베이스 높이를 고려
init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.1))
```

### 2. Simulation Reset 순서
```python
# 올바른 순서:
sim.reset()           # 1. Simulation 리셋
robot.update(dt)      # 2. Robot 상태 업데이트
```

### 3. Data 접근
```python
# ❌ 이전 방식
pos = robot.get_world_pose()

# ✅ IsaacLab 방식
pos = robot.data.root_pos_w
```

### 4. URDF Fix Base
- `fix_base=True`: 베이스가 공중에 고정 (Mobile 로봇에 부적합)
- `fix_base=False`: 중력 영향을 받음 (기본값)
- RoArm-M3처럼 탁상형 로봇은 `init_state.pos`로 위치 설정

---

**작성**: RoArm-M3 Vision RL Project  
**버전**: Isaac Sim 5.0 + IsaacLab (2025-11)
