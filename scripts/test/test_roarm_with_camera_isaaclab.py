#!/usr/bin/env python3
"""
RoArm-M3 + D405 Camera Test with IsaacLab (Production Version)

원본 kinematic chain을 유지하면서 카메라를 추가한 URDF를 
IsaacLab으로 로드하고 테스트합니다.

Features:
- 원본 roarm_m3.generated.urdf 구조 100% 보존
- D405 카메라를 gripper_link에 fixed joint로 장착
- IsaacLab Articulation API 사용
- 확장 가능한 구조 (향후 torso, 다중 팔 통합 준비)

구조:
    world (fixed, z=0.0701)
    └─> base_link
        └─> link1 -> link2 -> link3 -> link4 -> link5
            └─> gripper_link
                ├─> hand_tcp (fixed)
                └─> camera_link (fixed, xyz="0.05 0 0.02")
                    └─> camera_depth_frame -> camera_color_frame -> optical_frames

작성일: 2025-11-02
버전: v1.0 (Production)
"""

import argparse
import os
from pathlib import Path

# Note: omni.usd, pxr는 런타임에 import됨 (IsaacLab 환경에서만 사용 가능)

# AppLauncher는 다른 import보다 먼저 실행되어야 함
from isaaclab.app import AppLauncher

# Argument parser
parser = argparse.ArgumentParser(
    description="RoArm-M3 + D405 Camera Test with IsaacLab (Production)"
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch Omniverse App
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows after AppLauncher."""

import torch
import numpy as np
from PIL import Image
from datetime import datetime

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.sensors import Camera, CameraCfg

# ==============================================================================
# Configuration
# ==============================================================================

# 경로 설정
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parents[1]
URDF_PATH = PROJECT_DIR / "assets" / "roarm_m3" / "urdf" / "roarm_m3_with_camera_correct.urdf"
USD_DIR = PROJECT_DIR / "assets" / "roarm_m3" / "usd"
USD_FILE = "roarm_m3_with_camera_correct.usd"
CAMERA_OUTPUT_DIR = PROJECT_DIR / "output" / "camera_images" / datetime.now().strftime("%Y%m%d_%H%M%S")

print("=" * 80)
print("🎥 RoArm-M3 + D405 Camera Test (IsaacLab Production)")
print("=" * 80)
print(f"\n📁 Project: {PROJECT_DIR}")
print(f"📁 URDF: {URDF_PATH}")
print(f"📁 USD Output: {USD_DIR / USD_FILE}")
print(f"📁 Camera Output: {CAMERA_OUTPUT_DIR}")

# URDF 파일 확인
if not URDF_PATH.exists():
    raise FileNotFoundError(f"URDF file not found: {URDF_PATH}")

# USD 디렉토리 생성
USD_DIR.mkdir(parents=True, exist_ok=True)

# 카메라 이미지 출력 디렉토리 생성
CAMERA_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
print(f"✅ Camera output directory created: {CAMERA_OUTPUT_DIR}")

# ==============================================================================
# Robot Configuration
# ==============================================================================

def get_roarm_m3_articulation_cfg() -> ArticulationCfg:
    """
    RoArm-M3 + Camera Articulation Configuration
    
    Returns:
        ArticulationCfg: IsaacLab articulation configuration
    """
    
    # URDF Converter configuration
    converter_cfg = UrdfConverterCfg(
        asset_path=str(URDF_PATH),
        usd_dir=str(USD_DIR),
        usd_file_name=USD_FILE,
        fix_base=True,  # Base를 ground에 고정
        merge_fixed_joints=False,  # Camera frames를 유지하기 위해 False
        collision_from_visuals=False,
        self_collision=False,
        force_usd_conversion=True,
        make_instanceable=True,
        # Joint drive configuration (필수!)
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=400.0,  # 기본 stiffness
                damping=40.0,     # 기본 damping
            ),
        ),
    )
    
    print("\n" + "=" * 80)
    print("STEP 1: URDF to USD Conversion")
    print("=" * 80)
    print("📦 Converting URDF to USD...")
    
    # Run converter
    converter = UrdfConverter(converter_cfg)
    usd_path = converter.usd_path
    
    print(f"✅ USD generated: {usd_path}")
    
    # Create Articulation Configuration
    cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=100.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),  # Ground에 직접 배치 (world_to_base_link가 z=0.0701 offset 제공)
            rot=(1.0, 0.0, 0.0, 0.0),  # Quaternion (w, x, y, z)
            joint_pos={
                # 'ㄱ'자 포즈 설정
                "base_link_to_link1": 0.0,           # 정면
                "link1_to_link2": 0.0,               # 수직
                "link2_to_link3": -0.9,              # 'ㄱ'자 (약 -51.6°)
                "link3_to_link4": 0.0,
                "link4_to_link5": 0.0,
                "link5_to_gripper_link": 0.5,        # 그리퍼 약간 열림
            },
            joint_vel={
                "base_link_to_link1": 0.0,
                "link1_to_link2": 0.0,
                "link2_to_link3": 0.0,
                "link3_to_link4": 0.0,
                "link4_to_link5": 0.0,
                "link5_to_gripper_link": 0.0,
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                # 실제 서보 스펙 기반 (카메라 60g 보상 포함)
                # ST3215/ST3235: 30 kg·cm (2.94 Nm), Shoulder Dual: 60 kg·cm (5.88 Nm)
                stiffness={
                    "base_link_to_link1": 400.0,     # Base: 2.94 Nm × 2 (안전계수)
                    "link1_to_link2": 5000.0,        # Shoulder: ULTRA HIGH (카메라 무게 + 긴 팔 모멘트!)
                    "link2_to_link3": 3000.0,        # Elbow: ULTRA HIGH (카메라 무게 극복!)
                    "link3_to_link4": 2500.0,        # Wrist Pitch: ULTRA HIGH (카메라 끝단 지지!)
                    "link4_to_link5": 400.0,         # Wrist Yaw: 2.94 Nm × 2
                    "link5_to_gripper_link": 250.0,  # Gripper: 1.96 Nm × 2
                },
                damping={
                    "base_link_to_link1": 8.0,       # Critically damped
                    "link1_to_link2": 100.0,         # Shoulder ULTRA HIGH DAMPING (카메라 하중 + 진동 억제!)
                    "link2_to_link3": 80.0,          # Elbow ULTRA HIGH DAMPING (카메라 무게, 진동 억제!)
                    "link3_to_link4": 60.0,          # Wrist Pitch ULTRA HIGH (카메라 끝단 안정화!)
                    "link4_to_link5": 8.0,
                    "link5_to_gripper_link": 5.0,
                },
                armature={
                    "base_link_to_link1": 0.01,
                    "link1_to_link2": 0.1,           # Shoulder: HIGH ARMATURE (관성 증가로 안정화!)
                    "link2_to_link3": 0.05,          # Elbow: MEDIUM ARMATURE
                    "link3_to_link4": 0.03,          # Wrist Pitch: MEDIUM ARMATURE
                    "link4_to_link5": 0.01,
                    "link5_to_gripper_link": 0.005,
                },
                effort_limit={
                    "base_link_to_link1": 100.0,     # 2배 증가!
                    "link1_to_link2": 200.0,         # Shoulder: 4배 증가! (카메라 무게!)
                    "link2_to_link3": 150.0,         # Elbow: 3배 증가!
                    "link3_to_link4": 100.0,         # Wrist Pitch: 3배 증가! (카메라 끝단!)
                    "link4_to_link5": 50.0,          # 증가!
                    "link5_to_gripper_link": 30.0,   # 증가!
                },
                velocity_limit={
                    "base_link_to_link1": 2.0,
                    "link1_to_link2": 2.0,
                    "link2_to_link3": 2.0,
                    "link3_to_link4": 2.0,
                    "link4_to_link5": 2.0,
                    "link5_to_gripper_link": 1.0,
                },
            ),
        },
    )
    
    return cfg


def setup_camera_sensor() -> CameraCfg:
    """
    Intel RealSense D405 카메라 센서 설정
    
    Returns:
        CameraCfg: 카메라 센서 설정
    """
    return CameraCfg(
        prim_path="/World/Robot/gripper_link/camera_link/Camera",
        update_period=0.0,  # 매 프레임 업데이트
        height=480,  # D405 Depth 해상도
        width=640,
        data_types=["rgb", "distance_to_image_plane"],  # RGB + Depth
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=1.88,  # D405: 1.88mm
            focus_distance=400.0,
            horizontal_aperture=3.6,  # 87° FOV 반영
            clipping_range=(0.07, 10.0),  # D405 Range: 7cm~10m
        ),
    )


def capture_camera_image(camera: Camera, motion_idx: int, description: str):
    """
    카메라 이미지 캡처 및 저장
    
    Args:
        camera: Camera 센서 객체
        motion_idx: 동작 인덱스
        description: 동작 설명
    """
    # 카메라 데이터 업데이트
    camera.update(dt=0.0)
    
    # RGB 이미지 추출 (shape: [H, W, 4] - RGBA)
    rgb_data = camera.data.output["rgb"][0].cpu().numpy()  # [0]: 첫 번째 카메라
    rgb_image = (rgb_data[:, :, :3] * 255).astype(np.uint8)  # RGB만 추출 및 0-255 변환
    
    # Depth 이미지 추출 (shape: [H, W, 1])
    depth_data = camera.data.output["distance_to_image_plane"][0].cpu().numpy()
    depth_normalized = (depth_data[:, :, 0] / 10.0 * 255).clip(0, 255).astype(np.uint8)  # 0-10m → 0-255
    
    # 파일 저장
    rgb_filename = CAMERA_OUTPUT_DIR / f"motion_{motion_idx:02d}_{description}_rgb.png"
    depth_filename = CAMERA_OUTPUT_DIR / f"motion_{motion_idx:02d}_{description}_depth.png"
    
    Image.fromarray(rgb_image).save(rgb_filename)
    Image.fromarray(depth_normalized).save(depth_filename)
    
    print(f"  📸 Captured: {rgb_filename.name}, {depth_filename.name}")


# ==============================================================================
# Main Execution
# ==============================================================================

def main():
    """Main function"""
    
    # 1. Simulation 설정
    print("\n" + "=" * 80)
    print("STEP 2: Simulation Setup")
    print("=" * 80)
    print("🌍 Initializing Simulation Context...")
    
    sim_cfg = SimulationCfg(
        dt=1.0 / 60.0,  # 60 Hz
        device="cuda:0" if torch.cuda.is_available() else "cpu",
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )
    sim = SimulationContext(sim_cfg)
    
    # 카메라 뷰 설정
    sim.set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.0, 0.0, 0.3])
    
    print(f"✅ Simulation initialized on {sim.device}")
    
    # 2. Scene 구성
    print("\n" + "=" * 80)
    print("STEP 3: Scene Setup")
    print("=" * 80)
    
    # Ground plane
    print("📐 Creating ground plane...")
    ground_cfg = sim_utils.GroundPlaneCfg(
        color=(0.15, 0.15, 0.15),
        size=(100.0, 100.0),
    )
    ground_cfg.func("/World/ground", ground_cfg)
    print("✅ Ground plane created")
    
    # # 🎨 링크별 색상 정의 (나중에 사용)
    # link_colors = {
    #     "base_link": (0.2, 0.2, 0.2),      # 다크 그레이 (고정 베이스)
    #     "link1": (1.0, 0.0, 0.0),          # 빨강 (Base 회전)
    #     "link2": (1.0, 0.5, 0.0),          # 주황 (Shoulder)
    #     "link3": (1.0, 1.0, 0.0),          # 노랑 (Elbow)
    #     "link4": (0.0, 1.0, 0.0),          # 초록 (Wrist Roll)
    #     "link5": (0.0, 0.5, 1.0),          # 하늘색 (Wrist Pitch)
    #     "gripper_link": (0.5, 0.0, 1.0),   # 보라 (그리퍼)
    #     "camera_link": (1.0, 0.0, 0.5),    # 핑크 (카메라)
    # }
    
    # Light
    print("💡 Creating light...")
    light_cfg = sim_utils.DomeLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )
    light_cfg.func("/World/light", light_cfg)
    print("✅ Light created")
    
    # Robot
    print("🤖 Creating robot...")
    robot_cfg = get_roarm_m3_articulation_cfg()
    robot = Articulation(cfg=robot_cfg)
    print("✅ Robot created at /World/Robot")
    
    # Camera Sensor
    print("📷 Creating camera sensor...")
    camera_cfg = setup_camera_sensor()
    camera = Camera(cfg=camera_cfg)
    print("✅ Camera sensor created")
    
    # # 🎨 링크별 색상 적용 (TODO: 나중에 구현)
    # print("\n⏭️  Skipping color application (will implement later)")
    # # 색상 적용 코드 주석 처리 (omni.usd import 이슈)
    
    # 3. Initialize Scene
    print("\n" + "=" * 80)
    print("STEP 4: Initialize Scene")
    print("=" * 80)
    print("🔄 Resetting simulation...")
    
    sim.reset()
    robot.reset()
    camera.reset()
    
    print("✅ Simulation reset complete")
    
    # 🔥 초기 자세 명시적 설정 (중요!)
    # init_state만으로는 actuator가 그 자세를 유지하지 않음
    # → reset 후 직접 자세를 설정하고 actuator target도 설정해야 함
    print("\n📍 Setting initial pose explicitly...")
    
    # 'ㄱ'자 자세
    desired_joint_pos = torch.tensor(
        [[0.0, 0.0, -0.9, 0.0, 0.0, 0.5]], 
        device=sim.device
    )
    desired_joint_vel = torch.zeros((1, 6), device=sim.device)
    
    # URDF Joint Limits (수동 강제 적용)
    joint_limits_lower = torch.tensor(
        [[-1.5708, -1.5708, -1.0, -1.5708, -3.1416, 0.0]],
        device=sim.device
    )
    joint_limits_upper = torch.tensor(
        [[1.5708, 1.5708, 2.95, 1.5708, 3.1416, 1.5]],
        device=sim.device
    )
    
    # 1. 직접 Joint State 설정 (즉시 적용)
    robot.write_joint_state_to_sim(desired_joint_pos, desired_joint_vel)
    
    # 2. Actuator Target도 설정 (자세 유지)
    robot.set_joint_position_target(desired_joint_pos)
    robot.write_data_to_sim()
    
    # 3. 안정화 (200 프레임 = 10초, 매 프레임마다 target 재설정 + Joint Limit 강제)
    print("   ⏳ Stabilizing for 200 frames (10 seconds, with continuous target enforcement + joint limit clamping)...")
    for frame_idx in range(200):
        # 🔥 Joint Limit 수동 강제 (Isaac Sim이 무시하므로)
        actual_pos = robot.data.joint_pos.clone()
        actual_pos = torch.clamp(actual_pos, joint_limits_lower, joint_limits_upper)
        robot.write_joint_position_to_sim(actual_pos)
        
        # 매 프레임마다 target 재설정 (중력 저항)
        robot.set_joint_position_target(desired_joint_pos)
        robot.write_data_to_sim()
        
        sim.step()
        robot.update(dt=sim_cfg.dt)
        
        # 진행 상황 출력 (10프레임마다)
        if frame_idx % 10 == 0:
            joint_pos_current = robot.data.joint_pos[0].cpu().numpy()
            print(f"      Frame {frame_idx:2d}/50: Joint[2]={joint_pos_current[2]:6.3f} rad ({np.rad2deg(joint_pos_current[2]):6.1f}°)")
    
    print("   ✅ Initial pose set and stabilized")
    
    # 4. Robot State 출력
    print("\n" + "=" * 80)
    print("STEP 5: Robot Information")
    print("=" * 80)
    
    print(f"📊 Robot Information:")
    print(f"   Prim Path: {robot.cfg.prim_path}")
    print(f"   Number of Bodies: {robot.num_bodies}")
    print(f"   Number of DOF: {robot.num_joints}")
    print(f"   Body Names: {robot.body_names}")
    print(f"   Joint Names: {robot.joint_names}")
    
    # Initial state
    root_state = robot.data.root_pos_w[0].cpu().numpy()
    root_quat = robot.data.root_quat_w[0].cpu().numpy()
    joint_pos = robot.data.joint_pos[0].cpu().numpy()
    
    print(f"\n🤖 Initial Robot State:")
    print(f"   Root Position: [{root_state[0]:.3f}, {root_state[1]:.3f}, {root_state[2]:.3f}]")
    print(f"   Root Orientation (wxyz): [{root_quat[0]:.3f}, {root_quat[1]:.3f}, {root_quat[2]:.3f}, {root_quat[3]:.3f}]")
    print(f"\n   Joint Positions:")
    for i, (name, pos) in enumerate(zip(robot.joint_names, joint_pos)):
        print(f"      {name:30s} {pos:7.3f} rad ({np.rad2deg(pos):7.1f}°)")
    
    # 5. Simulation Loop with Joint Motion Test
    print("\n" + "=" * 80)
    print("STEP 6: Camera & Joint Motion Test")
    print("=" * 80)
    
    # 📷 카메라 정보 출력
    print("\n📷 Camera Information:")
    print(f"   Camera should be attached to: {robot.cfg.prim_path}/gripper_link/camera_link")
    print(f"   Camera position offset: x=0.02m (20mm forward from gripper)")
    print(f"   Camera type: Intel RealSense D405")
    print(f"   Camera mass: 72g (14.1% of total robot mass)")
    
    # 🔥 간단한 동작 테스트 (카메라 무게 확인용)
    # 📋 실제 URDF Limits:
    #   Joint 0 (base_link_to_link1):     -1.5708 ~ 1.5708 (±90°)
    #   Joint 1 (link1_to_link2):         -1.5708 ~ 1.5708 (±90°)  
    #   Joint 2 (link2_to_link3):         -1.0 ~ 2.95 (Elbow)
    #   Joint 3 (link3_to_link4):         -1.5708 ~ 1.5708 (±90°)
    #   Joint 4 (link4_to_link5):         -3.1416 ~ 3.1416 (±180°)
    #   Joint 5 (link5_to_gripper_link):  0.0 ~ 1.5 (Gripper)
    
    # 동작 시퀀스 정의 (카메라 무게 테스트)
    motion_sequence = [
        ("초기 'ㄱ'자 자세 (5초 유지)", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 300),
        
        # 카메라 무게 테스트: Shoulder & Elbow 동작
        ("Shoulder 앞으로 +45°", torch.tensor([[0.0, 0.785, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 180),
        ("Shoulder 원위치", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 180),
        ("Elbow 펴기 +90°", torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], device=sim.device), 180),
        ("Elbow 원위치", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 180),
        
        # Base 회전 테스트
        ("Base 우회전 +90°", torch.tensor([[1.57, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 180),
        ("Base 좌회전 -90°", torch.tensor([[-1.57, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 180),
        ("Base 원위치", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 180),
    ]
    
    print("\n▶️  카메라 무게 테스트 시작")
    print(f"   총 동작 수: {len(motion_sequence)}")
    print(f"   목표: 카메라 72g 무게에도 안정적인 동작 확인")
    print()
    
    # 🔥 각 링크 최대 범위 테스트 (URDF Joint Limits 준수)
    # 📋 실제 URDF Limits:
    #   Joint 0 (base_link_to_link1):     -1.5708 ~ 1.5708 (±90°)
    #   Joint 1 (link1_to_link2):         -1.5708 ~ 1.5708 (±90°)  
    #   Joint 2 (link2_to_link3):         -1.0 ~ 2.95 (Elbow)
    #   Joint 3 (link3_to_link4):         -1.5708 ~ 1.5708 (±90°)
    #   Joint 4 (link4_to_link5):         -3.1416 ~ 3.1416 (±180°)
    #   Joint 5 (link5_to_gripper_link):  0.0 ~ 1.5 (Gripper)
    
    # # 동작 시퀀스 정의 (최대 범위 테스트) - 주석 처리
    # motion_sequence_full = [
    #     ("초기 'ㄱ'자 자세 (3초 유지)", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 180),
    #     
    #     # Joint 0 테스트 (Base: -1.57 ~ 1.57)
    #     ("Joint 0: Base 최대 우회전 +90°", torch.tensor([[1.57, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 120),
    #     ("Joint 0: Base 최대 좌회전 -90°", torch.tensor([[-1.57, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 120),
    #     ("Joint 0: 원위치", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 120),
    #     
    #     # Joint 1 테스트 (Shoulder: -1.57 ~ 1.57)
    #     ("Joint 1: Shoulder 최대 앞으로 +90°", torch.tensor([[0.0, 1.57, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 120),
    #     ("Joint 1: Shoulder 최대 뒤로 -90°", torch.tensor([[0.0, -1.57, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 120),
    #     ("Joint 1: 원위치", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 120),
    #     
    #     # Joint 2 테스트 (Elbow: -1.0 ~ 2.95)
    #     ("Joint 2: Elbow 최대 펴기 +2.95", torch.tensor([[0.0, 0.0, 2.95, 0.0, 0.0, 0.0]], device=sim.device), 120),
    #     ("Joint 2: Elbow 최대 굽히기 -1.0", torch.tensor([[0.0, 0.0, -1.0, 0.0, 0.0, 0.0]], device=sim.device), 120),
    #     ("Joint 2: 원위치", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 120),
    #     
    #     # Joint 3 테스트 (Wrist1: -1.57 ~ 1.57)
    #     ("Joint 3: Wrist1 최대 좌회전 -90°", torch.tensor([[0.0, 0.0, -0.9, -1.57, 0.0, 0.0]], device=sim.device), 120),
    #     ("Joint 3: Wrist1 최대 우회전 +90°", torch.tensor([[0.0, 0.0, -0.9, 1.57, 0.0, 0.0]], device=sim.device), 120),
    #     ("Joint 3: 원위치", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 120),
    #     
    #     # Joint 4 테스트 (Wrist2: -3.14 ~ 3.14)
    #     ("Joint 4: Wrist2 최대 회전 +180°", torch.tensor([[0.0, 0.0, -0.9, 0.0, 3.14, 0.0]], device=sim.device), 120),
    #     ("Joint 4: Wrist2 최대 회전 -180°", torch.tensor([[0.0, 0.0, -0.9, 0.0, -3.14, 0.0]], device=sim.device), 120),
    #     ("Joint 4: 원위치", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 120),
    #     
    #     # Joint 5 테스트 (Gripper: 0.0 ~ 1.5)
    #     ("🤏 그리퍼: 완전 열림 (0.0)", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.0]], device=sim.device), 90),
    #     ("🤏 그리퍼: 1/3 닫힘 (0.5)", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 0.5]], device=sim.device), 90),
    #     ("🤏 그리퍼: 2/3 닫힘 (1.0)", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 1.0]], device=sim.device), 90),
    #     ("🤏 그리퍼: 완전 닫힘 (1.5)", torch.tensor([[0.0, 0.0, -0.9, 0.0, 0.0, 1.5]], device=sim.device), 90),
    # ]
    
    # print("▶️  각 링크 순차 동작 테스트 시작")
    # print(f"   총 동작 수: {len(motion_sequence)}")
    # print()
    
    try:
        for idx, (description, target_pos, hold_frames) in enumerate(motion_sequence, 1):
            print(f"🔄 [{idx}/{len(motion_sequence)}] {description}")
            
            # Set target position
            robot.set_joint_position_target(target_pos)
            robot.write_data_to_sim()  # ← 핵심! Actuator를 통해 적용
            
            # Hold for specified frames (with joint limit enforcement)
            for frame in range(hold_frames):
                # 🔥 Joint Limit 수동 강제 (매 프레임)
                actual_pos = robot.data.joint_pos.clone()
                actual_pos = torch.clamp(actual_pos, joint_limits_lower, joint_limits_upper)
                robot.write_joint_position_to_sim(actual_pos)
                
                # Actuator target 재설정 (PD 제어 강화)
                robot.set_joint_position_target(target_pos)
                robot.write_data_to_sim()
                
                sim.step()
            
            # 🔥 각 동작 후 추가 안정화 (100 프레임 = 5초)
            print(f"   ⏳ Stabilizing for 5 seconds...")
            for stab_frame in range(100):
                actual_pos = robot.data.joint_pos.clone()
                actual_pos = torch.clamp(actual_pos, joint_limits_lower, joint_limits_upper)
                robot.write_joint_position_to_sim(actual_pos)
                robot.set_joint_position_target(target_pos)
                robot.write_data_to_sim()
                sim.step()
                robot.update(dt=sim_cfg.dt)
                
                # Print status every 30 frames (0.5초)
                if frame % 30 == 0:
                    root_z = robot.data.root_pos_w[0, 2].item()
                    actual_pos_cpu = robot.data.joint_pos[0].cpu().numpy()
                    print(f"   Frame {frame:3d}/{hold_frames}: Root Z={root_z:.3f}m, "
                          f"Joint[1]={actual_pos_cpu[1]:.2f}, Joint[2]={actual_pos_cpu[2]:.2f}")
            
            # 📷 동작 완료 후 이미지 캡처
            capture_camera_image(camera, idx, description.replace(" ", "_").replace("'", ""))
            
            print(f"   ✅ 완료\n")
            
    except KeyboardInterrupt:
        print("\n\n⏹️  Test stopped by user")
    
    print("\n" + "=" * 80)
    print("✅ Joint Motion Test Complete")
    print("=" * 80)
    
    # 📷 카메라 캡처 결과 요약
    print("\n📷 Camera Capture Summary:")
    print(f"   Output Directory: {CAMERA_OUTPUT_DIR}")
    print(f"   Total Images Captured: {len(motion_sequence) * 2} (RGB + Depth)")
    print(f"   View captured images:")
    print(f"      cd {CAMERA_OUTPUT_DIR}")
    print(f"      ls -lh")
    
    # 6. Cleanup
    print("\n" + "=" * 80)
    print("STEP 7: Cleanup")
    print("=" * 80)
    print("🧹 Closing simulation...")
    
    simulation_app.close()
    
    print("✅ Simulation closed")
    print("\n" + "=" * 80)
    print("✅ Test Complete")
    print("=" * 80)


if __name__ == "__main__":
    main()
