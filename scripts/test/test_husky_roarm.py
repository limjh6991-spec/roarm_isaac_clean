#!/usr/bin/env python3
"""
Husky + RoArm-M3 Mobile Manipulator Test
야외/농업용 모바일 매니퓰레이터 통합 테스트
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({'headless': False})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
import omni.usd
from pxr import UsdLux, UsdGeom, Gf, UsdPhysics
import numpy as np

print("\n🚜 Husky + RoArm-M3 Mobile Manipulator Test")

# Initialize world with smaller timestep for stability
world = World(stage_units_in_meters=1.0, physics_dt=1.0/120.0)
stage = omni.usd.get_context().get_stage()
world.scene.add_default_ground_plane()

# Add lighting
light = UsdLux.DomeLight(stage.DefinePrim('/World/DomeLight', 'DomeLight'))
light.GetIntensityAttr().Set(1500.0)

world.reset()

# Load complete mobile manipulator (10 DOF)
print("📦 Loading Complete Mobile Manipulator (10 DOF)...")
add_reference_to_stage(
    usd_path='/home/roarm_m3/roarm_isaac_clean/assets/mobile_manipulator/husky_roarm_complete.usd',
    prim_path='/World/Robot'
)

world.reset()

robot = None
try:
    robot = SingleArticulation(prim_path='/World/Robot', name='robot')
    world.scene.add(robot)
    world.reset()
    print(f"✅ Robot loaded! DOF: {robot.num_dof}")
    if robot.dof_names:
        print(f"   Joints: {robot.dof_names}")
except Exception as e:
    print(f"⚠️ Articulation: {e}")

# === Set initial arm posture (horizontal) BEFORE physics starts ===
target_positions = None
if robot is not None and robot.num_dof >= 10:
    print("\n🦾 Setting initial arm posture (horizontal)...")
    
    # Initial positions for horizontal arm posture
    target_positions = np.zeros(robot.num_dof)
    
    # Wheels at 0
    target_positions[0:4] = 0.0
    
    # Arm joints for horizontal posture (base_link, link1 stay at 0)
    target_positions[4] = 0.0      # arm_joint1 (base rotation): keep 0
    target_positions[5] = 0.0      # arm_joint2 (shoulder): 0 for horizontal
    target_positions[6] = 1.0      # arm_joint3 (elbow): level the forearm
    target_positions[7] = 0.5      # arm_joint4 (wrist pitch): compensate
    target_positions[8] = 0.0      # arm_joint5 (wrist roll): 0
    target_positions[9] = 0.5      # gripper: half open
    
    # Set positions multiple times to ensure they stick
    for _ in range(5):
        robot.set_joint_positions(target_positions)
        robot.set_joint_velocities(np.zeros(robot.num_dof))
        world.step(render=False)
    
    print(f"✅ Initial posture set: {target_positions[4:10]}")

# Stabilize before rendering - more steps and force positions
print("⏳ Stabilizing physics (100 steps)...")
for i in range(100):
    if robot is not None and target_positions is not None:
        robot.set_joint_positions(target_positions)
        robot.set_joint_velocities(np.zeros(robot.num_dof))
    world.step(render=False)

print("✅ Physics stabilized")

# === Main Loop with PD Control ===
print("\n🎮 Running simulation...")
print("   - Arm is under PD position control")
print("   - 2000 steps")
print("")

prev_error = np.zeros(robot.num_dof) if robot else None
step = 0

try:
    while step < 2000:
        if robot is not None and target_positions is not None:
            # PD Control for arm joints
            current_pos = robot.get_joint_positions()
            current_vel = robot.get_joint_velocities()
            
            if current_pos is not None:
                # PD gains
                kp = 20.0   # Position gain (lower for stability)
                kd = 2.0    # Derivative gain
                
                error = target_positions - current_pos
                
                # Apply control to arm joints only (indices 4-9)
                efforts = np.zeros(robot.num_dof)
                efforts[4:10] = kp * error[4:10] - kd * current_vel[4:10]
                
                # Clamp efforts
                efforts = np.clip(efforts, -5.0, 5.0)
                robot.set_joint_efforts(efforts)
        
        world.step(render=True)
        step += 1
        
except KeyboardInterrupt:
    print("\n👋 Simulation stopped")

print("\n📊 Final Summary:")
if robot:
    print(f"   Total DOF: {robot.num_dof}")
    final_pos = robot.get_joint_positions()
    if final_pos is not None:
        print(f"   Final arm positions: {final_pos[4:10]}")

simulation_app.close()
