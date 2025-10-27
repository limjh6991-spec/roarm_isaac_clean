#!/usr/bin/env python3
"""URDF to USD converter for RoArm M3"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
URDF_PATH = PROJECT_ROOT / "assets" / "roarm_m3" / "urdf" / "roarm_m3_v2_complete.urdf"
USD_OUTPUT_PATH = PROJECT_ROOT / "assets" / "roarm_m3" / "usd" / "roarm_m3_v2.usd"

USD_OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)

def main():
    print("="*60)
    print("RoArm M3 URDF to USD Conversion")
    print("="*60)
    print(f"Input:  {URDF_PATH}")
    print(f"Output: {USD_OUTPUT_PATH}\n")
    
    if not URDF_PATH.exists():
        print(f"ERROR: URDF not found: {URDF_PATH}")
        sys.exit(1)
    
    print("Initializing Isaac Sim...")
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": True})
    print("OK\n")
    
    from omni.isaac.core.utils.extensions import enable_extension
    from pxr import Usd, UsdPhysics
    
    enable_extension("isaacsim.asset.importer.urdf")
    
    from isaacsim.asset.importer.urdf import _urdf
    
    config = _urdf.ImportConfig()
    config.merge_fixed_joints = False
    config.convex_decomp = False
    config.import_inertia_tensor = True
    config.fix_base = True
    config.distance_scale = 1.0
    config.density = 0.0
    config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    config.default_drive_strength = 1e4
    config.default_position_drive_damping = 1e3
    
    from omni.isaac.core.utils.stage import create_new_stage, get_current_stage
    
    create_new_stage()
    stage = get_current_stage()
    
    urdf_interface = _urdf.acquire_urdf_interface()
    
    success = urdf_interface.parse_urdf(
        str(URDF_PATH.parent),
        str(URDF_PATH.name),
        config
    )
    
    if not success:
        print("ERROR: Failed to parse URDF")
        simulation_app.close()
        sys.exit(1)
    
    print("URDF parsed OK")
    
    robot = urdf_interface.get_urdf_robot(
        str(URDF_PATH.parent),
        str(URDF_PATH.name)
    )
    
    prim_path = urdf_interface.import_robot(
        str(URDF_PATH.parent),
        str(URDF_PATH.name),
        robot,
        config,
        "/World/roarm_m3"
    )
    
    if not prim_path:
        print("ERROR: Failed to import robot")
        simulation_app.close()
        sys.exit(1)
    
    print(f"Robot imported: {prim_path}\n")
    
    stage.Export(str(USD_OUTPUT_PATH))
    print(f"USD exported: {USD_OUTPUT_PATH}")
    
    robot_prim = stage.GetPrimAtPath(prim_path)
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    links = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    
    print("="*60)
    print(f"Robot Path: {prim_path}")
    print(f"Joints:     {len(joints)}")
    print(f"Links:      {len(links)}")
    print("="*60)
    print("\nConversion complete!")
    print(f"\nNext: Open {USD_OUTPUT_PATH.name} in Isaac Sim GUI\n")
    
    simulation_app.close()

if __name__ == "__main__":
    main()
