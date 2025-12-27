#!/usr/bin/env python3
"""
URDF to USD Converter for Isaac Sim
Enhanced RoArm-M3 with D455 Camera
"""

import sys
from pathlib import Path

# Add Isaac Sim Python path
ISAAC_SIM_PATH = "/home/roarm_m3/.local/share/ov/pkg/isaac-sim-4.2.0"
sys.path.insert(0, f"{ISAAC_SIM_PATH}/exts/omni.isaac.kit/omni/isaac/kit")

from isaacsim import SimulationApp

# Launch Isaac Sim (headless mode for conversion)
simulation_app = SimulationApp({"headless": False, "width": 1280, "height": 720})

import omni
from isaacsim.core.api import World
from isaacsim.asset.importer.urdf import _urdf  # ✅ Isaac Sim 5.1 API
import carb

# Paths
URDF_PATH = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_enhanced.urdf"
USD_OUTPUT_DIR = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/usd"
USD_OUTPUT_NAME = "roarm_m3_enhanced.usd"

def convert_urdf_to_usd():
    """Convert Enhanced URDF to USD format for Isaac Sim."""
    
    print("\n" + "="*60)
    print("  URDF to USD Converter")
    print("  Enhanced RoArm-M3 with D455 Camera")
    print("="*60)
    
    # Verify URDF file exists
    urdf_path = Path(URDF_PATH)
    if not urdf_path.exists():
        print(f"❌ Error: URDF file not found: {URDF_PATH}")
        return False
    
    print(f"\n📦 URDF File: {URDF_PATH}")
    print(f"   Size: {urdf_path.stat().st_size / 1024:.2f} KB")
    
    # Create USD output directory
    usd_output_dir = Path(USD_OUTPUT_DIR)
    usd_output_dir.mkdir(parents=True, exist_ok=True)
    
    usd_output_path = usd_output_dir / USD_OUTPUT_NAME
    print(f"\n📝 USD Output: {usd_output_path}")
    
    # Initialize World
    print("\n🌍 Initializing Isaac Sim World...")
    world = World(stage_units_in_meters=1.0)
    
    # URDF Import Configuration
    print("\n⚙️  Configuring URDF Import...")
    import_config = _urdf.ImportConfig()
    
    # Essential settings
    import_config.merge_fixed_joints = False  # Keep camera frames
    import_config.import_inertia_tensor = True  # Import inertial properties
    import_config.fix_base = False  # Allow base to move
    import_config.self_collision = True  # Enable self-collision
    
    # Scaling and positioning
    import_config.default_drive_strength = 1e7
    import_config.default_position_drive_damping = 1e5
    import_config.distance_scale = 1.0  # Meters
    
    print("   Configuration:")
    print(f"     - merge_fixed_joints: {import_config.merge_fixed_joints}")
    print(f"     - import_inertia_tensor: {import_config.import_inertia_tensor}")
    print(f"     - fix_base: {import_config.fix_base}")
    print(f"     - self_collision: {import_config.self_collision}")
    
    # Import URDF
    print("\n🚀 Importing URDF...")
    try:
        success, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(URDF_PATH),
            import_config=import_config,
        )
        
        if not success:
            print("❌ URDF import failed!")
            return False
        
        print(f"✅ URDF imported successfully!")
        print(f"   Prim path: {prim_path}")
        
    except Exception as e:
        print(f"❌ Error during URDF import: {e}")
        return False
    
    # Verify imported structure
    print("\n🔍 Verifying imported structure...")
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    
    if not prim.IsValid():
        print(f"❌ Error: Prim not found at {prim_path}")
        return False
    
    # Count children (links)
    children = prim.GetChildren()
    print(f"   Links imported: {len(children)}")
    
    # List first few links
    print("\n   📋 Link structure:")
    for i, child in enumerate(children[:5]):
        print(f"      {i+1}. {child.GetName()}")
    if len(children) > 5:
        print(f"      ... and {len(children) - 5} more")
    
    # Save as USD
    print(f"\n💾 Saving USD file...")
    try:
        omni.usd.get_context().save_as_stage(str(usd_output_path))
        print(f"✅ USD file saved: {usd_output_path}")
        print(f"   Size: {usd_output_path.stat().st_size / 1024:.2f} KB")
        
    except Exception as e:
        print(f"❌ Error saving USD: {e}")
        return False
    
    print("\n" + "="*60)
    print("✅ Conversion completed successfully!")
    print("="*60)
    print("\n📚 Next Steps:")
    print("   1. Load USD in Isaac Sim: File → Open → " + str(usd_output_path))
    print("   2. Add Camera sensor to 'camera_link'")
    print("   3. Test Gripper mimic joint behavior")
    print("   4. Verify Inertial properties in Physics Inspector")
    print("\n")
    
    return True

if __name__ == "__main__":
    try:
        success = convert_urdf_to_usd()
        
        # Keep Isaac Sim open to view the result
        if success:
            print("💡 Isaac Sim is now running. Close the window to exit.")
            print("   Press Ctrl+C in terminal to force quit.\n")
            
            # Run simulation for viewing
            world = World.instance()
            world.reset()
            
            # Keep running
            while simulation_app.is_running():
                world.step(render=True)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n🔚 Shutting down Isaac Sim...")
        simulation_app.close()
