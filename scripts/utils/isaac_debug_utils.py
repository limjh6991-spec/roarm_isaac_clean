#!/usr/bin/env python3
"""
Isaac Sim Debug Utilities

자동 스크린샷 + 구조화된 진단 출력을 제공하여 
AI 어시스턴트가 시뮬레이션 상태를 분석할 수 있게 함
"""

import os
import json
import numpy as np
from datetime import datetime
from typing import Dict, List, Optional, Tuple

# Debug output directory
DEBUG_DIR = "/tmp/isaac_debug"


def ensure_debug_dir():
    """Create debug directory if not exists"""
    os.makedirs(DEBUG_DIR, exist_ok=True)
    return DEBUG_DIR


def save_camera_image(rgb_data: np.ndarray, name: str = "camera") -> str:
    """
    Save camera RGB image to debug directory
    
    Args:
        rgb_data: RGB image array (H, W, 3)
        name: Image name prefix
        
    Returns:
        Path to saved image
    """
    ensure_debug_dir()
    timestamp = datetime.now().strftime("%H%M%S")
    filepath = os.path.join(DEBUG_DIR, f"{name}_{timestamp}.png")
    
    try:
        from PIL import Image
        
        # Ensure proper format
        if rgb_data.dtype != np.uint8:
            if rgb_data.max() <= 1.0:
                rgb_data = (rgb_data * 255).astype(np.uint8)
            else:
                rgb_data = rgb_data.astype(np.uint8)
        
        img = Image.fromarray(rgb_data[:, :, :3])
        img.save(filepath)
        print(f"📸 Saved: {filepath}")
        return filepath
    except Exception as e:
        print(f"⚠️ Failed to save image: {e}")
        return ""


def get_prim_world_transform(stage, prim_path: str) -> Optional[Dict]:
    """
    Get world transform of a prim
    
    Returns:
        Dict with position, rotation, forward_vector
    """
    try:
        from pxr import UsdGeom, Gf
        
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return None
        
        xformable = UsdGeom.Xformable(prim)
        world_transform = xformable.ComputeLocalToWorldTransform(0)
        
        # Extract position
        pos = world_transform.ExtractTranslation()
        
        # Extract rotation matrix
        rot_matrix = world_transform.ExtractRotationMatrix()
        
        # Get forward vector (assuming -Z is forward for cameras, +X for robots)
        forward_x = Gf.Vec3d(rot_matrix[0][0], rot_matrix[1][0], rot_matrix[2][0])
        forward_z = Gf.Vec3d(-rot_matrix[0][2], -rot_matrix[1][2], -rot_matrix[2][2])
        
        return {
            "path": prim_path,
            "position": [float(pos[0]), float(pos[1]), float(pos[2])],
            "forward_x": [float(forward_x[0]), float(forward_x[1]), float(forward_x[2])],
            "forward_neg_z": [float(forward_z[0]), float(forward_z[1]), float(forward_z[2])],
        }
    except Exception as e:
        return {"path": prim_path, "error": str(e)}


def diagnose_robot_camera_setup(stage, robot_path: str = "/World/Robot", 
                                 camera_path: str = None,
                                 cube_path: str = "/World/TargetCube") -> Dict:
    """
    Comprehensive diagnosis of robot and camera setup
    
    Returns:
        Dict with all diagnostic information
    """
    result = {
        "timestamp": datetime.now().isoformat(),
        "robot": {},
        "camera": {},
        "cube": {},
        "analysis": {}
    }
    
    try:
        from pxr import UsdGeom, Gf
        
        # Robot base
        robot_transform = get_prim_world_transform(stage, robot_path)
        if robot_transform:
            result["robot"]["base"] = robot_transform
        
        # Gripper link
        gripper_path = f"{robot_path}/gripper_link"
        gripper_transform = get_prim_world_transform(stage, gripper_path)
        if gripper_transform:
            result["robot"]["gripper"] = gripper_transform
        
        # Camera
        if camera_path:
            cam_transform = get_prim_world_transform(stage, camera_path)
            if cam_transform:
                result["camera"] = cam_transform
        else:
            # Try to find camera under gripper
            for suffix in ["/HandEyeCamera", "/TestCam", "/Camera"]:
                cam_transform = get_prim_world_transform(stage, gripper_path + suffix)
                if cam_transform and "error" not in cam_transform:
                    result["camera"] = cam_transform
                    break
        
        # Cube
        cube_transform = get_prim_world_transform(stage, cube_path)
        if cube_transform:
            result["cube"] = cube_transform
        
        # Analysis
        if "position" in result.get("camera", {}) and "position" in result.get("cube", {}):
            cam_pos = np.array(result["camera"]["position"])
            cube_pos = np.array(result["cube"]["position"])
            
            # Distance
            distance = np.linalg.norm(cube_pos - cam_pos)
            result["analysis"]["camera_to_cube_distance"] = float(distance)
            
            # Direction to cube
            direction = cube_pos - cam_pos
            direction = direction / np.linalg.norm(direction)
            result["analysis"]["direction_to_cube"] = direction.tolist()
            
            # Camera forward direction (assuming -Z is forward for USD cameras)
            if "forward_neg_z" in result["camera"]:
                cam_forward = np.array(result["camera"]["forward_neg_z"])
                
                # Dot product to check alignment
                alignment = np.dot(cam_forward, direction)
                result["analysis"]["camera_cube_alignment"] = float(alignment)
                result["analysis"]["camera_facing_cube"] = alignment > 0.5
                result["analysis"]["alignment_note"] = (
                    "Camera facing cube ✓" if alignment > 0.5 
                    else f"Camera NOT facing cube (alignment: {alignment:.2f})"
                )
    
    except Exception as e:
        result["error"] = str(e)
    
    return result


def save_diagnosis_report(report: Dict, name: str = "diagnosis") -> str:
    """Save diagnosis report to JSON file"""
    ensure_debug_dir()
    timestamp = datetime.now().strftime("%H%M%S")
    filepath = os.path.join(DEBUG_DIR, f"{name}_{timestamp}.json")
    
    with open(filepath, 'w') as f:
        json.dump(report, f, indent=2)
    
    print(f"📋 Diagnosis saved: {filepath}")
    return filepath


def print_diagnosis_summary(report: Dict):
    """Print human-readable diagnosis summary"""
    print("\n" + "=" * 60)
    print("🔍 DIAGNOSIS SUMMARY")
    print("=" * 60)
    
    if "robot" in report and "gripper" in report["robot"]:
        g = report["robot"]["gripper"]
        print(f"\n📍 Gripper Position: ({g['position'][0]:.3f}, {g['position'][1]:.3f}, {g['position'][2]:.3f})")
        print(f"   Forward (+X): ({g['forward_x'][0]:.3f}, {g['forward_x'][1]:.3f}, {g['forward_x'][2]:.3f})")
    
    if "camera" in report and "position" in report["camera"]:
        c = report["camera"]
        print(f"\n📷 Camera Position: ({c['position'][0]:.3f}, {c['position'][1]:.3f}, {c['position'][2]:.3f})")
        print(f"   Looking (-Z): ({c['forward_neg_z'][0]:.3f}, {c['forward_neg_z'][1]:.3f}, {c['forward_neg_z'][2]:.3f})")
    
    if "cube" in report and "position" in report["cube"]:
        cb = report["cube"]
        print(f"\n🎯 Cube Position: ({cb['position'][0]:.3f}, {cb['position'][1]:.3f}, {cb['position'][2]:.3f})")
    
    if "analysis" in report:
        a = report["analysis"]
        print(f"\n📊 Analysis:")
        if "camera_to_cube_distance" in a:
            print(f"   Distance to cube: {a['camera_to_cube_distance']:.3f} m")
        if "alignment_note" in a:
            print(f"   {a['alignment_note']}")
    
    print("=" * 60)


def run_full_diagnosis(world, camera_annotator=None, 
                       cube_path: str = "/World/TargetCube") -> Tuple[Dict, str]:
    """
    Run complete diagnosis and save all outputs
    
    Returns:
        Tuple of (diagnosis_report, image_path)
    """
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    
    # Get diagnosis
    report = diagnose_robot_camera_setup(stage, cube_path=cube_path)
    
    # Save image if annotator available
    image_path = ""
    if camera_annotator is not None:
        try:
            rgb_data = camera_annotator.get_data()
            if rgb_data is not None:
                image_path = save_camera_image(rgb_data[:, :, :3], "camera_view")
                report["image_saved"] = image_path
        except:
            pass
    
    # Print and save
    print_diagnosis_summary(report)
    report_path = save_diagnosis_report(report)
    report["report_path"] = report_path
    
    return report, image_path
