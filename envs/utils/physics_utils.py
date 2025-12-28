#!/usr/bin/env python3
"""
Physics Utilities for Isaac Sim VRL Environments

Provides functions for querying real-time physics state:
- End effector position
- Cube position
- Grasp detection
"""

import numpy as np
from typing import Tuple, Optional


class PhysicsQueryInterface:
    """
    Interface for querying physics state from Isaac Sim
    
    Uses dynamic_control API for real-time physics positions
    (USD API does not reflect physics simulation state)
    """
    
    def __init__(self):
        self._dc = None
        self._initialized = False
    
    def initialize(self):
        """Initialize dynamic control interface"""
        try:
            from omni.isaac.dynamic_control import _dynamic_control
            self._dc = _dynamic_control.acquire_dynamic_control_interface()
            self._initialized = True
            print("✅ Dynamic Control interface acquired")
            return True
        except Exception as e:
            print(f"⚠️ Dynamic Control init failed: {e}")
            self._dc = None
            self._initialized = False
            return False
    
    def get_rigid_body_position(self, prim_path: str) -> Optional[np.ndarray]:
        """
        Get world position of a rigid body from physics simulation
        
        Args:
            prim_path: USD path to the rigid body (e.g., "/World/RoArm/gripper_link")
            
        Returns:
            position: [x, y, z] numpy array, or None if failed
        """
        if self._dc is None:
            return None
        
        try:
            rigid_body = self._dc.get_rigid_body(prim_path)
            if rigid_body:
                pose = self._dc.get_rigid_body_pose(rigid_body)
                return np.array([pose.p.x, pose.p.y, pose.p.z])
        except:
            pass
        
        return None
    
    def get_rigid_body_pose(self, prim_path: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get world position and orientation of a rigid body
        
        Args:
            prim_path: USD path to the rigid body
            
        Returns:
            position: [x, y, z] numpy array
            orientation: [qx, qy, qz, qw] quaternion numpy array
        """
        if self._dc is None:
            return None, None
        
        try:
            rigid_body = self._dc.get_rigid_body(prim_path)
            if rigid_body:
                pose = self._dc.get_rigid_body_pose(rigid_body)
                position = np.array([pose.p.x, pose.p.y, pose.p.z])
                orientation = np.array([pose.r.x, pose.r.y, pose.r.z, pose.r.w])
                return position, orientation
        except:
            pass
        
        return None, None


def get_position_usd_fallback(prim_path: str) -> Optional[np.ndarray]:
    """
    Fallback: Get position using USD API
    
    Note: This may not reflect physics simulation state!
    Use only when dynamic_control is unavailable.
    """
    try:
        from pxr import UsdGeom, Usd
        import omni.usd
        
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        
        if prim.IsValid():
            xformable = UsdGeom.Xformable(prim)
            world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            translation = world_transform.ExtractTranslation()
            return np.array([translation[0], translation[1], translation[2]])
    except:
        pass
    
    return None


def compute_distance(pos1: np.ndarray, pos2: np.ndarray) -> float:
    """Compute Euclidean distance between two positions"""
    return float(np.linalg.norm(pos1 - pos2))


def detect_cube_in_image(image: np.ndarray, color_range: Tuple[Tuple, Tuple] = None) -> Tuple[bool, float, Optional[np.ndarray]]:
    """
    Detect colored object (cube) in image
    
    Args:
        image: RGB image [H, W, 3] or [3, H, W]
        color_range: ((R_min, G_max, B_max), (R_min_bright, G_max, B_max)) for red detection
        
    Returns:
        detected: Whether cube is detected
        ratio: Percentage of pixels matching
        centroid: [cx, cy] center of detected region, or None
    """
    # Handle CHW format
    if image.shape[0] == 3:
        image = np.transpose(image, (1, 2, 0))
    
    # Default: detect red (cube color)
    if color_range is None:
        red_mask = (image[:, :, 0] > 150) & (image[:, :, 1] < 100) & (image[:, :, 2] < 100)
    else:
        (r_min, g_max, b_max) = color_range[0]
        red_mask = (image[:, :, 0] > r_min) & (image[:, :, 1] < g_max) & (image[:, :, 2] < b_max)
    
    total_pixels = image.shape[0] * image.shape[1]
    matching_pixels = np.sum(red_mask)
    ratio = matching_pixels / total_pixels
    
    detected = ratio > 0.005  # 0.5% threshold
    
    # Compute centroid
    if detected:
        y_coords, x_coords = np.where(red_mask)
        cx = np.mean(x_coords)
        cy = np.mean(y_coords)
        centroid = np.array([cx, cy])
    else:
        centroid = None
    
    return detected, ratio, centroid


# Test
if __name__ == "__main__":
    print("Testing physics utilities...")
    
    # Test image detection
    test_image = np.zeros((84, 84, 3), dtype=np.uint8)
    # Add red cube in center
    test_image[30:50, 30:50, 0] = 200  # Red
    test_image[30:50, 30:50, 1] = 50   # Low green
    test_image[30:50, 30:50, 2] = 50   # Low blue
    
    detected, ratio, centroid = detect_cube_in_image(test_image)
    print(f"Detected: {detected}, Ratio: {ratio:.4f}, Centroid: {centroid}")
    
    # Test distance
    pos1 = np.array([0.0, 0.0, 0.0])
    pos2 = np.array([1.0, 0.0, 0.0])
    print(f"Distance: {compute_distance(pos1, pos2):.3f}")
