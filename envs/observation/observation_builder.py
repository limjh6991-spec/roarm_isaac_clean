#!/usr/bin/env python3
"""
관측 생성 모듈
RoArm-M3 Pick & Place 환경의 관측 벡터 생성
"""

import numpy as np
from typing import Tuple


class ObservationBuilder:
    """관측 벡터 생성기 (28차원)"""
    
    def __init__(self, robot, gripper, cube, target, ee_prim):
        """
        초기화
        
        Args:
            robot: Articulation 객체
            gripper: Gripper 객체
            cube: DynamicCuboid 객체
            target: VisualCuboid 객체
            ee_prim: End-Effector Prim
        """
        self.robot = robot
        self.gripper = gripper
        self.cube = cube
        self.target = target
        self.ee_prim = ee_prim
    
    def build_observation(self) -> np.ndarray:
        """
        28차원 관측 생성
        
        Returns:
            obs: (28,) numpy array
                [0:3]   EE position (world frame)
                [3:6]   EE velocity
                [6:9]   Cube position relative to EE
                [9:13]  Cube rotation (quaternion)
                [13:16] Target position relative to EE
                [16:19] Cube to target vector
                [19:22] Joint positions (3개: joint_2, 3, 4)
                [22:23] Gripper width
                [23:24] Is grasped (0 or 1)
                [24:28] Reserved (0s)
        """
        # 1. EE 위치 (world frame)
        ee_pos = self._get_ee_position()
        
        # 2. EE 속도
        ee_vel = self._get_ee_velocity()
        
        # 3. Cube 위치 (EE 기준 상대 좌표)
        cube_pos = self._get_cube_position()
        cube_rel = cube_pos - ee_pos
        
        # 4. Cube 회전 (quaternion)
        cube_rot = self._get_cube_rotation()
        
        # 5. Target 위치 (EE 기준 상대 좌표)
        target_pos = self._get_target_position()
        target_rel = target_pos - ee_pos
        
        # 6. Cube → Target 벡터
        cube_to_target = target_pos - cube_pos
        
        # 7. Joint positions (3개만: joint_2, 3, 4)
        joint_positions = self.robot.get_joint_positions()
        selected_joints = joint_positions[2:5]  # indices 2, 3, 4
        
        # 8. Gripper width
        gripper_width = self.gripper.get_width()
        
        # 9. Is grasped
        is_grasped = float(self.gripper.is_attached())
        
        # 10. Reserved (4차원)
        reserved = np.zeros(4)
        
        # 관측 벡터 조합 (28차원)
        obs = np.concatenate([
            ee_pos,           # 3
            ee_vel,           # 3
            cube_rel,         # 3
            cube_rot,         # 4
            target_rel,       # 3
            cube_to_target,   # 3
            selected_joints,  # 3
            [gripper_width],  # 1
            [is_grasped],     # 1
            reserved          # 4
        ])
        
        return obs.astype(np.float32)
    
    def _get_ee_position(self) -> np.ndarray:
        """EE 위치 (world frame)"""
        from pxr import UsdGeom, Gf
        
        xformable = UsdGeom.Xformable(self.ee_prim)
        world_transform: Gf.Matrix4d = xformable.ComputeLocalToWorldTransform(0)
        translation: Gf.Vec3d = world_transform.ExtractTranslation()
        
        return np.array([translation[0], translation[1], translation[2]], dtype=np.float32)
    
    def _get_ee_velocity(self) -> np.ndarray:
        """EE 속도 (근사치)"""
        # Articulation의 root velocity 사용
        velocities = self.robot.get_joint_velocities()
        
        # 간단한 근사: joint velocities의 평균
        # 실제로는 Jacobian 기반 계산이 필요하지만, 여기서는 근사
        if len(velocities) >= 6:
            arm_vels = velocities[:6]
            ee_vel = arm_vels[:3]  # 첫 3개 joint의 속도 사용
        else:
            ee_vel = np.zeros(3)
        
        return ee_vel.astype(np.float32)
    
    def _get_cube_position(self) -> np.ndarray:
        """큐브 위치 (world frame)"""
        pos, _ = self.cube.get_world_pose()
        return np.array(pos, dtype=np.float32)
    
    def _get_cube_rotation(self) -> np.ndarray:
        """큐브 회전 (quaternion [x, y, z, w])"""
        _, rot = self.cube.get_world_pose()
        return np.array(rot, dtype=np.float32)
    
    def _get_target_position(self) -> np.ndarray:
        """타겟 위치 (world frame)"""
        pos, _ = self.target.get_world_pose()
        return np.array(pos, dtype=np.float32)
