"""
🔥 v3.5: End-Effector Pose Utility

핵심 기능:
1. EE prim 자동 탐색 (tcp, tool0, ee_link, gripper_base 등)
2. USD 월드 변환 행렬에서 위치/쿼터니언 추출
3. FK 오차 제거 - 실제 USD 포즈 사용

사용 예:
    ee_prim = find_ee_prim(stage, robot_prim_path)
    ee_pos, ee_quat = get_ee_position(stage, ee_prim)
"""

import numpy as np
from typing import Optional, Tuple
from pxr import UsdGeom, Gf


def find_ee_prim(stage, robot_prim_path: str, candidates: list = None) -> Optional[str]:
    """
    🔥 FIX #1: EE prim 자동 탐색
    
    Args:
        stage: USD stage
        robot_prim_path: 로봇 prim 경로
        candidates: 탐색할 링크 이름 후보 리스트
    
    Returns:
        EE prim 경로 (예: "/World/roarm_m3/gripper_base")
    """
    if candidates is None:
        # 일반적인 EE 링크 이름들
        candidates = [
            "gripper_base",
            "tcp",
            "tool0", 
            "ee_link",
            "end_effector",
            "link_6"
        ]
    
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim or not robot_prim.IsValid():
        print(f"  ⚠️ Robot prim not found: {robot_prim_path}")
        return None
    
    # 후보들을 순서대로 탐색
    for candidate in candidates:
        ee_path = f"{robot_prim_path}/{candidate}"
        ee_prim = stage.GetPrimAtPath(ee_path)
        
        if ee_prim and ee_prim.IsValid():
            print(f"  ✅ EE prim found: {ee_path}")
            return ee_path
    
    # 못 찾으면 첫 번째 후보를 기본값으로
    default_path = f"{robot_prim_path}/{candidates[0]}"
    print(f"  ⚠️ EE prim not found, using default: {default_path}")
    return default_path


def get_ee_position(stage, ee_prim_path: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    🔥 FIX #1: USD에서 EE 월드 포즈 추출
    
    Args:
        stage: USD stage
        ee_prim_path: EE prim 경로
    
    Returns:
        (position, quaternion) - 위치 (3,), 쿼터니언 (4,) [w,x,y,z]
        실패 시 position은 zeros, quaternion은 None
    """
    try:
        ee_prim = stage.GetPrimAtPath(ee_prim_path)
        
        if not ee_prim or not ee_prim.IsValid():
            print(f"  ⚠️ EE prim not valid: {ee_prim_path}")
            return np.zeros(3), None
        
        # UsdGeom.Xformable로 변환 행렬 추출
        xformable = UsdGeom.Xformable(ee_prim)
        world_transform = xformable.ComputeLocalToWorldTransform(0)  # time=0
        
        # 위치 추출 (4x4 행렬의 translation 부분)
        position = np.array([
            world_transform[3][0],
            world_transform[3][1],
            world_transform[3][2]
        ])
        
        # 회전 추출 (선택사항 - orientation도 필요하면)
        # 3x3 rotation matrix에서 quaternion 변환
        rotation_matrix = Gf.Matrix3d(
            world_transform[0][0], world_transform[0][1], world_transform[0][2],
            world_transform[1][0], world_transform[1][1], world_transform[1][2],
            world_transform[2][0], world_transform[2][1], world_transform[2][2]
        )
        
        rotation = Gf.Rotation(rotation_matrix)
        quat = rotation.GetQuat()
        
        # quaternion [w, x, y, z] 형식
        quaternion = np.array([
            quat.GetReal(),      # w
            quat.GetImaginary()[0],  # x
            quat.GetImaginary()[1],  # y
            quat.GetImaginary()[2]   # z
        ])
        
        return position, quaternion
        
    except Exception as e:
        print(f"  ⚠️ USD EE 추출 실패: {e}")
        return np.zeros(3), None


def get_ee_position_fallback(joint_positions: np.ndarray) -> np.ndarray:
    """
    Fallback: 간단한 FK로 EE 위치 근사
    
    Args:
        joint_positions: Joint position 배열
    
    Returns:
        EE 위치 (3,)
    """
    # 베이스에서 시작
    z_base = 0.06  # base_link height
    z_link1 = 0.08  # link_1 height
    
    # Joint 2-4는 수평 암 (link_2, link_3, link_4)
    link2_length = 0.16
    link3_length = 0.15
    
    # 간단히 X축 방향으로 투영
    x_reach = link2_length * np.cos(joint_positions[1]) + \
              link3_length * np.cos(joint_positions[1] + joint_positions[2])
    
    z_reach = z_base + z_link1 + \
              link2_length * np.sin(joint_positions[1]) + \
              link3_length * np.sin(joint_positions[1] + joint_positions[2])
    
    # Joint 1은 Z축 회전
    y_offset = x_reach * np.sin(joint_positions[0])
    x_offset = x_reach * np.cos(joint_positions[0])
    
    return np.array([x_offset, y_offset, z_reach])
