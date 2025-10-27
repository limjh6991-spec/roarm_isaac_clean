"""
🔥 v3.5: Gripper Controller Module

핵심 기능:
1. Prismatic joint 자동 탐색 및 LINEAR drive 설정
2. 그리퍼 폭 측정 (measure_width)
3. 목표 간격 명령 (command_gap)
4. 그립 상태 판정 (is_grasped) - 큐브 폭 기반
5. FixedJoint attach/detach 관리

사용 예:
    gripper = Gripper(stage, robot_prim_path, 
                     finger_joint_names=["gripper_left_joint", "gripper_right_joint"])
    gripper.setup_drives(stiffness=8000, damping=800, max_force=50)
    
    width = gripper.measure_width()
    is_gripping = gripper.is_grasped(ee_pos, cube_pos, width, cube_size=0.04)
    
    if is_gripping:
        gripper.attach(stage, gripper_base_path, cube_path)
    else:
        gripper.detach(stage)
"""

import numpy as np
from typing import List, Optional, Tuple
from pxr import UsdPhysics, Sdf


class Gripper:
    """Gripper controller with LINEAR drive and FixedJoint attach"""
    
    def __init__(self, 
                 stage,
                 robot_prim_path: str,
                 finger_joint_names: List[str] = None):
        """
        Args:
            stage: USD stage
            robot_prim_path: 로봇 prim 경로 (예: "/World/roarm_m3")
            finger_joint_names: 그리퍼 joint 이름 리스트 (None이면 자동 탐색)
        """
        self.stage = stage
        self.robot_prim_path = robot_prim_path
        
        # Joint 탐색
        if finger_joint_names is None:
            self.finger_joint_names = self._find_gripper_joints()
        else:
            self.finger_joint_names = finger_joint_names
        
        print(f"  ✅ Gripper joints: {self.finger_joint_names}")
        
        # Attach 상태
        self.is_attached = False
        self.grasp_joint_path = None
        self.detach_called_this_step = False  # 🔥 v3.9.1: Detach 플래그
    
    def _find_gripper_joints(self) -> List[str]:
        """Prismatic joint 자동 탐색"""
        gripper_joints = []
        
        # robot prim의 모든 자식 탐색
        robot_prim = self.stage.GetPrimAtPath(self.robot_prim_path)
        if not robot_prim or not robot_prim.IsValid():
            print(f"  ⚠️ Robot prim not found: {self.robot_prim_path}")
            return []
        
        for prim in robot_prim.GetAllChildren():
            # "gripper" 키워드 포함하는 joint 찾기
            if "gripper" in prim.GetName().lower() and "joint" in prim.GetName().lower():
                # Prismatic joint인지 확인
                if prim.GetTypeName() == "PhysicsPrismaticJoint":
                    gripper_joints.append(prim.GetName())
        
        if not gripper_joints:
            # Fallback: 일반적인 이름으로 시도
            gripper_joints = ["gripper_left_joint", "gripper_right_joint"]
            print(f"  ⚠️ Prismatic joints not found, using default: {gripper_joints}")
        
        return gripper_joints
    
    def setup_drives(self, 
                     stiffness: float = 8000.0,
                     damping: float = 800.0,
                     max_force: float = 50.0):
        """
        🔥 FIX #3: Prismatic joint에 LINEAR drive 적용
        
        Args:
            stiffness: 강성 (높을수록 정확한 위치 제어)
            damping: 감쇠 (안정성)
            max_force: 최대 힘 (N) - 충분한 힘으로 죄기
        """
        for joint_name in self.finger_joint_names:
            joint_prim = self.stage.GetPrimAtPath(f"{self.robot_prim_path}/{joint_name}")
            
            if joint_prim and joint_prim.IsValid():
                # 🔥 LINEAR drive 적용 (Prismatic joint)
                drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
                drive_api.GetStiffnessAttr().Set(stiffness)
                drive_api.GetDampingAttr().Set(damping)
                drive_api.GetMaxForceAttr().Set(max_force)
                
                print(f"    ✅ {joint_name}: LINEAR drive (stiff={stiffness}, damp={damping}, force={max_force}N)")
            else:
                print(f"    ⚠️ Joint prim not found: {joint_name}")
    
    def measure_width(self, joint_positions: np.ndarray, joint_indices: Tuple[int, int] = (6, 7)) -> float:
        """
        그리퍼 폭 측정
        
        Args:
            joint_positions: 전체 joint position 배열
            joint_indices: 그리퍼 joint 인덱스 (left, right)
        
        Returns:
            그리퍼 폭 (m) - 양쪽 finger 간격
        """
        # 🔥 DEBUG: 입력 확인
        if len(joint_positions) < 8:
            print(f"[WARN] joint_positions too short: len={len(joint_positions)}")
            return 0.0
        
        left_pos = abs(joint_positions[joint_indices[0]])
        right_pos = abs(joint_positions[joint_indices[1]])
        width = left_pos + right_pos
        
        # 🔥 DEBUG: 계산 과정 로깅 (매 100번째)
        import random
        if random.random() < 0.01:  # 1% 확률로 로깅
            print(f"[DEBUG-MEASURE] left={left_pos:.4f}, right={right_pos:.4f}, width={width:.4f}")
        
        return width
    
    def command_gap(self, target_gap: float, current_positions: np.ndarray) -> np.ndarray:
        """
        목표 간격으로 그리퍼 명령
        
        Args:
            target_gap: 목표 간격 (m)
            current_positions: 현재 joint positions
        
        Returns:
            업데이트된 joint positions
        """
        # 양쪽 finger가 대칭으로 움직인다고 가정
        target_per_finger = target_gap / 2.0
        
        new_positions = current_positions.copy()
        new_positions[6] = target_per_finger  # left finger
        new_positions[7] = target_per_finger  # right finger
        
        return new_positions
    
    def is_grasped(self,
                   ee_pos: np.ndarray,
                   cube_pos: np.ndarray,
                   gripper_width: float,
                   cube_size: float = 0.04,
                   dist_tol: float = 0.03,
                   z_tol: float = 0.01,
                   width_margin: float = 0.006) -> bool:
        """
        🔥 FIX #2: 그립 상태 판정 - "그리퍼 폭 ≈ 큐브 폭" 방식
        
        Args:
            ee_pos: End-effector 위치 (m)
            cube_pos: 큐브 위치 (m)
            gripper_width: 현재 그리퍼 폭 (m)
            cube_size: 큐브 크기 (m)
            dist_tol: 거리 허용 오차 (m)
            z_tol: Z축 정렬 허용 오차 (m)
            width_margin: 그리퍼 폭 여유 (m)
        
        Returns:
            True if 큐브를 잡고 있음
        """
        # 1. 거리 체크
        ee_to_cube_dist = np.linalg.norm(cube_pos - ee_pos)
        if ee_to_cube_dist > dist_tol:
            return False
        
        # 2. Z축 정렬 체크
        z_alignment = abs(cube_pos[2] - ee_pos[2])
        if z_alignment > z_tol:
            return False
        
        # 3. 그리퍼 폭 체크: 큐브를 끼운 상태
        # 정상 그립: cube_size - margin < width < cube_size + margin
        is_grasping_cube = (
            (cube_size - width_margin) < gripper_width < (cube_size + width_margin)
        )
        
        return is_grasping_cube
    
    def attach(self, 
               stage,
               gripper_base_path: str,
               cube_path: str,
               timestep: int = 0):
        """
        🔥 FIX #5: FixedJoint로 큐브 부착
        
        Args:
            stage: USD stage
            gripper_base_path: gripper_base prim 경로
            cube_path: 큐브 prim 경로
            timestep: 현재 timestep (unique joint 경로 생성용)
        """
        if self.is_attached:
            return  # 이미 부착됨
        
        try:
            # FixedJoint 생성 경로
            self.grasp_joint_path = f"/World/GraspJoint_{timestep}"
            
            # FixedJoint prim 생성
            joint_prim = stage.DefinePrim(self.grasp_joint_path, "PhysicsFixedJoint")
            fixed_joint = UsdPhysics.FixedJoint(joint_prim)
            
            # Body0: gripper_base
            fixed_joint.CreateBody0Rel().SetTargets([gripper_base_path])
            
            # Body1: cube
            fixed_joint.CreateBody1Rel().SetTargets([cube_path])
            
            # Joint position (gripper_base 기준)
            fixed_joint.CreateLocalPos0Attr().Set((0.0, 0.0, 0.04))
            fixed_joint.CreateLocalRot0Attr().Set((1.0, 0.0, 0.0, 0.0))
            
            fixed_joint.CreateLocalPos1Attr().Set((0.0, 0.0, 0.0))
            fixed_joint.CreateLocalRot1Attr().Set((1.0, 0.0, 0.0, 0.0))
            
            # Joint 활성화
            fixed_joint.CreateJointEnabledAttr().Set(True)
            
            self.is_attached = True
            print(f"    🔗 FixedJoint 생성: 큐브 부착! ({self.grasp_joint_path})")
            
        except Exception as e:
            print(f"    ⚠️ FixedJoint 생성 실패: {e}")
    
    def detach(self, stage):
        """
        🔥 FIX #5: FixedJoint 제거하여 큐브 분리
        
        Args:
            stage: USD stage
        """
        if not self.is_attached:
            return  # 이미 분리됨
        
        try:
            if self.grasp_joint_path:
                joint_prim = stage.GetPrimAtPath(self.grasp_joint_path)
                if joint_prim and joint_prim.IsValid():
                    stage.RemovePrim(self.grasp_joint_path)
                    print(f"    🔓 FixedJoint 제거: 큐브 분리! ({self.grasp_joint_path})")
            
            self.is_attached = False
            self.grasp_joint_path = None
            
        except Exception as e:
            print(f"    ⚠️ FixedJoint 제거 실패: {e}")
    
    def reset(self):
        """리셋 시 attach 상태 초기화"""
        self.is_attached = False
        self.grasp_joint_path = None
