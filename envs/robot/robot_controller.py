#!/usr/bin/env python3
"""
로봇 제어 모듈
RoArm-M3 제어 로직 (증분 제어 방식)
"""

import numpy as np
from isaacsim.core.utils.types import ArticulationAction  # ✅ Isaac Sim 5.1 API


class RobotController:
    """RoArm-M3 제어기"""
    
    def __init__(self, robot, gripper):
        """
        초기화
        
        Args:
            robot: Articulation 객체
            gripper: Gripper 객체
        """
        self.robot = robot
        self.gripper = gripper
        
        # Joint 인덱스
        self.arm_joint_indices = [0, 1, 2, 3, 4, 5]
        self.gripper_joint_indices = [6, 7]
        
        # 그리퍼 증분 제어 변수 (v3.7.7)
        self.current_gripper_pos = 0.0  # rad (0.0 = 닫힘, 0.025 = 열림)
        self.current_gripper_width = 0.03  # m (BASE_GAP)
        
        # Step count (디버깅용)
        self.step_count = 0
    
    def apply_action(self, action: np.ndarray):
        """
        액션 적용 (7차원)
        
        Args:
            action: (7,) numpy array
                [0:6] Arm joint velocities (scaled)
                [6]   Gripper scalar (-1 to 1)
        """
        # 1. Arm: Joint velocities (6차원)
        arm_velocities = action[:6] * 0.5  # scale
        
        # 2. Gripper: 증분 제어 (1차원)
        gripper_scalar = np.clip(action[6], -1.0, 1.0)
        
        # 🔥 v3.7.7 FIX: 그리퍼 증분 제어
        gripper_delta = gripper_scalar * 0.01  # ±0.01 rad/step
        self.current_gripper_pos = np.clip(
            self.current_gripper_pos + gripper_delta,
            0.0,    # 완전 닫힘
            0.025   # 완전 열림 (URDF limit)
        )
        
        # 🔥 v3.7.5 FIX: URDF 베이스 간격(3cm) 포함한 실제 폭 계산
        BASE_GAP = 0.03
        self.current_gripper_width = BASE_GAP + (self.current_gripper_pos * 2.0)
        
        # 🔥 DEBUG: 그리퍼 액션 로깅 (매 100 스텝마다)
        if self.step_count % 100 == 0:
            current_positions = self.robot.get_joint_positions()
            print(f"[DEBUG-v3.7.7] step={self.step_count}, gripper_delta={gripper_delta:.4f}, gripper_pos={self.current_gripper_pos:.4f}, tracked_width={self.current_gripper_width:.4f} (BASE={BASE_GAP}), current_gripper=[{current_positions[6]:.4f}, {current_positions[7]:.4f}]")
        
        # 목표 positions 조합
        target_positions = np.concatenate([
            arm_velocities,
            [self.current_gripper_pos, self.current_gripper_pos]
        ])
        
        # 🔥 v3.7.6: ArticulationAction으로 타깃 기반 제어
        action_obj = ArticulationAction(joint_positions=target_positions)
        self.robot.apply_action(action_obj)
        
        self.step_count += 1
    
    def reset(self, joint_positions: np.ndarray):
        """
        관절 리셋
        
        Args:
            joint_positions: (8,) numpy array (6 arm + 2 gripper)
        """
        # 그리퍼 증분 제어 변수 리셋
        self.current_gripper_pos = 0.0
        self.current_gripper_width = 0.03
        
        # Joint positions 설정
        self.robot.set_joint_positions(joint_positions)
        
        self.step_count = 0
    
    def get_gripper_width(self) -> float:
        """그리퍼 폭 반환 (m)"""
        return self.current_gripper_width
