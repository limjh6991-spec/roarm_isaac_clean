"""
🔥 v3.5: Pick-and-Place Reward & Gating Module

핵심 기능:
1. 단계별 보상 계산 (REACH → GRIP → LIFT → PLACE)
2. 게이팅 기준 한곳에서 관리 (GateConfig)
3. 큐브 폭 기반 그립 판정

사용 예:
    config = GateConfig(cube_size=0.04)
    
    gate_ok = grasp_gate(obs, config)
    r_grasp = compute_grasp_reward(obs, gate_ok)
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass
class GateConfig:
    """게이팅 기준 상수"""
    # GRIP 조건 (🔥 v3.5)
    cube_size: float = 0.04          # 큐브 크기 (4cm)
    grip_dist_tol: float = 0.03      # EE-큐브 거리 허용 (3cm)
    grip_z_tol: float = 0.01         # Z축 정렬 허용 (1cm)
    grip_width_margin: float = 0.005 # 그리퍼 폭 여유 (0.5cm)
    
    # REACH 조건
    reach_dist_threshold: float = 0.10  # 10cm 이내면 REACH
    
    # LIFT 조건
    lift_height_threshold: float = 0.05  # 5cm 들어올리기
    
    # PLACE 조건
    place_dist_threshold: float = 0.08   # 타겟 8cm 이내
    
    # SUCCESS 조건
    success_threshold: float = 0.02      # 타겟 2cm 이내


def grasp_gate(obs: np.ndarray, config: GateConfig) -> bool:
    """
    🔥 v3.5 FIX #2: 그립 게이트 - "그리퍼 폭 ≈ 큐브 폭" 방식
    
    Args:
        obs: Observation (28 dim)
            - obs[8:11]: cube_relative_to_ee
            - obs[23]: gripper_width
            - obs[25]: dist_to_cube
        config: 게이팅 기준
    
    Returns:
        True if 유효한 그립 상태
    """
    cube_relative_to_ee = obs[8:11]
    gripper_width = obs[23]
    dist_to_cube = obs[25]
    
    # 1. 거리 체크
    if dist_to_cube > config.grip_dist_tol:
        return False
    
    # 2. Z축 정렬 체크
    z_alignment = abs(cube_relative_to_ee[2])
    if z_alignment > config.grip_z_tol:
        return False
    
    # 3. 그리퍼 폭 체크: 큐브를 끼운 상태
    is_grasping_cube = (
        (config.cube_size - config.grip_width_margin) < gripper_width <
        (config.cube_size + config.grip_width_margin)
    )
    
    return is_grasping_cube


def compute_reach_reward(obs: np.ndarray, 
                         prev_dist: float = None,
                         config: GateConfig = None) -> Tuple[float, Dict]:
    """
    REACH 단계 보상: EE가 큐브에 접근
    
    Args:
        obs: Observation
        prev_dist: 이전 EE-큐브 거리
        config: 게이팅 기준
    
    Returns:
        (reward, info_dict)
    """
    if config is None:
        config = GateConfig()
    
    dist_to_cube = obs[25]
    reward = 0.0
    info = {}
    
    # Dense reward: 거리 기반 (음수)
    reward -= dist_to_cube * 3.0
    
    # 진전 보너스
    if prev_dist is not None and dist_to_cube < prev_dist:
        improvement = prev_dist - dist_to_cube
        reward += improvement * 5.0
        info["improvement"] = improvement
    
    # Milestone: REACH 달성
    if dist_to_cube < config.reach_dist_threshold:
        info["reached"] = True
    
    return reward, info


def compute_align_reward(obs: np.ndarray, config: GateConfig = None) -> Tuple[float, Dict]:
    """
    ALIGN 단계 보상: Z축 정렬
    
    Args:
        obs: Observation
        config: 게이팅 기준
    
    Returns:
        (reward, info_dict)
    """
    if config is None:
        config = GateConfig()
    
    cube_relative_to_ee = obs[8:11]
    z_alignment = abs(cube_relative_to_ee[2])
    
    reward = 0.0
    info = {}
    
    # Z축 정렬 보상 (낮을수록 좋음)
    reward -= z_alignment * 10.0
    
    # 정렬 성공
    if z_alignment < config.grip_z_tol:
        info["aligned"] = True
    
    return reward, info


def compute_grasp_reward(obs: np.ndarray, 
                        grasp_valid: bool,
                        config: GateConfig = None) -> Tuple[float, Dict]:
    """
    GRASP 단계 보상: 그리퍼로 큐브 잡기
    
    Args:
        obs: Observation
        grasp_valid: grasp_gate 결과
        config: 게이팅 기준
    
    Returns:
        (reward, info_dict)
    """
    if config is None:
        config = GateConfig()
    
    gripper_width = obs[23]
    reward = 0.0
    info = {}
    
    # 그리퍼가 목표 폭에 가까울수록 보상
    target_width = config.cube_size
    width_error = abs(gripper_width - target_width)
    reward -= width_error * 20.0
    
    # 유효한 그립
    if grasp_valid:
        info["grasped"] = True
    
    return reward, info


def compute_lift_reward(obs: np.ndarray, 
                       cube_initial_z: float,
                       config: GateConfig = None) -> Tuple[float, Dict]:
    """
    LIFT 단계 보상: 큐브 들어올리기
    
    Args:
        obs: Observation
        cube_initial_z: 초기 큐브 Z 위치
        config: 게이팅 기준
    
    Returns:
        (reward, info_dict)
    """
    if config is None:
        config = GateConfig()
    
    # 큐브 위치 재구성 (obs[8:11]은 EE 기준 상대 좌표)
    # 실제 env에서 절대 좌표를 추가로 전달해야 함
    # 여기서는 info dict에서 받는다고 가정
    
    reward = 0.0
    info = {}
    
    # 높이 차이 보상은 env에서 직접 계산
    # (여기서는 placeholder)
    
    return reward, info


def compute_place_reward(obs: np.ndarray,
                        prev_dist_to_target: float = None,
                        config: GateConfig = None) -> Tuple[float, Dict]:
    """
    PLACE 단계 보상: 큐브를 타겟에 이동
    
    Args:
        obs: Observation
        prev_dist_to_target: 이전 큐브-타겟 거리
        config: 게이팅 기준
    
    Returns:
        (reward, info_dict)
    """
    if config is None:
        config = GateConfig()
    
    dist_cube_to_target = obs[26]
    reward = 0.0
    info = {}
    
    # Dense reward: 큐브-타겟 거리
    reward -= dist_cube_to_target * 2.0
    
    # 진전 보너스
    if prev_dist_to_target is not None and dist_cube_to_target < prev_dist_to_target:
        improvement = prev_dist_to_target - dist_cube_to_target
        reward += improvement * 3.0
        info["improvement"] = improvement
    
    # Milestone: PLACE 달성
    if dist_cube_to_target < config.place_dist_threshold:
        info["placed"] = True
    
    # SUCCESS
    if dist_cube_to_target < config.success_threshold:
        info["success"] = True
    
    return reward, info


def compute_hybrid_reward(obs: np.ndarray,
                         prev_obs: np.ndarray,
                         grasp_valid: bool,
                         lifted: bool,
                         cube_initial_z: float,
                         cube_current_z: float,
                         config: GateConfig = None) -> Tuple[float, Dict]:
    """
    전체 Hybrid Reward 계산
    
    Args:
        obs: 현재 observation
        prev_obs: 이전 observation
        grasp_valid: grasp_gate 결과
        lifted: LIFT 달성 여부
        cube_initial_z: 초기 큐브 Z
        cube_current_z: 현재 큐브 Z
        config: 게이팅 기준
    
    Returns:
        (total_reward, info_dict)
    """
    if config is None:
        config = GateConfig()
    
    total_reward = 0.0
    info = {}
    
    # Time penalty
    total_reward -= 0.001
    
    # 1. REACH 보상
    prev_dist = prev_obs[25] if prev_obs is not None else None
    r_reach, info_reach = compute_reach_reward(obs, prev_dist, config)
    total_reward += r_reach
    info.update(info_reach)
    
    # 2. ALIGN 보상
    r_align, info_align = compute_align_reward(obs, config)
    total_reward += r_align * 0.5  # 가중치
    info.update(info_align)
    
    # 3. GRASP 보상
    r_grasp, info_grasp = compute_grasp_reward(obs, grasp_valid, config)
    total_reward += r_grasp
    info.update(info_grasp)
    
    # 4. LIFT 보상 (grasp_valid일 때만)
    if grasp_valid:
        lift_height = cube_current_z - cube_initial_z
        total_reward += lift_height * 50.0  # 들어올릴수록 큰 보상
        
        if lift_height > config.lift_height_threshold:
            info["lifted"] = True
    
    # 5. PLACE 보상 (lifted일 때만)
    if lifted:
        prev_dist_to_target = prev_obs[26] if prev_obs is not None else None
        r_place, info_place = compute_place_reward(obs, prev_dist_to_target, config)
        total_reward += r_place
        info.update(info_place)
    
    return total_reward, info
