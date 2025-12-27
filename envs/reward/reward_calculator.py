#!/usr/bin/env python3
"""
보상 계산 모듈 (v3.9.6)
RoArm-M3 Pick & Place 환경의 보상 계산 로직
"""

import numpy as np
from typing import Dict


class RewardCalculator:
    """보상 계산기 (v3.9.6 로깅 최소화)"""
    
    def __init__(self):
        """초기화"""
        # 이전 거리 (개선량 계산용)
        self.prev_dist_to_cube = None
        self.prev_cube_to_target_dist = None
        
        # v3.9.6: 누적 통계만 (리스트 제거)
        self.episode_min_distance = float('inf')
        self.episode_distance_sum = 0.0
        self.episode_distance_count = 0
        
        # 단계별 보상 카운터 (v3.9.4)
        self.stage_7cm_count = 0
        self.stage_5cm_count = 0
        self.stage_3cm_count = 0
        
        # 단계별 트리거 플래그 (1회만)
        self.stage_7cm_triggered = 0
        self.stage_5cm_triggered = 0
        self.stage_3cm_triggered = 0
        
        # 그리퍼 보상 통계
        self.gripper_reward_count = 0
        self.gripper_reward_total = 0.0
        
        # 보상 분해 (v3.9.6: 5개 항목)
        self.episode_reward_breakdown = {
            'distance_reward': 0.0,
            'stage_bonus': 0.0,
            'gripper_bonus': 0.0,
            'synergy_bonus': 0.0,
            'milestone_bonus': 0.0,
        }
    
    def calculate_reward(
        self,
        dist_to_cube: float,
        gripper_width: float,
        is_attached: bool,
        dist_cube_to_target: float,
        cube_lift_z: float,
        step_count: int
    ) -> float:
        """
        보상 계산 (v3.9.4 기반)
        
        Args:
            dist_to_cube: EE에서 큐브까지 거리 (m)
            gripper_width: 그리퍼 폭 (m)
            is_attached: 부착 여부
            dist_cube_to_target: 큐브에서 타겟까지 거리 (m)
            cube_lift_z: 큐브 높이 (m)
            step_count: 현재 스텝 수
        
        Returns:
            reward: 총 보상
        """
        reward = 0.0
        
        # ═══════════════════════════════════════════════════════════
        # 1. 거리 기반 보상 (Dense)
        # ═══════════════════════════════════════════════════════════
        if self.prev_dist_to_cube is not None:
            dist_improvement = self.prev_dist_to_cube - dist_to_cube
            dist_reward = 100.0 * dist_improvement
            reward += dist_reward
            self.episode_reward_breakdown['distance_reward'] += dist_reward
        
        self.prev_dist_to_cube = dist_to_cube
        
        # 통계 업데이트
        self.episode_distance_sum += dist_to_cube
        self.episode_distance_count += 1
        self.episode_min_distance = min(self.episode_min_distance, dist_to_cube)
        
        # ═══════════════════════════════════════════════════════════
        # 2. 단계별 보상 (v3.9.4: 점진적 유도)
        # ═══════════════════════════════════════════════════════════
        stage_bonus = 0.0
        
        # Stage 1: 7cm 진입 (초기 신호)
        if dist_to_cube < 0.07 and self.stage_7cm_triggered == 0:
            stage_bonus += 10.0
            self.stage_7cm_triggered = 1
            self.stage_7cm_count += 1
        
        # Stage 2: 5cm 진입 (중간 목표)
        if dist_to_cube < 0.05 and self.stage_5cm_triggered == 0:
            stage_bonus += 20.0
            self.stage_5cm_triggered = 1
            self.stage_5cm_count += 1
        
        # Stage 3: 3cm 진입 (최종 목표, v3.9.2에서 +20 → v3.9.4에서 +30)
        if dist_to_cube < 0.03 and self.stage_3cm_triggered == 0:
            stage_bonus += 30.0
            self.stage_3cm_triggered = 1
            self.stage_3cm_count += 1
        
        reward += stage_bonus
        self.episode_reward_breakdown['stage_bonus'] += stage_bonus
        
        # ═══════════════════════════════════════════════════════════
        # 3. 그리퍼 보상 (v3.9.2 유지)
        # ═══════════════════════════════════════════════════════════
        gripper_reward = 3.0  # 일관성 유지 (v3.9.3 조건부 보상 제거)
        reward += gripper_reward
        self.gripper_reward_count += 1
        self.gripper_reward_total += gripper_reward
        self.episode_reward_breakdown['gripper_bonus'] += gripper_reward
        
        # ═══════════════════════════════════════════════════════════
        # 4. Synergy Bonus (거리-폭 연동, v3.9.4 완화)
        # ═══════════════════════════════════════════════════════════
        synergy_bonus = 0.0
        IDEAL_MIN = 0.03  # 3cm
        IDEAL_MAX = 0.06  # 6cm
        
        if IDEAL_MIN <= gripper_width <= IDEAL_MAX:
            # 단계별 연동 보상
            if dist_to_cube < 0.10:
                synergy_bonus += 10.0  # 초기 신호 (10cm 이내)
            if dist_to_cube < 0.05:
                synergy_bonus += 40.0  # 최종 목표 (5cm 이내)
        
        reward += synergy_bonus
        self.episode_reward_breakdown['synergy_bonus'] += synergy_bonus
        
        # ═══════════════════════════════════════════════════════════
        # 5. Width Penalty (이상적 폭 유도, v3.7.7)
        # ═══════════════════════════════════════════════════════════
        width_penalty = 0.0
        if dist_to_cube < 0.15:  # 15cm 이내에서만
            IDEAL_WIDTH = 0.04  # 4cm (큐브와 동일)
            width_error = gripper_width - IDEAL_WIDTH
            width_penalty = -5.0 * (width_error ** 2)
            reward += width_penalty
            # v3.9.6: width_penalty 제거됨 (간소화)
        
        # ═══════════════════════════════════════════════════════════
        # 6. Cube → Target 접근 보상 (부착 시만)
        # ═══════════════════════════════════════════════════════════
        if is_attached and self.prev_cube_to_target_dist is not None:
            cube_progress = self.prev_cube_to_target_dist - dist_cube_to_target
            reward += 15.0 * cube_progress  # 부착 후 목표로 이동
        
        self.prev_cube_to_target_dist = dist_cube_to_target
        
        # ═══════════════════════════════════════════════════════════
        # 7. 마일스톤 보상 (Reach/Attach/Lift, v3.9.0)
        # ═══════════════════════════════════════════════════════════
        # 이 부분은 메인 환경에서 처리 (is_attached, lift_z 상태 필요)
        # 여기서는 milestone_bonus 값을 외부에서 받아서 누적만
        
        return reward
    
    def add_milestone_bonus(self, bonus: float):
        """마일스톤 보상 추가 (외부에서 호출)"""
        self.episode_reward_breakdown['milestone_bonus'] += bonus
    
    def reset(self):
        """에피소드 리셋"""
        self.prev_dist_to_cube = None
        self.prev_cube_to_target_dist = None
        
        # 통계 리셋
        self.episode_min_distance = float('inf')
        self.episode_distance_sum = 0.0
        self.episode_distance_count = 0
        
        # 단계별 트리거 리셋
        self.stage_7cm_triggered = 0
        self.stage_5cm_triggered = 0
        self.stage_3cm_triggered = 0
        
        # 카운터 리셋
        self.stage_7cm_count = 0
        self.stage_5cm_count = 0
        self.stage_3cm_count = 0
        
        # 그리퍼 통계 리셋
        self.gripper_reward_count = 0
        self.gripper_reward_total = 0.0
        
        # 보상 분해 리셋
        self.episode_reward_breakdown = {
            'distance_reward': 0.0,
            'stage_bonus': 0.0,
            'gripper_bonus': 0.0,
            'synergy_bonus': 0.0,
            'milestone_bonus': 0.0,
        }
    
    def get_stats(self) -> Dict:
        """통계 반환"""
        mean_dist = (self.episode_distance_sum / self.episode_distance_count 
                     if self.episode_distance_count > 0 else 0)
        
        mean_gripper_reward = (self.gripper_reward_total / self.gripper_reward_count
                               if self.gripper_reward_count > 0 else 0)
        
        return {
            'min_distance': self.episode_min_distance,
            'mean_distance': mean_dist,
            'stage_7cm_count': self.stage_7cm_count,
            'stage_5cm_count': self.stage_5cm_count,
            'stage_3cm_count': self.stage_3cm_count,
            'gripper_reward_count': self.gripper_reward_count,
            'mean_gripper_reward': mean_gripper_reward,
            'reward_breakdown': self.episode_reward_breakdown.copy(),
        }
