"""
⚠️ DEPRECATED: This module is deprecated.

All reward calculation logic has been consolidated into:
    envs/reward/reward_calculator.py

The GateConfig class has been moved to:
    configs/base.yaml (as reward section)

For new code, use:
    from envs.reward.reward_calculator import RewardCalculator
    
    # Or for config-driven rewards:
    from configs.config_loader import get_config
    config = get_config("base")
    reward_config = config["reward"]
"""

import warnings

warnings.warn(
    "rewards/ module is deprecated. Use envs/reward/reward_calculator.py instead.",
    DeprecationWarning,
    stacklevel=2
)

# Re-export for backward compatibility
from rewards.pick_place import (
    GateConfig,
    grasp_gate,
    compute_reach_reward,
    compute_align_reward,
    compute_grasp_reward,
    compute_lift_reward,
    compute_place_reward,
    compute_hybrid_reward,
)

__all__ = [
    "GateConfig",
    "grasp_gate", 
    "compute_reach_reward",
    "compute_align_reward",
    "compute_grasp_reward",
    "compute_lift_reward",
    "compute_place_reward",
    "compute_hybrid_reward",
]
