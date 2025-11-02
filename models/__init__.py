"""
Models Package for Vision-Based RL

This package contains CNN feature extractors for vision-based reinforcement learning
with Stable-Baselines3.

Modules:
    cnn_extractor: CNN architectures (NatureCNN, LargerCNN)

Example:
    >>> from models.cnn_extractor import NatureCNN
    >>> from stable_baselines3 import SAC
    >>> 
    >>> policy_kwargs = dict(
    ...     features_extractor_class=NatureCNN,
    ...     features_extractor_kwargs=dict(features_dim=512),
    ... )
    >>> model = SAC("CnnPolicy", env, policy_kwargs=policy_kwargs)
"""

from .cnn_extractor import NatureCNN, LargerCNN

__all__ = ["NatureCNN", "LargerCNN"]
