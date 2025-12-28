#!/usr/bin/env python3
"""
CNN Feature Extractor for Vision-based RL

Architecture: NatureCNN (Atari-style)
- Conv1: 32 filters (8×8, stride 4)
- Conv2: 64 filters (4×4, stride 2)
- Conv3: 64 filters (3×3, stride 1)
- Flatten + Linear(512)

Input: (3, 84, 84) - RGB or (4, 84, 84) - RGBD
Output: (512,) - Feature vector

작성일: 2025-11-02
"""

import torch
import torch.nn as nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from gymnasium import spaces


class NatureCNN(BaseFeaturesExtractor):
    """
    CNN Feature Extractor for Vision RL
    
    Based on Nature DQN architecture with modifications for RGBD input
    
    Args:
        observation_space: Gym observation space (Box)
        features_dim: Output feature dimension (default: 512)
    
    Example:
        >>> obs_space = spaces.Box(0, 1, shape=(4, 84, 84))
        >>> cnn = NatureCNN(obs_space, features_dim=512)
        >>> batch = torch.randn(8, 4, 84, 84)
        >>> features = cnn(batch)
        >>> print(features.shape)  # (8, 512)
    """
    
    def __init__(self, observation_space: spaces.Box, features_dim: int = 512):
        super().__init__(observation_space, features_dim)
        
        # Input channels (RGB: 3, RGBD: 4)
        n_input_channels = observation_space.shape[0]
        
        # CNN layers
        self.cnn = nn.Sequential(
            # Conv1: (4, 84, 84) → (32, 20, 20)
            nn.Conv2d(n_input_channels, 32, kernel_size=8, stride=4, padding=0),
            nn.ReLU(),
            
            # Conv2: (32, 20, 20) → (64, 9, 9)
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            
            # Conv3: (64, 9, 9) → (64, 7, 7)
            nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=0),
            nn.ReLU(),
            
            # Flatten: (64, 7, 7) → (3136,)
            nn.Flatten(),
        )
        
        # Compute flatten dimension
        with torch.no_grad():
            sample_input = torch.zeros(1, *observation_space.shape)
            n_flatten = self.cnn(sample_input).shape[1]
        
        # Linear layers
        self.linear = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU(),
        )
        
        print(f"✅ NatureCNN initialized")
        print(f"   Input: {observation_space.shape}")
        print(f"   Flatten dim: {n_flatten}")
        print(f"   Output: {features_dim}")
    
    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        """
        Forward pass
        
        Args:
            observations: (batch, channels, height, width)
        
        Returns:
            features: (batch, features_dim)
        """
        return self.linear(self.cnn(observations))


class LargerCNN(BaseFeaturesExtractor):
    """
    Larger CNN with more capacity
    
    Architecture:
        Conv1: 32 → 8×8, stride 4
        Conv2: 64 → 4×4, stride 2
        Conv3: 128 → 3×3, stride 1
        Conv4: 128 → 3×3, stride 1
        Linear(1024)
    
    Use when:
        - More complex visual features
        - Larger dataset
        - Need better representation
    """
    
    def __init__(self, observation_space: spaces.Box, features_dim: int = 1024):
        super().__init__(observation_space, features_dim)
        
        n_input_channels = observation_space.shape[0]
        
        self.cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 32, kernel_size=8, stride=4, padding=0),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=0),
            nn.ReLU(),
            nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=0),
            nn.ReLU(),
            nn.Flatten(),
        )
        
        with torch.no_grad():
            n_flatten = self.cnn(torch.zeros(1, *observation_space.shape)).shape[1]
        
        self.linear = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU(),
        )
        
        print(f"✅ LargerCNN initialized")
        print(f"   Input: {observation_space.shape}")
        print(f"   Flatten dim: {n_flatten}")
        print(f"   Output: {features_dim}")
    
    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        return self.linear(self.cnn(observations))


def test_cnn():
    """Test CNN feature extractor"""
    print("=" * 80)
    print("🧪 Testing CNN Feature Extractor")
    print("=" * 80)
    
    # Create observation space
    obs_space = spaces.Box(low=0, high=1, shape=(4, 84, 84), dtype=float)
    
    # Test NatureCNN
    print("\n1. NatureCNN (512)")
    cnn = NatureCNN(obs_space, features_dim=512)
    
    # Create batch
    batch_size = 8
    sample = torch.randn(batch_size, 4, 84, 84)
    
    # Forward pass
    features = cnn(sample)
    
    print(f"\n   Input shape: {sample.shape}")
    print(f"   Output shape: {features.shape}")
    print(f"   Output range: [{features.min():.3f}, {features.max():.3f}]")
    print(f"   Output mean: {features.mean():.3f}")
    print(f"   Output std: {features.std():.3f}")
    
    # Test LargerCNN
    print("\n2. LargerCNN (1024)")
    large_cnn = LargerCNN(obs_space, features_dim=1024)
    
    features_large = large_cnn(sample)
    
    print(f"\n   Input shape: {sample.shape}")
    print(f"   Output shape: {features_large.shape}")
    print(f"   Output range: [{features_large.min():.3f}, {features_large.max():.3f}]")
    
    # Parameter count
    nature_params = sum(p.numel() for p in cnn.parameters())
    large_params = sum(p.numel() for p in large_cnn.parameters())
    
    print("\n📊 Model Comparison")
    print(f"   NatureCNN: {nature_params:,} parameters")
    print(f"   LargerCNN: {large_params:,} parameters")
    print(f"   Ratio: {large_params/nature_params:.2f}x")
    
    print("\n✅ All tests passed!")


if __name__ == "__main__":
    test_cnn()
