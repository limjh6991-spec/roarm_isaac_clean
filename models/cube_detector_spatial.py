#!/usr/bin/env python3
"""
Spatial Softmax CNN for Cube Detection
Preserves spatial information by extracting keypoint coordinates from feature maps

Reference: "Deep Spatial Autoencoders for Visuomotor Learning" (Finn et al., 2015)
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np


class SpatialSoftmax(nn.Module):
    """
    Spatial Softmax layer that extracts 2D keypoint coordinates from feature maps
    
    For each feature channel:
    1. Apply softmax across spatial dimensions
    2. Compute expected (x, y) coordinate as weighted sum
    
    Output: [batch, channels * 2] - (x, y) for each channel
    """
    
    def __init__(self, height, width, num_channels, temperature=1.0, learnable_temp=False):
        super().__init__()
        
        self.height = height
        self.width = width
        self.num_channels = num_channels
        
        # Temperature for softmax sharpness
        if learnable_temp:
            self.temperature = nn.Parameter(torch.ones(1) * temperature)
        else:
            self.register_buffer('temperature', torch.ones(1) * temperature)
        
        # Create coordinate grids
        # x: [-1, 1] across width, y: [-1, 1] across height
        pos_x, pos_y = torch.meshgrid(
            torch.linspace(-1., 1., width),
            torch.linspace(-1., 1., height),
            indexing='xy'
        )
        
        # [1, 1, H, W]
        self.register_buffer('pos_x', pos_x.reshape(1, 1, height, width))
        self.register_buffer('pos_y', pos_y.reshape(1, 1, height, width))
    
    def forward(self, features):
        """
        Args:
            features: [B, C, H, W] feature maps
        Returns:
            keypoints: [B, C*2] flattened (x, y) coordinates for each channel
        """
        batch_size, channels, height, width = features.shape
        
        # Verify dimensions
        assert height == self.height and width == self.width, \
            f"Expected ({self.height}, {self.width}), got ({height}, {width})"
        
        # Apply softmax across spatial dimensions
        # [B, C, H, W] -> [B, C, H*W]
        features_flat = features.view(batch_size, channels, -1)
        attention = F.softmax(features_flat / self.temperature, dim=-1)
        attention = attention.view(batch_size, channels, height, width)
        
        # Compute expected coordinates (weighted sum)
        # [B, C, 1, 1]
        expected_x = (attention * self.pos_x).sum(dim=[2, 3])  # [B, C]
        expected_y = (attention * self.pos_y).sum(dim=[2, 3])  # [B, C]
        
        # Interleave x, y coordinates: [x1, y1, x2, y2, ...]
        keypoints = torch.stack([expected_x, expected_y], dim=-1)  # [B, C, 2]
        keypoints = keypoints.view(batch_size, -1)  # [B, C*2]
        
        return keypoints


class CubeDetectorSpatialSoftmax(nn.Module):
    """
    CNN with Spatial Softmax for accurate cube 3D position prediction
    
    Key improvement: Preserves WHERE in the image the cube is located
    """
    
    def __init__(self, input_channels=3, output_dim=3, input_size=224):
        super().__init__()
        
        self.input_size = input_size
        
        # Feature extractor (same as before but fewer pooling)
        self.conv1 = nn.Conv2d(input_channels, 32, kernel_size=8, stride=4, padding=2)
        self.bn1 = nn.BatchNorm2d(32)
        
        self.conv2 = nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=1)
        self.bn2 = nn.BatchNorm2d(64)
        
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=1)
        self.bn3 = nn.BatchNorm2d(64)
        
        # Calculate feature map size
        # 224 -> 56 (stride 4) -> 28 (stride 2) -> 28 (stride 1)
        self.feature_height = input_size // 8
        self.feature_width = input_size // 8
        
        # Spatial Softmax layer
        self.spatial_softmax = SpatialSoftmax(
            height=self.feature_height,
            width=self.feature_width,
            num_channels=64,
            temperature=1.0,
            learnable_temp=True
        )
        
        # Output: 64 channels * 2 (x, y) = 128 keypoint coordinates
        keypoint_dim = 64 * 2
        
        # Fully connected layers for 3D position regression
        self.fc1 = nn.Linear(keypoint_dim, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, output_dim)
        
        self.dropout = nn.Dropout(0.2)
    
    def forward(self, x):
        """
        Args:
            x: [B, C, H, W] normalized images
        Returns:
            pos: [B, 3] predicted (x, y, z) position
        """
        # Feature extraction
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        
        # Spatial softmax: [B, 64, H, W] -> [B, 128]
        keypoints = self.spatial_softmax(x)
        
        # Position regression
        x = F.relu(self.fc1(keypoints))
        x = self.dropout(x)
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        
        return x
    
    def predict(self, image: np.ndarray) -> np.ndarray:
        """Predict cube position from numpy image"""
        self.eval()
        
        with torch.no_grad():
            if image.dtype == np.uint8:
                image = image.astype(np.float32) / 255.0
            
            if image.shape[-1] == 3:  # HWC -> CHW
                image = np.transpose(image, (2, 0, 1))
            
            x = torch.from_numpy(image).unsqueeze(0).float()
            device = next(self.parameters()).device
            x = x.to(device)
            
            pos = self(x)
            return pos.cpu().numpy()[0]
    
    def get_keypoints(self, image: np.ndarray) -> np.ndarray:
        """Get intermediate keypoints for visualization"""
        self.eval()
        
        with torch.no_grad():
            if image.dtype == np.uint8:
                image = image.astype(np.float32) / 255.0
            
            if image.shape[-1] == 3:
                image = np.transpose(image, (2, 0, 1))
            
            x = torch.from_numpy(image).unsqueeze(0).float()
            device = next(self.parameters()).device
            x = x.to(device)
            
            # Get features
            x = F.relu(self.bn1(self.conv1(x)))
            x = F.relu(self.bn2(self.conv2(x)))
            x = F.relu(self.bn3(self.conv3(x)))
            
            keypoints = self.spatial_softmax(x)  # [1, 128]
            keypoints = keypoints.view(-1, 2)  # [64, 2]
            
            return keypoints.cpu().numpy()


# Test
if __name__ == "__main__":
    print("="*60)
    print("🧪 Testing Spatial Softmax CNN")
    print("="*60)
    
    model = CubeDetectorSpatialSoftmax(input_size=224)
    print(f"Model parameters: {sum(p.numel() for p in model.parameters()):,}")
    
    # Test forward pass
    dummy_input = torch.randn(2, 3, 224, 224)
    output = model(dummy_input)
    
    print(f"Input shape: {dummy_input.shape}")
    print(f"Output shape: {output.shape}")
    print(f"Output: {output[0]}")
    
    # Test keypoints
    keypoints = model.get_keypoints(np.random.rand(224, 224, 3).astype(np.float32))
    print(f"Keypoints shape: {keypoints.shape}")
    print(f"First 5 keypoints: {keypoints[:5]}")
    
    print("\n✅ Test passed!")
