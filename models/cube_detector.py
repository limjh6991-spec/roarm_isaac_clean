#!/usr/bin/env python3
"""
Cube Detector CNN Model
Predicts cube 3D position from RGB image

Input: RGB image (84x84x3)
Output: Cube position (x, y, z)
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np


class CubeDetector(nn.Module):
    """
    CNN that predicts cube 3D position from image
    Supports both 84x84 and 224x224 input
    """
    
    def __init__(self, input_channels=3, output_dim=3, input_size=224):
        super().__init__()
        
        self.input_size = input_size
        
        # CNN feature extractor (similar to NatureCNN)
        self.conv1 = nn.Conv2d(input_channels, 32, kernel_size=8, stride=4)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=4, stride=2)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1)
        
        # Calculate flattened size dynamically
        # For 84x84: 84 -> 20 -> 9 -> 7, 7*7*64 = 3136
        # For 224x224: 224 -> 55 -> 27 -> 25, 25*25*64 = 40000
        self.flatten_dim = self._get_flatten_dim(input_channels, input_size)
        
        # MLP for position regression
        self.fc1 = nn.Linear(self.flatten_dim, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, output_dim)
        
        # Dropout for regularization
        self.dropout = nn.Dropout(0.3)
    
    def _get_flatten_dim(self, channels, size):
        """Calculate flatten dimension dynamically"""
        x = torch.zeros(1, channels, size, size)
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        return x.view(1, -1).size(1)
        
    def forward(self, x):
        """
        Args:
            x: Image tensor [B, C, H, W] normalized to [0, 1]
        Returns:
            pos: Predicted position [B, 3]
        """
        # CNN layers
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        
        # Flatten
        x = x.view(x.size(0), -1)
        
        # MLP layers
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = F.relu(self.fc2(x))
        x = self.dropout(x)
        x = self.fc3(x)
        
        return x
    
    def predict(self, image: np.ndarray) -> np.ndarray:
        """
        Predict cube position from numpy image
        
        Args:
            image: RGB image [H, W, C] or [C, H, W], uint8 or float
        Returns:
            position: [x, y, z] in meters
        """
        self.eval()
        
        with torch.no_grad():
            # Preprocess
            if image.dtype == np.uint8:
                image = image.astype(np.float32) / 255.0
            
            # Ensure CHW format
            if image.shape[-1] == 3:  # HWC -> CHW
                image = np.transpose(image, (2, 0, 1))
            
            # Add batch dimension
            x = torch.from_numpy(image).unsqueeze(0).float()
            
            # Move to same device as model
            device = next(self.parameters()).device
            x = x.to(device)
            
            # Predict
            pos = self(x)
            
            return pos.cpu().numpy()[0]


class CubeDetectorWithConfidence(CubeDetector):
    """
    Extended version that also outputs confidence score
    """
    
    def __init__(self, input_channels=3):
        super().__init__(input_channels, output_dim=4)  # x, y, z, confidence
        
    def forward(self, x):
        out = super().forward(x)
        pos = out[:, :3]
        confidence = torch.sigmoid(out[:, 3:4])  # 0-1 confidence
        return pos, confidence


def create_cube_detector(pretrained_path=None):
    """
    Factory function to create cube detector
    """
    model = CubeDetector()
    
    if pretrained_path:
        state_dict = torch.load(pretrained_path, map_location='cpu')
        model.load_state_dict(state_dict)
        print(f"✅ Loaded pretrained weights from {pretrained_path}")
    
    return model


# Test
if __name__ == "__main__":
    model = CubeDetector()
    print(f"Model parameters: {sum(p.numel() for p in model.parameters()):,}")
    
    # Test forward pass
    dummy_input = torch.randn(1, 3, 84, 84)
    output = model(dummy_input)
    print(f"Input shape: {dummy_input.shape}")
    print(f"Output shape: {output.shape}")
    print(f"Output: {output}")
