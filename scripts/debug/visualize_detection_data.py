#!/usr/bin/env python3
"""
Visualize Cube Detection Data
Check if cube is visible in collected images
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))

import pickle
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def visualize_data():
    # Find latest data
    data_dir = Path("data/cube_detection")
    data_files = list(data_dir.glob("*.pkl"))
    
    if not data_files:
        print("No data found!")
        return
    
    data_path = max(data_files, key=lambda x: x.stat().st_mtime)
    print(f"Loading: {data_path}")
    
    with open(data_path, 'rb') as f:
        data = pickle.load(f)
    
    images = data['images']
    positions = data['positions']
    
    print(f"Images: {images.shape}")
    print(f"Positions: {positions.shape}")
    print(f"Position range X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}]")
    print(f"Position range Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}]")
    print(f"Position range Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}]")
    print(f"Position mean: [{positions[:, 0].mean():.3f}, {positions[:, 1].mean():.3f}, {positions[:, 2].mean():.3f}]")
    print(f"Position std: [{positions[:, 0].std():.3f}, {positions[:, 1].std():.3f}, {positions[:, 2].std():.3f}]")
    
    # Save sample images
    output_dir = Path("data/cube_detection/samples")
    output_dir.mkdir(exist_ok=True)
    
    # Random samples
    indices = np.random.choice(len(images), size=10, replace=False)
    
    fig, axes = plt.subplots(2, 5, figsize=(15, 6))
    for i, idx in enumerate(indices):
        ax = axes[i // 5, i % 5]
        ax.imshow(images[idx])
        ax.set_title(f"Pos: ({positions[idx, 0]:.2f}, {positions[idx, 1]:.2f}, {positions[idx, 2]:.2f})")
        ax.axis('off')
    
    plt.tight_layout()
    plt.savefig(output_dir / "sample_images.png", dpi=150)
    print(f"\n✅ Saved sample images to {output_dir / 'sample_images.png'}")
    
    # Check if CNN is just predicting mean
    mean_pos = positions.mean(axis=0)
    mse_mean = np.mean((positions - mean_pos) ** 2)
    print(f"\nIf CNN predicts mean: MSE would be {mse_mean:.6f} (error ~{np.sqrt(mse_mean)*100:.1f} cm)")
    
    # Check image variance
    img_variance = images.astype(np.float32).var()
    print(f"Image variance: {img_variance:.2f}")


if __name__ == "__main__":
    visualize_data()
