#!/usr/bin/env python3
"""
Train Cube Detector CNN
Supervised learning: image -> cube 3D position

Usage:
    python scripts/train/train_cube_detector.py
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset, random_split
import numpy as np
from pathlib import Path
import pickle
from datetime import datetime

from models.cube_detector import CubeDetector
from models.cube_detector_spatial import CubeDetectorSpatialSoftmax


def load_data(data_path):
    """Load collected data"""
    with open(data_path, 'rb') as f:
        data = pickle.load(f)
    
    images = data['images']  # [N, H, W, C]
    positions = data['positions']  # [N, 3]
    
    print(f"✅ Loaded {len(images)} samples")
    print(f"   Images: {images.shape}")
    print(f"   Positions: {positions.shape}")
    
    # Preprocess images: HWC -> CHW, normalize
    images = images.astype(np.float32) / 255.0
    images = np.transpose(images, (0, 3, 1, 2))  # NCHW
    
    return images, positions


def train_detector(data_path, epochs=50, batch_size=64, lr=1e-3, use_spatial=False):
    """Train the cube detector"""
    print("="*80)
    model_type = "Spatial Softmax" if use_spatial else "Standard"
    print(f"🧠 Training Cube Detector CNN ({model_type})")
    print("="*80)
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")
    
    # Load data
    images, positions = load_data(data_path)
    input_size = images.shape[2]  # Height
    
    # Convert to tensors
    X = torch.from_numpy(images).float()
    y = torch.from_numpy(positions).float()
    
    # Create dataset
    dataset = TensorDataset(X, y)
    
    # Train/val split (90/10)
    train_size = int(0.9 * len(dataset))
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = random_split(dataset, [train_size, val_size])
    
    print(f"Train samples: {train_size}")
    print(f"Val samples: {val_size}")
    
    # DataLoaders
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)
    
    # Model - choose based on flag
    input_channels = images.shape[1]  # Number of channels (3 for RGB, 4 for RGBD)
    if use_spatial:
        model = CubeDetectorSpatialSoftmax(input_channels=input_channels, output_dim=3, input_size=input_size).to(device)
    else:
        model = CubeDetector(input_channels=input_channels, output_dim=3, input_size=input_size).to(device)
    print(f"Model parameters: {sum(p.numel() for p in model.parameters()):,}")
    
    # Loss and optimizer
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, patience=5, factor=0.5)
    
    # Training loop
    best_val_loss = float('inf')
    best_model_state = None
    
    output_dir = Path("models/trained")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print("\n📈 Training...")
    for epoch in range(epochs):
        # Train
        model.train()
        train_loss = 0.0
        for batch_X, batch_y in train_loader:
            batch_X, batch_y = batch_X.to(device), batch_y.to(device)
            
            optimizer.zero_grad()
            outputs = model(batch_X)
            loss = criterion(outputs, batch_y)
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item() * batch_X.size(0)
        
        train_loss /= train_size
        
        # Validate
        model.eval()
        val_loss = 0.0
        val_errors = []
        
        with torch.no_grad():
            for batch_X, batch_y in val_loader:
                batch_X, batch_y = batch_X.to(device), batch_y.to(device)
                
                outputs = model(batch_X)
                loss = criterion(outputs, batch_y)
                val_loss += loss.item() * batch_X.size(0)
                
                # Calculate position error in cm
                errors = torch.norm(outputs - batch_y, dim=1) * 100  # meters to cm
                val_errors.extend(errors.cpu().numpy())
        
        val_loss /= val_size
        mean_error_cm = np.mean(val_errors)
        
        scheduler.step(val_loss)
        
        # Save best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_model_state = model.state_dict().copy()
        
        # Print progress
        if (epoch + 1) % 5 == 0 or epoch == 0:
            print(f"Epoch {epoch+1:3d}/{epochs}: "
                  f"Train Loss: {train_loss:.6f}, "
                  f"Val Loss: {val_loss:.6f}, "
                  f"Mean Error: {mean_error_cm:.2f} cm")
    
    # Save best model
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    model_path = output_dir / f"cube_detector_{timestamp}.pth"
    torch.save(best_model_state, model_path)
    
    # Also save as 'latest'
    latest_path = output_dir / "cube_detector_latest.pth"
    torch.save(best_model_state, latest_path)
    
    print("\n✅ Training complete!")
    print(f"   Best Val Loss: {best_val_loss:.6f}")
    print(f"   Model saved: {model_path}")
    print(f"   Latest saved: {latest_path}")
    
    # Final evaluation
    model.load_state_dict(best_model_state)
    model.eval()
    
    all_errors = []
    with torch.no_grad():
        for batch_X, batch_y in val_loader:
            batch_X, batch_y = batch_X.to(device), batch_y.to(device)
            outputs = model(batch_X)
            errors = torch.norm(outputs - batch_y, dim=1) * 100
            all_errors.extend(errors.cpu().numpy())
    
    print(f"\n📊 Final Evaluation:")
    print(f"   Mean Error: {np.mean(all_errors):.2f} cm")
    print(f"   Std Error: {np.std(all_errors):.2f} cm")
    print(f"   Max Error: {np.max(all_errors):.2f} cm")
    print(f"   <1cm accuracy: {100*np.mean(np.array(all_errors) < 1):.1f}%")
    print(f"   <2cm accuracy: {100*np.mean(np.array(all_errors) < 2):.1f}%")
    
    return model_path


def find_latest_data():
    """Find latest data file"""
    data_dir = Path("data/cube_detection")
    if not data_dir.exists():
        return None
    
    data_files = list(data_dir.glob("*.pkl"))
    if not data_files:
        return None
    
    return max(data_files, key=lambda x: x.stat().st_mtime)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=str, default=None, help="Path to data file")
    parser.add_argument("--epochs", type=int, default=50, help="Training epochs")
    parser.add_argument("--batch_size", type=int, default=64, help="Batch size")
    parser.add_argument("--lr", type=float, default=1e-3, help="Learning rate")
    parser.add_argument("--spatial", action="store_true", help="Use Spatial Softmax model")
    args = parser.parse_args()
    
    # Find data
    if args.data:
        data_path = Path(args.data)
    else:
        data_path = find_latest_data()
    
    if data_path is None or not data_path.exists():
        print("❌ No data found. Run collect_detection_data.py first.")
        sys.exit(1)
    
    print(f"Using data: {data_path}")
    
    train_detector(
        data_path=data_path,
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        use_spatial=args.spatial
    )

