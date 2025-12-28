#!/usr/bin/env python3
"""
Joint Position Sweep Test for Camera Visibility
Tests various joint configurations to find when cube is visible
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({'headless': True})

import numpy as np
from PIL import Image
import os

# Setup environment
from envs.pick_place_vision_env import PickPlaceVisionEnv
from envs.utils.physics_utils import detect_cube_in_image

env = PickPlaceVisionEnv()

# Test configurations: [j1, j2, j3, j4, j5] in radians
# j2 and j3 control arm tilt (forward/down lean)
test_configs = []

# Generate test configurations
for j2 in np.linspace(-1.2, 0.5, 6):      # Shoulder tilt
    for j3 in np.linspace(-1.0, 0.5, 5):  # Elbow
        for j4 in np.linspace(-0.5, 0.5, 3):  # Wrist 1
            test_configs.append([0.0, j2, j3, j4, 0.0, 0.06])

print(f"Testing {len(test_configs)} joint configurations...")
print("="*60)

results = []
output_dir = "/home/roarm_m3/roarm_isaac_clean/resources/camera_visibility_test"
os.makedirs(output_dir, exist_ok=True)

visible_count = 0

for i, config in enumerate(test_configs):
    # Set joint positions
    if env.robot is not None:
        env.robot.set_joint_positions(np.array(config))
        env.robot.set_joint_velocities(np.zeros(6))
    
    # Step simulation to settle
    for _ in range(5):
        env.world.step(render=True)
    
    # Update camera
    try:
        import omni.replicator.core as rep
        rep.orchestrator.step(rt_subframes=4)
    except:
        pass
    
    # Get image
    raw_img = env._get_raw_image()
    if raw_img is not None:
        detected, ratio, centroid = detect_cube_in_image(raw_img)
        
        if detected:
            visible_count += 1
            # Save image
            img = Image.fromarray(raw_img.astype(np.uint8))
            j2, j3, j4 = config[1:4]
            fname = f"{output_dir}/visible_j2_{j2:.2f}_j3_{j3:.2f}_j4_{j4:.2f}.png"
            img.save(fname)
            
            results.append({
                'config': config[:5],
                'ratio': ratio,
                'centroid': centroid
            })
            
            if visible_count <= 5:  # Print first 5
                print(f"✅ VISIBLE: j2={j2:.2f}, j3={j3:.2f}, j4={j4:.2f} | red={ratio*100:.2f}%")

    if i % 30 == 0:
        print(f"  Progress: {i}/{len(test_configs)}")

print("="*60)
print(f"\n📊 Results Summary:")
print(f"  Total tested: {len(test_configs)}")
print(f"  Cube visible: {visible_count} ({visible_count/len(test_configs)*100:.1f}%)")

if results:
    # Find best configuration (highest red ratio)
    best = max(results, key=lambda x: x['ratio'])
    print(f"\n🎯 Best configuration (highest visibility):")
    print(f"  Joints: {[f'{j:.3f}' for j in best['config']]}")
    print(f"  Red ratio: {best['ratio']*100:.2f}%")
    print(f"  Centroid: {best['centroid']}")
    
    # Save summary
    with open(f"{output_dir}/summary.txt", 'w') as f:
        f.write("Visible Joint Configurations\n")
        f.write("="*50 + "\n")
        for r in sorted(results, key=lambda x: -x['ratio'])[:20]:
            f.write(f"Joints: {r['config']}, Red: {r['ratio']*100:.2f}%\n")
    
    print(f"\n📁 Images saved to: {output_dir}/")
else:
    print("\n❌ Cube not visible in any configuration!")
    print("   Camera may need orientation adjustment.")

env.close()
simulation_app.close()
