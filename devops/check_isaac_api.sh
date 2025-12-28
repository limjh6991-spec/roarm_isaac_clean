#!/bin/bash
# Isaac Sim 4.2.0 API 호환성 체크 스크립트

IMAGE="nvcr.io/nvidia/isaac-sim:4.2.0"

echo "=========================================="
echo "Isaac Sim 4.2.0 API 호환성 확인"
echo "=========================================="
echo ""

docker run --rm \
    --runtime=nvidia \
    --gpus all \
    -e ACCEPT_EULA=Y \
    -v ~/roarm_isaac_clean:/workspace \
    -w /workspace \
    ${IMAGE} \
    python3 -c "
import sys
print('Python version:', sys.version)
print()

# Check for Isaac Sim modules
try:
    import omni.isaac.kit
    print('✅ omni.isaac.kit available')
except ImportError:
    print('❌ omni.isaac.kit NOT available')

try:
    import omni.isaac.core
    print('✅ omni.isaac.core available')
except ImportError:
    print('❌ omni.isaac.core NOT available')

try:
    from isaaclab.app import AppLauncher
    print('✅ isaaclab available')
except ImportError:
    print('❌ isaaclab NOT available (expected for Isaac Sim 4.2.0)')

print()
print('Available omni.isaac modules:')
import pkgutil
import omni.isaac
for importer, modname, ispkg in pkgutil.iter_modules(omni.isaac.__path__, 'omni.isaac.'):
    print(f'  - {modname}')
"
