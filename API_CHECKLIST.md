# API 버전 체크리스트 (API Version Checklist)

## 📌 현재 환경 (2025-10-20 기준)

### 시스템 정보
```
OS: Ubuntu 24.04.3 LTS (Noble Numbat)
Kernel: 6.14.0-33-generic
GPU: NVIDIA GeForce RTX 5090
CPU: AMD Ryzen 7 9800X3D 8-Core
RAM: 31GB
```

### 핵심 소프트웨어 버전
```
Isaac Sim: 5.0.0
Python: 3.11
CUDA: 12.8
Driver: 13.0
```

---

## 🔍 Isaac Sim API 버전 체크

### Isaac Sim 5.0.0 (현재)

#### 주요 변경사항
```python
# ✅ NEW API (Isaac Sim 5.0+)
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.asset.importer.urdf import import_urdf

# ❌ DEPRECATED (Isaac Sim 4.x)
from omni.isaac.core.world import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
# 경고: "has been deprecated in favor of isaacsim.*"
```

#### API 매핑 테이블
| Old (4.x) | New (5.0+) | Status |
|-----------|------------|--------|
| `omni.isaac.core.world.World` | `isaacsim.core.api.world.World` | ✅ Migrated |
| `omni.isaac.core.articulations.Articulation` | `isaacsim.core.prims.Articulation` | ✅ Migrated |
| `omni.isaac.core.utils.prims` | `isaacsim.core.utils.prims` | ✅ Migrated |
| `omni.isaac.core.utils.stage` | `isaacsim.core.utils.stage` | ✅ Migrated |
| `omni.isaac.cloner` | `isaacsim.core.cloner` | ✅ Migrated |
| `omni.isaac.sensor` | `isaacsim.sensors.*` | ✅ Migrated |

#### 확인 방법
```python
# 터미널에서 실행
~/isaacsim/python.sh -c "import isaacsim; print(isaacsim.__version__)"

# 또는 Python 스크립트에서
import sys
print(f"Python: {sys.version}")

try:
    import isaacsim
    print(f"Isaac Sim: {isaacsim.__version__ if hasattr(isaacsim, '__version__') else 'Unknown'}")
except ImportError:
    print("Isaac Sim not found")
```

---

## 🐍 Python 라이브러리 버전

### Stable-Baselines3 (RL)
```python
# 버전 확인
import stable_baselines3
print(f"SB3 Version: {stable_baselines3.__version__}")

# 권장 버전: 2.0.0+
# 설치: pip install stable-baselines3>=2.0.0
```

#### 주요 API
```python
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.callbacks import BaseCallback

# ⚠️ PPO on GPU Warning 대응
# GPU 사용 시 경고 발생 (MlpPolicy는 CPU 권장)
# 해결: device='cpu' 명시 또는 경고 무시
```

### Gymnasium (OpenAI Gym 후속)
```python
# 버전 확인
import gymnasium as gym
print(f"Gymnasium Version: {gym.__version__}")

# 권장 버전: 0.29.0+
# 주의: 이전 'gym' 패키지와 호환 안 됨
```

### NumPy
```python
# 버전 확인
import numpy as np
print(f"NumPy Version: {np.__version__}")

# ⚠️ 주요 이슈: NumPy 2.x vs 1.x 호환성
# Isaac Sim 5.0은 NumPy 2.2.6 사용
# 일부 패키지 (osqp 등)는 NumPy 1.x 컴파일
# 오류: "AttributeError: _ARRAY_API not found"
```

#### NumPy 호환성 해결
```python
# 해결 방법 1: NumPy 다운그레이드 (비추천)
# pip install "numpy<2.0"

# 해결 방법 2: 패키지 재컴파일
# pip install --no-binary :all: osqp

# 해결 방법 3: 경고 무시 (현재 방법)
import warnings
warnings.filterwarnings('ignore', message='.*NumPy 1.x.*')
```

---

## 🔧 Isaac Sim Extensions

### 확인 명령
```bash
# Isaac Sim에서 설치된 확장 목록
~/isaacsim/python.sh -c "
import omni.kit.app
app = omni.kit.app.get_app()
print('Installed Extensions:')
# 확장 목록 출력
"
```

### 주요 확장 버전
```
isaacsim.simulation_app: 2.9.2
isaacsim.core.api: 4.6.10
isaacsim.robot.manipulators: 3.3.5
isaacsim.asset.importer.urdf: 2.4.19
isaacsim.sensors.camera: 1.3.1
isaacsim.replicator.core: 1.12.16
```

---

## 📚 온라인 리소스 체크리스트

### 코딩 전 필수 확인

#### 1. Isaac Sim 공식 문서
```
URL: https://docs.omniverse.nvidia.com/isaacsim/latest/

[ ] Release Notes 확인
[ ] Migration Guide 확인 (4.x → 5.0)
[ ] API Reference 확인
[ ] Known Issues 확인
```

#### 2. GitHub Issues 검색
```
Repositories:
- https://github.com/isaac-sim/IsaacLab
- https://github.com/NVIDIA-Omniverse/IsaacGymEnvs

검색 키워드:
[ ] "gripper prismatic"
[ ] "URDF import"
[ ] "NumPy 2.0"
[ ] "AttributeError _ARRAY_API"
[ ] 현재 작업 관련 키워드
```

#### 3. Isaac Lab Migration Guide
```
URL: https://isaac-sim.github.io/IsaacLab/

[ ] Deprecated APIs 확인
[ ] New Features 확인
[ ] Breaking Changes 확인
```

#### 4. Stable-Baselines3 문서
```
URL: https://stable-baselines3.readthedocs.io/

[ ] PPO 하이퍼파라미터 가이드
[ ] VecNormalize 사용법
[ ] Custom Callback 작성법
```

---

## 🐛 알려진 이슈 및 해결 방법

### Issue 1: NumPy 2.x 호환성 오류
```
오류: AttributeError: _ARRAY_API not found
오류: numpy.core.multiarray failed to import

원인: NumPy 1.x로 컴파일된 모듈 vs NumPy 2.x 런타임

해결:
1. 경고 무시 (권장)
   import warnings
   warnings.filterwarnings('ignore', message='.*NumPy 1.x.*')

2. 모듈 재설치
   pip install --force-reinstall --no-binary :all: osqp

3. Isaac Sim 내장 Python 사용
   ~/isaacsim/python.sh (이미 호환성 해결됨)
```

### Issue 2: Deprecated API 경고
```
경고: "omni.isaac.* has been deprecated in favor of isaacsim.*"

해결:
1. 새 API로 마이그레이션 (권장)
   from isaacsim.core.api import World

2. 경고 무시 (임시)
   import warnings
   warnings.filterwarnings('ignore', category=DeprecationWarning)
```

### Issue 3: PPO GPU 경고
```
경고: "You are trying to run PPO on the GPU, but it is primarily 
intended to run on the CPU when not using a CNN policy"

원인: MlpPolicy는 CPU 최적화, GPU 효율 낮음

해결:
1. CPU 명시 (권장)
   model = PPO('MlpPolicy', env, device='cpu')

2. 경고 무시
   성능 큰 차이 없으면 무시 가능
```

### Issue 4: URDF 조인트 인식 오류
```
증상: "Joints (8): ['joint_1', 'joint_2', 'joint_3']..."
      8개 중 3개만 출력

가능한 원인:
1. get_joint_names() 메서드 버그
2. Articulation 초기화 타이밍 이슈
3. Fixed joint는 제외됨

해결: 
- 확인 중 (현재 디버깅 대상)
```

---

## 🔄 버전 업그레이드 체크리스트

### Isaac Sim 업그레이드 시
```
[ ] Release Notes 정독
[ ] Breaking Changes 확인
[ ] Migration Script 실행
[ ] URDF 재검증
[ ] 환경 테스트 실행
[ ] 학습 재현성 확인
[ ] 문서 업데이트
```

### Python 라이브러리 업그레이드 시
```
[ ] CHANGELOG 확인
[ ] 호환성 매트릭스 확인
[ ] 의존성 충돌 검사
[ ] 단위 테스트 실행
[ ] 통합 테스트 실행
```

---

## 📋 코딩 전 체크리스트 (요약)

```bash
#!/bin/bash
# pre_coding_check.sh

echo "🔍 코딩 전 체크리스트"
echo "=" 60

# 1. Isaac Sim 버전 확인
echo "1. Isaac Sim 버전 확인"
~/isaacsim/python.sh -c "import sys; print(f'Python: {sys.version}')"

# 2. 주요 라이브러리 버전 확인
echo "2. 라이브러리 버전"
~/isaacsim/python.sh -c "
import numpy as np
import stable_baselines3 as sb3
import gymnasium as gym
print(f'NumPy: {np.__version__}')
print(f'SB3: {sb3.__version__}')
print(f'Gymnasium: {gym.__version__}')
"

# 3. 알려진 이슈 확인
echo "3. docs/TROUBLESHOOTING.md 확인 완료? (Y/N)"
read answer

# 4. API 문서 확인
echo "4. API 공식 문서 확인 완료? (Y/N)"
read answer

# 5. GitHub Issues 검색
echo "5. GitHub Issues 검색 완료? (Y/N)"
read answer

echo "✅ 체크리스트 완료!"
```

---

## 📝 버전 이력

| Date | Isaac Sim | Python | Notes |
|------|-----------|--------|-------|
| 2025-10-20 | 5.0.0 | 3.11 | NumPy 2.x 호환성 이슈 확인 |
| 2025-10-19 | 5.0.0 | 3.11 | 프로젝트 시작 |

---

**Last Updated**: 2025-10-20  
**Maintainer**: RoArm Isaac Lab Team

## 🔗 Quick Links

- [Isaac Sim 5.0 Release Notes](https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html)
- [Isaac Lab Migration Guide](https://isaac-sim.github.io/IsaacLab/migration/migrating_from_isaacgymenvs.html)
- [Stable-Baselines3 Changelog](https://github.com/DLR-RM/stable-baselines3/releases)
- [NumPy 2.0 Migration Guide](https://numpy.org/devdocs/numpy_2_0_migration_guide.html)
