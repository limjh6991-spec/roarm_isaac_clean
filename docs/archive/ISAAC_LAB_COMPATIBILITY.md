# Isaac Lab + Isaac Sim 호환성 가이드

## 🔍 문제 분석

### 현재 상황
- 프로젝트: `isaaclab` API 사용 (Isaac Lab 1.x 기반)
- 시도한 버전들:
  - ❌ Isaac Sim 5.0.0-rc.45: Warp CUDA 초기화 실패
  - ✅ Isaac Sim 4.2.0: 정상 작동하지만 `isaaclab` 미포함

### Isaac Lab vs Isaac Sim 버전 호환성

| Isaac Lab 버전 | 호환 Isaac Sim 버전 | 비고 |
|----------------|---------------------|------|
| Isaac Lab 1.0.0 | Isaac Sim 4.0.0 - 4.2.0 | 안정 버전 |
| Isaac Lab 1.1.0 | Isaac Sim 2024.1.0 (5.0) | 최신 안정 |
| Isaac Lab 1.2.0+ | Isaac Sim 2025.x | 개발 중 |

## ✅ 권장 해결책: Isaac Lab Docker 이미지 사용

NVIDIA에서 제공하는 **Isaac Lab 공식 Docker 이미지**를 사용하면:
- ✅ Isaac Sim + Isaac Lab 사전 통합
- ✅ 모든 의존성 해결됨
- ✅ GPU 가속 완벽 지원

### Isaac Lab Docker 설정

```bash
# Isaac Lab 1.1.0 + Isaac Sim 2024.1.0
docker pull isaac-sim.nvidia.com/isaac-lab:1.1.0

# 또는 최신 버전
docker pull isaac-sim.nvidia.com/isaac-lab:latest
```

### Docker Compose 파일

```yaml
version: '3.8'

services:
  isaac-lab:
    image: isaac-sim.nvidia.com/isaac-lab:1.1.0
    container_name: isaac-lab
    runtime: nvidia
    environment:
      - ACCEPT_EULA=Y
      - DISPLAY=${DISPLAY}
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - ./:/workspace
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ~/.Xauthority:/root/.Xauthority:rw
    working_dir: /workspace
    stdin_open: true
    tty: true
    network_mode: host
    command: /bin/bash
```

## 🔧 대안 1: Isaac Lab을 Isaac Sim 4.2.0에 수동 설치

### 설치 단계

```bash
# Isaac Sim 4.2.0 Docker 컨테이너 실행
docker run -it --rm \
    --runtime=nvidia --gpus all \
    -e ACCEPT_EULA=Y \
    -v ~/roarm_isaac_clean:/workspace \
    nvcr.io/nvidia/isaac-sim:4.2.0 \
    /bin/bash

# 컨테이너 내부에서:
cd /workspace

# Isaac Lab 클론
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# Isaac Sim 4.2.0 호환 버전 체크아웃
git checkout v1.0.0

# 설치
./isaaclab.sh --install

# 테스트
./isaaclab.sh -p scripts/test/test_vision_env.py --headless
```

## 🔧 대안 2: isaacsim.core.api로 코드 마이그레이션

Isaac Lab 없이 Isaac Sim 5.1.0 native API 사용:

### Before (Isaac Lab):
```python
from omni.isaac.lab.app import AppLauncher
app_launcher = AppLauncher()
simulation_app = app_launcher.app
```

#### After (isaacsim.core.api):
```python
from isaacsim import SimulationApp
from isaacsim.core.api import World
import omni.isaac.core.utils.prims as prim_utils
from isaacsim.sensors.camera import Camera
```

### API 비교

| 기능 | Isaac Lab | isaacsim.core.api |

## 🎯 최종 권장 사항

### 옵션 A: Isaac Lab Docker (추천) ⭐

**장점**:
- 즉시 사용 가능
- 버전 호환성 보장
- 최소 설정 필요

**단점**:
- 이미지 크기 큼 (~20GB)
- NGC 계정 필요할 수 있음

**실행**:
```bash
cd ~/roarm_isaac_clean

# Isaac Lab Docker 실행
docker run -it --rm \
    --runtime=nvidia --gpus all \
    -e ACCEPT_EULA=Y \
    -v $(pwd):/workspace \
    isaac-sim.nvidia.com/isaac-lab:1.1.0
```

### 옵션 B: Isaac Sim 4.2.0 + 수동 Isaac Lab 설치

**장점**:
- 이미 다운로드됨
- 커스터마이징 가능

**단점**:
- 수동 설정 필요
- 버전 충돌 가능성

### 옵션 C: 코드 마이그레이션

**장점**:
- Isaac Lab 의존성 제거
- 네이티브 API 사용

**단점**:
- 대규모 코드 수정 필요
- 학습 곡선

## 📊 비교표

| 항목 | Isaac Lab Docker | 수동 설치 | 코드 마이그레이션 |
|------|------------------|-----------|-------------------|
| 설치 시간 | 30분 (다운로드) | 10분 | - |
| 코드 수정 | 불필요 | 불필요 | 대규모 |
| 유지보수 | 쉬움 | 보통 | 어려움 |
| 성능 | 최적 | 최적 | 최적 |
| 권장도 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ |

## 🚀 다음 단계

**즉시 실행 가능한 명령**:

```bash
# 1. Isaac Lab Docker 이미지 확인
docker search isaac-lab

# 2. 또는 수동 설치
docker run -it --rm \
    --runtime=nvidia --gpus all \
    -e ACCEPT_EULA=Y \
    -v ~/roarm_isaac_clean:/workspace \
    -w /workspace \
    nvcr.io/nvidia/isaac-sim:4.2.0 \
    bash -c "
    git clone https://github.com/isaac-sim/IsaacLab.git && \
    cd IsaacLab && \
    git checkout v1.0.0 && \
    ./isaaclab.sh --install && \
    cd /workspace && \
    ./IsaacLab/isaaclab.sh -p scripts/test/test_vision_env.py --headless
    "
```

---

**현재 상태**: Isaac Sim 4.2.0 Docker 실행 가능, Isaac Lab 설치 필요  
**목표**: Isaac Lab + Vision RL 학습 실행  
**다음 명령**: Isaac Lab 설치 또는 공식 Docker 이미지 사용
