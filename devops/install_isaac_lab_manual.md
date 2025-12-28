# Isaac Lab 수동 설치 가이드 (Isaac Sim 4.2.0)

## 방법: Isaac Sim 내장 Python으로 Isaac Lab 설치

Isaac Lab v1.0.0을 Isaac Sim 4.2.0과 함께 사용하려면 Isaac Sim의 Python 환경에 직접 설치해야 합니다.

### Step 1: Isaac Lab 소스 다운로드

```bash
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout v1.0.0
```

### Step 2: Isaac Sim Python 경로 확인

```bash
# Docker에서 Isaac Sim Python 찾기
docker run --rm --gpus all \
  --env ACCEPT_EULA=Y \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  bash -c "which python"
```

### Step 3: Isaac Lab 소스 복사 및 설치

```bash
# 방법 A: Docker 볼륨 마운트 + pip install
docker run --rm --gpus all \
  --env ACCEPT_EULA=Y \
  -v ~/IsaacLab:/workspace/IsaacLab \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  bash -c "cd /workspace/IsaacLab && pip install -e ."

# 방법 B: Isaac Sim Python으로 직접 설치
docker run --rm --gpus all \
  --env ACCEPT_EULA=Y \
  -v ~/IsaacLab:/workspace/IsaacLab \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  bash -c "cd /workspace/IsaacLab/source && /isaac-sim/python.sh -m pip install -e ."
```

### Step 4: 설치 검증

```bash
docker run --rm --gpus all \
  --env ACCEPT_EULA=Y \
  -v ~/IsaacLab:/workspace/IsaacLab \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  bash -c "python -c 'import isaaclab; print(isaaclab.__version__)'"
```

## 대안: 기존 코드 API 마이그레이션

Isaac Lab 없이 Isaac Sim 4.2.0 기본 API로 코드 수정:

- `isaaclab.app` → `omni.isaac.kit`
- `isaaclab.assets.Articulation` → `omni.isaac.core.articulations.Articulation`
- `isaaclab.sensors.Camera` → `omni.isaac.core.cameras.Camera`
- `isaaclab.sim` → `omni.isaac.core.simulation_context`

**권장**: 코드 마이그레이션보다 Isaac Lab 설치가 더 빠르고 안전합니다.

## 다음 단계

설치 완료 후:
1. Docker 이미지로 커밋: `docker commit <container_id> roarm-isaac-lab:1.0.0`
2. 환경 테스트: `python scripts/test/test_vision_env.py`
3. SAC 학습 시작: `python scripts/train/train_vision_sac.py`
