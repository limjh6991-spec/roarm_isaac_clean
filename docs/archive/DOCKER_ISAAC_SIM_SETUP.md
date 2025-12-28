# Isaac Sim 4.2.0 Docker 전환 완료

## ✅ 완료된 작업

1. **Isaac Sim 5.0 RC 백업**
   - 위치: `~/isaacsim_5.0_rc_backup`
   - 기존 `~/isaacsim` 제거됨

2. **Docker Compose 파일 생성**
   - 파일: `docker-compose.isaac-sim-4.2.yml`
   - Isaac Sim 4.2.0 공식 이미지 사용

## 🚀 Isaac Sim 4.2.0 Docker 사용 방법

### 1단계: Docker 이미지 다운로드 (약 15GB, 10-20분)

```bash
cd ~/roarm_isaac_clean
docker-compose -f docker-compose.isaac-sim-4.2.yml pull
```

### 2단계: 컨테이너 실행

```bash
# GUI 지원 (X11 forwarding)
xhost +local:docker
docker-compose -f docker-compose.isaac-sim-4.2.yml run --rm isaac-sim
```

### 3단계: 컨테이너 내에서 테스트

```bash
# 컨테이너 내부 프롬프트에서:

# 1. 기본 Isaac Sim 테스트
python scripts/test/test_isaac_basic.py --headless

# 2. Vision 환경 테스트
python scripts/test/test_vision_env.py --headless --num_envs 1

# 3. 전체 preflight 체크
python scripts/test/preflight_vision_rl.py --headless
```

## 📝 코드 호환성 확인 필요

Isaac Sim 4.2.0은 API가 다를 수 있습니다:

### API 차이점

| 항목 | Isaac Sim 5.0 (현재) | Isaac Sim 4.2.0 |
|------|---------------------|-----------------|
| 패키지 이름 | `isaaclab` | `omni.isaac.lab` 또는 없음 |
| AppLauncher | `isaaclab.app.AppLauncher` | `omni.isaac.kit.SimulationApp` |
| RigidObject | `isaaclab.assets.RigidObject` | `omni.isaac.core.objects.RigidPrim` |
| Camera | `isaaclab.sensors.Camera` | `omni.isaac.sensor.Camera` |

### 예상 수정 필요 파일

1. **`envs/simple_vision_env.py`**
   ```python
   # 현재 (Isaac Sim 5.0)
   from isaaclab.app import AppLauncher
   from isaaclab.assets import RigidObject
   
   # 수정 후 (Isaac Sim 4.2.0)
   from omni.isaac.kit import SimulationApp
   from omni.isaac.core.objects import RigidPrim
   ```

2. **`scripts/test/test_vision_env.py`**
   - AppLauncher 초기화 방식 변경

3. **`scripts/train/train_vision_sac.py`**
   - 학습 스크립트 API 호환성 확인

## 🐳 Docker 명령어 치트시트

```bash
# 이미지 다운로드
docker-compose -f docker-compose.isaac-sim-4.2.yml pull

# 컨테이너 실행 (대화형)
docker-compose -f docker-compose.isaac-sim-4.2.yml run --rm isaac-sim

# 백그라운드 실행
docker-compose -f docker-compose.isaac-sim-4.2.yml up -d

# 실행 중인 컨테이너 접속
docker exec -it isaac-sim-4.2 /bin/bash

# 컨테이너 중지
docker-compose -f docker-compose.isaac-sim-4.2.yml down

# 로그 확인
docker-compose -f docker-compose.isaac-sim-4.2.yml logs -f

# GPU 사용 확인 (컨테이너 내부)
nvidia-smi
```

## 🔧 X11 GUI 지원 (필요시)

```bash
# X11 forwarding 활성화
xhost +local:docker

# 테스트: GUI 모드로 Isaac Sim 실행
python scripts/test/test_vision_env.py  # --headless 없이
```

## ⚠️ 트러블슈팅

### 문제 1: Docker 권한 오류
```bash
sudo usermod -aG docker $USER
newgrp docker
```

### 문제 2: NVIDIA Docker Runtime 없음
```bash
# NVIDIA Container Toolkit 설치
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### 문제 3: X11 디스플레이 오류
```bash
xhost +local:docker
export DISPLAY=:0
```

## 📊 다음 단계

1. ✅ **Docker 이미지 다운로드**
   ```bash
   cd ~/roarm_isaac_clean
   docker-compose -f docker-compose.isaac-sim-4.2.yml pull
   ```

2. ✅ **기본 테스트**
   ```bash
   docker-compose -f docker-compose.isaac-sim-4.2.yml run --rm isaac-sim
   # 컨테이너 내부에서:
   python scripts/test/test_isaac_basic.py --headless
   ```

3. ✅ **코드 호환성 확인**
   - API 차이점 확인
   - 필요시 코드 수정

4. ✅ **Vision RL 학습 시작!**
   ```bash
   python scripts/train/train_vision_sac.py --headless --num_envs 8
   ```

---

**현재 상태:** 
- ✅ Isaac Sim 5.0 RC 백업 완료
- ✅ Docker Compose 파일 생성 완료
- ⏳ Isaac Sim 4.2.0 이미지 다운로드 대기 중

**다음 명령:**
```bash
cd ~/roarm_isaac_clean
docker-compose -f docker-compose.isaac-sim-4.2.yml pull
```

이미지 다운로드가 완료되면 즉시 테스트를 시작할 수 있습니다! 🚀
