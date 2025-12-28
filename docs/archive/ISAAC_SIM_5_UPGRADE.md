# Isaac Sim 5.x 업그레이드 가이드

**날짜**: 2025-11-11  
**목표**: Isaac Sim 2023.1.1 → 5.x 정식 버전

---

## 📊 현재 상태

```
✅ 드라이버: 580.95.05 (CUDA 13.0)
✅ CUDA 툴킷: 12.6.85
✅ PyTorch: 2.8.0+cu128 (sm_120 지원)
✅ Docker: 28.2.2
✅ NVIDIA Docker: 정상 작동
⚠️ Isaac Sim: 2023.1.1 (구버전)
```

---

## 🚀 방법 1: Isaac Lab Docker (권장 - 즉시 사용)

Isaac Lab은 Isaac Sim 5.x를 포함하며 RTX 5090을 완벽 지원합니다.

### 설치 및 실행

```bash
# 1. Isaac Lab 최신 이미지 다운로드 (약 10-15GB)
docker pull isaac-sim.nvidia.com/isaac-lab:latest

# 2. 컨테이너 실행
docker run --rm -it \
  --gpus all \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ACCEPT_EULA=Y \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -v $(pwd):/workspace \
  --name isaac-lab \
  isaac-sim.nvidia.com/isaac-lab:latest

# 3. 컨테이너 내부에서 확인
python -c "from isaacsim import SimulationApp; print('Isaac Sim Ready!')"
```

### Docker Compose 파일 생성

파일: `docker-compose.isaac-lab.yml`

---

## 🔧 방법 2: Isaac Sim 5.x 직접 설치

### 옵션 A: NGC에서 다운로드

```bash
# 1. NGC CLI 설치 (선택사항)
wget -O ngccli_linux.zip https://ngc.nvidia.com/downloads/ngccli_linux.zip
unzip ngccli_linux.zip
chmod +x ngc-cli/ngc

# 2. Isaac Sim 5.0.0 다운로드 (약 15GB)
# NGC 계정 필요: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim

docker pull nvcr.io/nvidia/isaac-sim:4.2.0
# 또는 최신 버전
docker pull nvcr.io/nvidia/isaac-sim:latest
```

### 옵션 B: Omniverse Launcher 설치 후 Isaac Sim 설치

```bash
# 1. Omniverse Launcher 다운로드
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# 2. 실행 권한 부여
chmod +x omniverse-launcher-linux.AppImage

# 3. Launcher 실행
./omniverse-launcher-linux.AppImage

# 4. Launcher에서 Isaac Sim 5.x 설치
# Exchange 탭 > Isaac Sim > 버전 5.0 이상 선택
```

---

## 📋 방법별 비교

| 방법 | 설치 시간 | 복잡도 | RTX 5090 지원 | 권장도 |
|------|-----------|--------|---------------|--------|
| Isaac Lab Docker | 30분 | ⭐ 쉬움 | ✅ 완벽 | ⭐⭐⭐⭐⭐ |
| Isaac Sim Docker | 20분 | ⭐⭐ 보통 | ✅ 완벽 | ⭐⭐⭐⭐ |
| Launcher + 설치 | 60분 | ⭐⭐⭐ 복잡 | ✅ 완벽 | ⭐⭐⭐ |

---

## ✅ 추천 방법: Isaac Lab Docker

**이유**:
1. ✅ Isaac Sim 5.x + Isaac Lab 통합
2. ✅ 모든 의존성 사전 해결
3. ✅ RTX 5090 완벽 지원 보장
4. ✅ 격리된 환경으로 시스템 안전
5. ✅ 즉시 사용 가능

**다음 단계**:
```bash
cd /home/roarm_m3/roarm_isaac_clean
bash scripts/install_isaac_lab_docker.sh
```

---

## 🔄 기존 프로젝트 마이그레이션

Isaac Lab Docker 내에서 현재 프로젝트 사용:

```bash
# 컨테이너에서 워크스페이스가 자동 마운트됨
docker run --rm -it --gpus all \
  -v /home/roarm_m3/roarm_isaac_clean:/workspace \
  isaac-sim.nvidia.com/isaac-lab:latest

# 컨테이너 내부
cd /workspace
python envs/simple_vision_env.py
```

---

## 📝 설치 후 확인

```bash
# Python 버전
python --version

# Isaac Sim 버전
python -c "from isaacsim import SimulationApp; app = SimulationApp({'headless': True}); print(f'Isaac Sim Version: {app.version}'); app.close()"

# GPU 확인
nvidia-smi

# PyTorch CUDA
python -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"
```

---

## 🎯 다음 단계

1. ✅ Isaac Lab Docker 이미지 다운로드
2. ✅ 컨테이너 실행 및 테스트
3. ✅ 기존 프로젝트 마운트
4. ✅ Vision RL 학습 재개
