# 환경 상태 및 설정 정보

**작성일**: 2025년 10월 15일  
**출처**: 이전 프로젝트 분석 및 시스템 검증  
**목적**: 현재 시스템 환경의 정확한 상태 파악

---

## 💻 하드웨어 환경

### CPU
```
모델: AMD Ryzen 7 9800X3D
코어: 8 Physical Cores / 16 Logical Cores
아키텍처: x86_64
```

### GPU
```
주 GPU: NVIDIA GeForce RTX 5090
VRAM: 32GB
드라이버: 580.95.05 (최신)
CUDA: 12.8
컴퓨트 능력: sm_120 (Blackwell 아키텍처)

보조 GPU: AMD Radeon Graphics (RADV RAPHAEL, iGPU)
VRAM: 10745 MB
```

### 메모리
```
총 RAM: 31GB
사용 가능: ~24GB
Swap: 8GB
```

---

## 🐧 소프트웨어 환경

### 운영체제
```
배포판: Ubuntu 24.04.3 LTS (Noble Numbat)
커널: 6.14.0-33-generic
X Server: 1.21.1.11 (The X.Org Foundation)
```

### Python 환경

#### 시스템 Python
```
버전: Python 3.12.3
경로: /usr/bin/python3
패키지 관리: apt, pip
```

#### Isaac Sim 전용 venv
```
버전: Python 3.11 (Isaac Sim 번들)
경로: ~/isaacsim-venv/
설치 방법: python3.11 -m venv ~/isaacsim-venv
패키지: isaacsim[all,extscache]==5.0.0
```

**활성화 방법:**
```bash
source ~/isaacsim-venv/bin/activate
# 또는
source scripts/activate_isaacsim_env.sh
```

---

## 🎮 Isaac Sim 환경

### 설치 정보
```
버전: Isaac Sim 5.0
설치 방법: pip install (추천)
원본 파일: isaac-sim-standalone-5.0.0-linux-x86_64.zip
설치 경로: ~/isaacsim-venv/lib/python3.11/site-packages/isaacsim/
```

### PhysX 정보
```
버전: PhysX 107.3.18
Solver: TGS (Temporal Gauss-Seidel)
타임스텝: 1/120 Hz (0.00833s)
GPU 가속: 활성화 (Device 0: RTX 5090)
```

### 주요 환경변수
```bash
# Isaac Sim 활성화 필수 변수
OMNI_KIT_ACCEPT_EULA=yes
PYTHONPATH=${ISAAC_SIM_PATH}/...
LD_LIBRARY_PATH=${ISAAC_SIM_PATH}/...

# Headless 모드 (선택)
ENABLE_HEADLESS=1
KIT_USE_EGL=1
CARB_APP_QUIET_SHUTDOWN=1

# 오디오 비활성화 (Headless 시)
CARB_AUDIO_DISABLED=1
CARB_ENABLE_AUDIO=0
```

---

## 🔧 개발 도구

### VS Code
```
버전: 1.104.3
확장:
  - GitHub Copilot (최신)
  - Python (최신)
  - Pylance (최신)
```

### Git
```
버전: 설치됨
설정: Git LFS 지원 (대형 파일용)
```

---

## 🐍 Python 패키지 환경

### RL 환경 (Python 3.12)
```
stable-baselines3==2.4.0
gymnasium==0.29.1
torch==2.10.0.dev+cu128 (나이틀리 빌드)
numpy
omegaconf (선택)
wandb (선택)
```

### Isaac 환경 (Python 3.11)
```
isaacsim[all,extscache]==5.0.0
pxr (USD Python 바인딩, 포함됨)
omni.isaac.* (Isaac Sim 확장, 포함됨)
```

---

## ⚙️ 알려진 설정 이슈

### Issue 1: Python 버전 충돌
**문제:**
- Isaac Sim 5.0은 Python 3.11 전용
- 시스템 Python은 3.12
- 단일 환경 통합 불가능 (SRE module mismatch)

**해결책:**
- ✅ Dual Environment 전략 채택
- Python 3.11 venv (Isaac 전용)
- Python 3.12 (RL/분석 전용)
- IPC로 통신 (TCP JSON)

### Issue 2: PyTorch CUDA 버전 불일치
**문제:**
- Isaac Sim은 CUDA 12.8 사용
- 기존 PyTorch는 CUDA 11.8 기반

**해결책:**
- ✅ PyTorch 나이틀리 빌드 설치 (2.10.0.dev+cu128)
- GPU 학습 가능
- 장기적으로 stable 빌드 전환 필요

### Issue 3: 원격 GUI 렌더링
**문제:**
- X11 포워딩으로는 Isaac Sim GUI 렌더링 불가
- OpenGL/Vulkan 컨텍스트 스트리밍 실패

**해결책:**
- ⚠️ WebRTC 스트리밍 설정 필요
- 또는 NICE DCV / TurboVNC 사용
- Headless 모드 + 메트릭 수집 기본 전략

### Issue 4: Extension 관리
**문제:**
- GUI/Replicator/Audio 확장이 Headless 모드에서도 활성화됨
- 메모리/성능 오버헤드

**해결책:**
- ✅ 확장 블록리스트 관리
- TOML override 파일 생성
- 런처 플래그 (`--disable`, `--enableOnly`)

---

## 📁 디렉토리 구조

### Isaac Sim 관련 경로
```
~/isaacsim-venv/                          # Isaac Sim Python 환경
~/.local/share/ov/data/Kit/Isaac-Sim Full/5.0/
  ├── exts/user/                          # 사용자 확장 override
  └── cache/                              # 캐시 파일
```

### 프로젝트 경로
```
~/codex_mcp/                              # 이전 프로젝트 (참조용)
~/roarm_isaac_clean/                      # 새 프로젝트 (현재)
```

---

## 🔍 확인 명령어

### Python 환경 확인
```bash
# 시스템 Python
python3 --version
which python3

# Isaac Sim Python
source ~/isaacsim-venv/bin/activate
python --version
python -c "import pxr; print(pxr.__version__)"
python -c "import isaacsim; print(isaacsim.__version__)"
```

### GPU 상태 확인
```bash
nvidia-smi
nvcc --version
```

### Isaac Sim 환경 점검
```bash
# 이전 프로젝트 스크립트 참조
cd ~/codex_mcp
source scripts/activate_isaacsim_env.sh
bash scripts/isaac_precheck.sh
python scripts/check_isaac_import.py
```

### CUDA 확인
```bash
# PyTorch CUDA 사용 가능 여부
python -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"
python -c "import torch; print(f'Device: {torch.cuda.get_device_name(0)}')"
```

---

## ⚠️ 주의사항

### 환경 활성화
- ✅ **Isaac 관련 작업**: 항상 `~/isaacsim-venv` 활성화
- ✅ **RL 학습/분석**: 별도 venv 또는 시스템 Python 사용
- ⚠️ **혼용 금지**: 두 환경을 동시에 사용하지 말 것

### Import 순서
```python
# Isaac 환경에서
import isaacsim  # 먼저
from isaacsim import SimulationApp
from isaacsim.core.prims import SingleArticulation  # 최신 API

# 구형 API 사용 금지
# from omni.isaac.core.articulations import Articulation  # ❌
```

### 파일 경로
- ✅ **절대 경로 사용** 권장
- ✅ **USD 파일 경로** 확인 필수
- ✅ **Mesh 경로** 상대 경로 설정 검증

---

## 📊 성능 기준

### 하드웨어 성능
```
CPU: 충분 (16 threads)
GPU: 최상급 (RTX 5090, 32GB VRAM)
RAM: 충분 (31GB)
스토리지: 확인 필요 (SSD 권장)
```

### 예상 성능
```
Isaac Sim 렌더링: 60+ FPS (GUI 모드)
PhysX 시뮬레이션: 120 Hz (안정)
RL 학습 속도: 매우 빠름 (GPU 가속)
IPC Latency: < 10ms (로컬, 최적화 시)
```

---

## 🎯 환경 준비 체크리스트

### 필수 항목
- [x] Ubuntu 24.04 설치
- [x] NVIDIA 드라이버 설치 (580.95.05)
- [x] CUDA 12.8 설치
- [x] Python 3.11 설치
- [x] Python 3.12 설치 (시스템)
- [x] Isaac Sim 5.0 설치 (pip)
- [ ] Isaac Sim 5.0 설치 검증
- [ ] 샘플 USD 파일 로딩 테스트
- [ ] PhysX 시뮬레이션 테스트

### 선택 항목
- [ ] WebRTC 스트리밍 설정
- [ ] NICE DCV 설치
- [ ] VS Code Remote SSH 설정
- [ ] Git LFS 설정
- [ ] Weights & Biases 계정

---

## 🔗 참고 자료

### 공식 문서
- [Isaac Sim 5.0 Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Isaac Sim Python API](https://docs.omniverse.nvidia.com/py/isaacsim/index.html)
- [USD Documentation](https://graphics.pixar.com/usd/docs/index.html)
- [PhysX Documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/)

### 커뮤니티
- [NVIDIA Isaac Sim Forums](https://forums.developer.nvidia.com/c/isaac-sim/)
- [Reddit r/IsaacSim](https://www.reddit.com/r/IsaacSim/)
- [GitHub isaac-sim Issues](https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles/issues)

---

**마지막 업데이트**: 2025년 10월 15일  
**다음 업데이트**: Isaac Sim 설치 검증 후
