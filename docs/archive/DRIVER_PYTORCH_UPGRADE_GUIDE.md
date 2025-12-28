# RTX 5090 드라이버 및 PyTorch 업그레이드 가이드

**날짜**: 2025-11-11  
**목적**: RTX 5090 최적 드라이버 및 PyTorch 2.5+ 호환성 정보

---

## 🔧 우분투용 NVIDIA 드라이버

### 현재 설치된 드라이버
```bash
Driver Version: 580.95.05
CUDA Version: 13.0
```

### RTX 5090 권장 드라이버 (Blackwell 아키텍처)

#### 최신 Production 드라이버
```
버전: 565.57.01 (2024-11 기준)
CUDA: 12.7
지원: Blackwell (RTX 50 series) 완전 지원
다운로드: https://www.nvidia.com/Download/driverResults.aspx/228184/
```

#### 설치 명령어
```bash
# 1. 기존 드라이버 제거 (선택사항)
sudo apt-get purge nvidia-*
sudo apt-get autoremove

# 2. 저장소 추가
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt-get update

# 3. 권장 드라이버 확인
ubuntu-drivers devices

# 4. 특정 버전 설치 (565 권장)
sudo apt-get install nvidia-driver-565

# 5. 재부팅
sudo reboot

# 6. 확인
nvidia-smi
```

#### ⚠️ 주의사항
- **Docker 컨테이너 재시작 필요**
- **Isaac Sim 호환성 검증 필요**
- **백업 후 진행 권장**

---

## 🐍 PyTorch 2.5+ 정보

### PyTorch 2.5.1 (최신 안정 버전)

#### 주요 기능
```
✅ CUDA 12.4/12.1/11.8 지원
✅ sm_120 (Blackwell) 완전 지원
✅ Python 3.8-3.12 호환
✅ cuDNN 9.x 지원
```

#### 설치 명령어

**CUDA 12.4 버전** (RTX 5090 최적):
```bash
pip install torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1 \
  --index-url https://download.pytorch.org/whl/cu124
```

**CUDA 12.1 버전** (현재 환경):
```bash
pip install torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1 \
  --index-url https://download.pytorch.org/whl/cu121
```

**CUDA 11.8 버전** (구버전):
```bash
pip install torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1 \
  --index-url https://download.pytorch.org/whl/cu118
```

#### 검증 스크립트
```python
import torch
print("="*60)
print("PyTorch Version:", torch.__version__)
print("CUDA Available:", torch.cuda.is_available())
print("CUDA Version:", torch.version.cuda)
print("cuDNN Version:", torch.backends.cudnn.version())
print("="*60)

if torch.cuda.is_available():
    print("\nGPU Information:")
    print("  Device:", torch.cuda.get_device_name(0))
    print("  Capability:", torch.cuda.get_device_capability(0))
    print("  Arch List:", torch.cuda.get_arch_list())
    
    print("\nCUDA Computation Test:")
    try:
        x = torch.randn(1000, 1000, device='cuda')
        y = x @ x
        print("  ✅ Matrix multiplication: SUCCESS")
        print(f"  Result shape: {y.shape}")
        print(f"  Device: {y.device}")
    except Exception as e:
        print(f"  ❌ FAILED: {e}")
else:
    print("\n❌ CUDA not available")
```

---

## 🧪 PyTorch 2.5 호환성 테스트 계획

### 테스트 환경 구성

#### Option 1: 별도 Docker 컨테이너
```dockerfile
# Dockerfile.pytorch25
FROM nvidia/cuda:12.4.0-devel-ubuntu22.04

# 기본 패키지
RUN apt-get update && apt-get install -y \
    python3.10 python3-pip git wget \
    && rm -rf /var/lib/apt/lists/*

# PyTorch 2.5.1 설치
RUN pip3 install torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1 \
    --index-url https://download.pytorch.org/whl/cu124

# Isaac Sim 의존성 (일부)
RUN pip3 install numpy gymnasium stable-baselines3

WORKDIR /workspace
```

**빌드 및 실행**:
```bash
# 빌드
docker build -f Dockerfile.pytorch25 -t pytorch25-test:latest .

# 실행
docker run -it --gpus all \
  -v /home/roarm_m3/roarm_isaac_clean:/workspace \
  pytorch25-test:latest bash

# 검증
python3 -c "import torch; print(torch.cuda.is_available())"
```

#### Option 2: Python venv
```bash
# venv 생성
python3.10 -m venv ~/pytorch25_env
source ~/pytorch25_env/bin/activate

# PyTorch 2.5 설치
pip install torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1 \
  --index-url https://download.pytorch.org/whl/cu124

# 테스트
python -c "import torch; print(torch.__version__)"
```

### 호환성 체크리스트

- [ ] **PyTorch 2.5 설치 확인**
  ```bash
  python -c "import torch; print(torch.__version__)"
  # 기대값: 2.5.1
  ```

- [ ] **CUDA sm_120 지원 확인**
  ```bash
  python -c "import torch; print(torch.cuda.get_arch_list())"
  # 기대값: [..., 'sm_90', 'sm_120'] 포함
  ```

- [ ] **GPU 연산 테스트**
  ```python
  import torch
  x = torch.randn(1000, 1000, device='cuda')
  y = x @ x
  print("✅ SUCCESS" if y.device.type == 'cuda' else "❌ FAILED")
  ```

- [ ] **stable-baselines3 호환성**
  ```python
  from stable_baselines3 import SAC
  import gymnasium as gym
  env = gym.make('Pendulum-v1')
  model = SAC('MlpPolicy', env, device='cuda')
  print("✅ SB3 CUDA OK")
  ```

- [ ] **Isaac Sim 부팅 테스트** (고위험)
  ```bash
  /isaac-sim/python.sh -c "from isaacsim import SimulationApp; app = SimulationApp({'headless': True}); app.close()"
  ```

- [ ] **의존성 충돌 기록**
  ```bash
  pip list | grep -E "(torch|nvidia|cuda)"
  ```

---

## 📋 Isaac Sim 릴리스 모니터링

### 공식 릴리스 노트
- **URL**: https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html
- **확인 주기**: 주 1회 (매주 월요일)
- **주요 확인 항목**:
  - PyTorch 버전 요구사항
  - CUDA 버전 지원
  - Blackwell/RTX 50 series 언급

### 예상 버전별 타임라인

| 버전 | 예상 릴리스 | PyTorch 지원 | RTX 5090 상태 |
|------|------------|-------------|--------------|
| **4.2.0** | 2024-09 (현재) | 2.2.2 | ❌ sm_120 미지원 |
| **4.3.0** | 2025-Q1 (예상) | 2.3.x? | ⚠️ 불확실 |
| **5.0.0** | 2025-Q2 (예상) | 2.4/2.5? | ✅ 지원 가능성 높음 |

### 모니터링 스크립트
```bash
#!/bin/bash
# isaac_sim_check.sh

RELEASE_URL="https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html"
CACHE_FILE="/tmp/isaac_sim_release.html"

# 다운로드
curl -s "$RELEASE_URL" > "$CACHE_FILE"

# 버전 확인
VERSION=$(grep -oP 'version.*?([0-9]+\.[0-9]+\.[0-9]+)' "$CACHE_FILE" | head -1)

echo "=== Isaac Sim Release Check ==="
echo "Latest Version: $VERSION"
echo "Date: $(date)"
echo ""

# PyTorch 버전 확인
if grep -qi "pytorch.*2\.[4-9]" "$CACHE_FILE"; then
    echo "✅ PyTorch 2.4+ 지원 가능성!"
elif grep -qi "pytorch.*2\.2" "$CACHE_FILE"; then
    echo "⚠️ 여전히 PyTorch 2.2.x"
fi

# RTX 5090/Blackwell 언급
if grep -qi "blackwell\|rtx.*50\|sm_120" "$CACHE_FILE"; then
    echo "✅ RTX 5090 관련 언급 발견!"
fi
```

### 커뮤니티 모니터링

#### NVIDIA Developer Forums
- **URL**: https://forums.developer.nvidia.com/c/omniverse/simulation/69
- **검색 키워드**: 
  - "RTX 5090"
  - "Blackwell"
  - "PyTorch 2.5"
  - "sm_120"
  - "CUDA capability"

#### GitHub Issues
```bash
# Isaac Gym Envs
https://github.com/NVIDIA-Omniverse/IsaacGymEnvs/issues

# Isaac Lab
https://github.com/isaac-sim/IsaacLab/issues

# 검색 쿼리
is:issue state:open "RTX 5090" OR "sm_120" OR "PyTorch 2.5"
```

---

## 🎯 Action Items (우선순위별)

### 🔴 HIGH (이번 주)

1. **드라이버 업데이트 검토**
   ```bash
   # 현재 확인
   nvidia-smi
   
   # 최신 드라이버 정보
   ubuntu-drivers devices
   
   # 결정: 580 → 565 업그레이드 여부
   ```

2. **CPU 모드 SAC 테스트**
   ```python
   # train_vision_sac.py 수정
   model = SAC(..., device="cpu")
   ```

3. **PyTorch 2.5 별도 테스트**
   ```bash
   # Docker 컨테이너 생성
   docker build -f Dockerfile.pytorch25 -t pytorch25-test .
   docker run -it --gpus all pytorch25-test bash
   ```

### 🟡 MEDIUM (이번 달)

4. **Isaac Sim 4.3 베타 신청**
   - NVIDIA Developer Program 가입
   - Early Access 신청

5. **대안 프레임워크 조사**
   - MuJoCo + Stable-Baselines3
   - PyBullet + GPU physics
   - Brax (JAX 기반)

### 🟢 LOW (지속적)

6. **릴리스 노트 모니터링**
   - 매주 월요일 체크
   - 자동화 스크립트 cron 등록

7. **커뮤니티 참여**
   - 포럼 구독
   - 이슈 트래킹

---

## 📞 리소스 링크 모음

### 드라이버 다운로드
- **NVIDIA 드라이버**: https://www.nvidia.com/Download/index.aspx
- **CUDA Toolkit**: https://developer.nvidia.com/cuda-downloads
- **cuDNN**: https://developer.nvidia.com/cudnn

### PyTorch
- **공식 사이트**: https://pytorch.org/
- **설치 가이드**: https://pytorch.org/get-started/locally/
- **릴리스 노트**: https://github.com/pytorch/pytorch/releases
- **아키텍처 지원**: https://pytorch.org/docs/stable/cpp_extension.html#torch.utils.cpp_extension.CUDAExtension

### Isaac Sim
- **문서**: https://docs.omniverse.nvidia.com/isaacsim/latest/
- **릴리스 노트**: https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html
- **포럼**: https://forums.developer.nvidia.com/c/omniverse/simulation/69
- **GitHub**: https://github.com/NVIDIA-Omniverse/IsaacGymEnvs

### 커뮤니티
- **PyTorch Forums**: https://discuss.pytorch.org/
- **NVIDIA Discord**: https://discord.gg/nvidia
- **Reddit r/MachineLearning**: https://reddit.com/r/MachineLearning

---

**최종 업데이트**: 2025-11-11  
**다음 체크**: 2025-11-18 (1주일 후)
