# Isaac Sim 버전 전환 가이드

## 🔴 현재 문제
- **Isaac Sim 5.0.0-rc.45** 사용 중
- Warp CUDA 초기화 실패 (`cuDeviceGetUuid` 오류)
- RTX 5090 + Driver 580.95.05 (CUDA 13.0) 호환성 문제
- 모든 환경 테스트가 멈춤

## ✅ 해결 방법: Isaac Sim 4.2.0 LTS로 전환

### 옵션 1: Omniverse Launcher 사용 (권장)

1. **Omniverse Launcher 실행**
```bash
# Launcher 찾기
find ~ -name "*omniverse*launcher*" -type f 2>/dev/null

# 또는 시스템 메뉴에서 "Omniverse Launcher" 실행
```

2. **Isaac Sim 4.2.0 설치**
   - Launcher > Exchange 탭
   - "Isaac Sim" 검색
   - **버전 4.2.0** 선택하여 설치
   - 설치 위치: `~/.local/share/ov/pkg/isaac-sim-4.2.0/`

3. **심볼릭 링크 변경**
```bash
# 기존 링크 백업
mv ~/isaacsim ~/isaacsim_5.0_rc_backup

# Isaac Sim 4.2.0으로 링크 생성
ln -s ~/.local/share/ov/pkg/isaac-sim-4.2.0 ~/isaacsim

# 버전 확인
~/isaacsim/python.sh --version
cat ~/isaacsim/VERSION
```

### 옵션 2: 직접 다운로드 (Omniverse Launcher 없는 경우)

1. **NGC에서 다운로드**
```bash
# Isaac Sim 4.2.0 다운로드 (약 15GB)
wget https://developer.download.nvidia.com/isaac/isaac_sim/isaac-sim-4.2.0-linux-x86_64-release.tar.gz

# 압축 해제
tar -xzf isaac-sim-4.2.0-linux-x86_64-release.tar.gz -C ~/

# 링크 생성
mv ~/isaacsim ~/isaacsim_5.0_rc_backup
ln -s ~/isaac-sim-4.2.0 ~/isaacsim
```

2. **설치 후 초기화**
```bash
cd ~/isaacsim
./setup_python_env.sh
```

### 옵션 3: Docker 사용

```bash
# Isaac Sim 4.2.0 Docker 이미지
docker pull nvcr.io/nvidia/isaac-sim:4.2.0

# 컨테이너 실행
docker run --name isaac-sim \
  --entrypoint bash \
  --gpus all \
  --rm -it \
  -v ~/roarm_isaac_clean:/workspace \
  nvcr.io/nvidia/isaac-sim:4.2.0

# 컨테이너 내에서 테스트
cd /workspace
./isaac-sim.headless.native.sh
python scripts/test/test_vision_env.py --headless
```

## 🔧 전환 후 확인 사항

### 1. 버전 확인
```bash
cat ~/isaacsim/VERSION
# 예상 출력: 4.2.0 또는 2024.1.0
```

### 2. Python 환경 확인
```bash
~/isaacsim/python.sh --version
~/isaacsim/python.sh -c "import torch; print(f'PyTorch: {torch.__version__}')"
~/isaacsim/python.sh -c "import omni; print('Isaac Sim imports OK')"
```

### 3. 기본 테스트 실행
```bash
cd ~/roarm_isaac_clean

# 간단한 Isaac Sim 부팅 테스트
~/isaacsim/python.sh scripts/test/test_isaac_basic.py --headless

# Vision 환경 테스트
~/isaacsim/python.sh scripts/test/test_vision_env.py --headless --num_envs 1
```

### 4. 코드 호환성 확인

Isaac Sim 4.2.0은 5.0과 API가 약간 다를 수 있습니다:

**주요 차이점:**
- `isaaclab` → `omni.isaac.lab` (패키지 이름 변경 전)
- 일부 RigidObject API 차이
- Camera 센서 API 차이

**필요시 코드 수정:**
```python
# Isaac Sim 5.0 (현재)
from isaaclab.app import AppLauncher
from isaaclab.sim import SimulationCfg

# Isaac Sim 4.2.0 (전환 후)
from omni.isaac.kit import SimulationApp
# 또는
from omni.isaac.lab.app import AppLauncher  # IsaacLab이 있다면
```

## 📝 IsaacLab 호환성

현재 코드는 **IsaacLab (Isaac Lab 1.x)**을 사용 중입니다.

### IsaacLab 버전 확인
```bash
~/isaacsim/python.sh -c "import isaaclab; print(isaaclab.__version__)"
```

### IsaacLab 설치 (필요시)
```bash
# IsaacLab 클론
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# Isaac Sim 4.2.0과 호환되는 브랜치 체크아웃
git checkout v1.0.0  # 또는 Isaac Sim 4.2.0 호환 버전

# 설치
~/isaacsim/python.sh -m pip install -e .
```

## ⚠️ 주의사항

1. **기존 5.0 RC 백업**
   - `~/isaacsim_5.0_rc_backup`에 백업됨
   - 필요시 복원 가능

2. **Python 환경 재설정**
   - `~/isaacsim-venv` 삭제 후 재생성 권장
   ```bash
   rm -rf ~/isaacsim-venv
   cd ~/isaacsim
   ./setup_python_env.sh
   ```

3. **캐시 정리**
   ```bash
   rm -rf ~/.cache/ov/Kit
   rm -rf ~/.cache/nvidia/ComputeCache
   ~/isaacsim/clear_caches.sh
   ```

## 🎯 전환 완료 후 다음 단계

1. ✅ Isaac Sim 4.2.0 설치 확인
2. ✅ 환경 테스트 성공
3. ✅ Vision RL 코드 호환성 확인
4. ✅ SAC 학습 시작!

## 🔍 트러블슈팅

### 문제: "No module named 'isaaclab'"
**해결:** IsaacLab 설치 필요 (위 참조)

### 문제: API 호환성 오류
**해결:** 
- Isaac Sim 4.2.0 문서 참조
- 코드 마이그레이션 필요

### 문제: CUDA 오류 지속
**해결:**
- Driver 다운그레이드 고려 (560.x)
- 또는 Docker 사용

---

**현재 상태:** Isaac Sim 5.0.0-rc.45 (문제 있음)  
**목표 상태:** Isaac Sim 4.2.0 LTS (안정)  
**예상 소요 시간:** 30분 ~ 1시간 (다운로드 속도 따라)
