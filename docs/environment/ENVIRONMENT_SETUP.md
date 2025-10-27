# RoArm M3 환경 설정 가이드

**목적**: Isaac Sim + 강화학습 환경 충돌 제거 및 재현 가능한 실행 환경 구축

**문제**: Python 버전 충돌, 경로 문제, 환경 변수 누락으로 인한 반복적인 오류

**해결**: 격리된 환경, 자동 검증, 통합 실행 스크립트

---

## 🎯 핵심 원칙

1. **환경 격리**: Isaac Sim (Python 3.11) vs RL 분석 (Python 3.12)
2. **재현성**: lock 파일, 환경 변수, preflight 검증
3. **통합 실행**: Makefile, VS Code tasks를 통한 원클릭 실행
4. **일일 점검**: 매일 `make preflight` 실행으로 환경 검증

---

## 📁 디렉토리 구조

```
roarm_isaac_clean/
├── envs/
│   ├── isaacsim-venv/          # Python 3.11 (Isaac Sim 전용)
│   └── rl-venv/                # Python 3.12 (RL 훈련/분석)
├── scripts/
│   ├── env_setup/
│   │   ├── preflight.sh        ⭐ 환경 검증 (매일 실행)
│   │   ├── setup_isaac_venv.sh
│   │   ├── setup_rl_venv.sh
│   │   └── validate_gpu.sh
│   ├── isaac/
│   │   ├── run_isaac_gui.sh
│   │   └── run_isaac_headless.sh
│   └── rl/
│       ├── train_rl.sh
│       ├── play_policy.sh
│       └── diagnose_env.sh     (기존)
├── requirements/
│   ├── rl.in                   # RL 의존성 (소스)
│   ├── rl.txt                  # RL lock file (커밋)
│   └── isaac.txt               # Isaac Sim 의존성
├── configs/
│   ├── task_cfg.yaml
│   └── env_cfg.yaml
├── .env                        # 환경 변수 (커밋)
├── .env.local                  # 로컬 설정 (.gitignore)
├── Makefile                    ⭐ 통합 실행 명령
├── .vscode/
│   └── tasks.json              # VS Code 통합
└── docs/
    └── environment/
        ├── ENVIRONMENT_SETUP.md     (이 문서)
        ├── TROUBLESHOOTING.md
        └── DAILY_CHECKLIST.md  ⭐ 매일 확인 체크리스트
```

---

## 🚀 빠른 시작

### 1. 초기 설정 (한 번만 실행)

```bash
# 1. 저장소 클론 후
cd ~/roarm_isaac_clean

# 2. 환경 변수 설정
cp .env.example .env
nano .env  # 경로 수정

# 3. Python 가상환경 생성
make setup-all

# 4. 환경 검증
make preflight
```

### 2. 매일 워밍업 루틴 (⭐ 중요!)

```bash
# 터미널을 열면 제일 먼저 실행
cd ~/roarm_isaac_clean
make preflight

# ✅ 모든 체크 통과 → 작업 시작
# ❌ 실패 → docs/environment/TROUBLESHOOTING.md 참조
```

### 3. 일반 작업 흐름

```bash
# Isaac Sim GUI 테스트
make isaac-gui

# 강화학습 훈련 (10K 빠른 테스트)
make train-quick

# 강화학습 훈련 (50K)
make train

# 학습된 모델 재생
make play

# 환경 진단
make diagnose
```

---

## 📋 환경 변수 (.env)

### 필수 설정

```bash
# Isaac Sim 경로
ISAAC_SIM_ROOT=/home/roarm_m3/isaacsim

# GPU 설정
VULKAN_DEVICE=0
KIT_USE_EGL=1

# 프로젝트 경로
PROJECT_ROOT=/home/roarm_m3/roarm_isaac_clean
ASSETS_ROOT=${PROJECT_ROOT}/assets
DATA_ROOT=${PROJECT_ROOT}/data
LOGS_ROOT=${PROJECT_ROOT}/logs

# Python 환경
ISAAC_PYTHON=${ISAAC_SIM_ROOT}/python.sh
RL_VENV=${PROJECT_ROOT}/envs/rl-venv

# 실행 설정
PYTHONUNBUFFERED=1
WANDB_DISABLED=true

# Isaac Sim 설정
ISAAC_HEADLESS=true
ISAAC_WIDTH=640
ISAAC_HEIGHT=480
```

### 로컬 설정 (.env.local) - 선택사항

```bash
# 개인 설정 (커밋하지 않음)
WANDB_API_KEY=your_key_here
WANDB_PROJECT=roarm_m3_rl
WANDB_DISABLED=false

# GPU 선택 (멀티 GPU 시스템)
CUDA_VISIBLE_DEVICES=0
```

---

## 🐍 Python 환경

### 1. Isaac Sim 환경 (Python 3.11)

**사용 목적**: Isaac Sim과 직접 상호작용하는 스크립트

```bash
# 생성
python3.11 -m venv envs/isaacsim-venv

# 활성화
source envs/isaacsim-venv/bin/activate

# 설치
pip install -r requirements/isaac.txt

# ⚠️ 실제로는 Isaac Sim 내장 Python 사용
# ~/isaacsim/python.sh
```

### 2. RL 환경 (Python 3.12)

**사용 목적**: 데이터 분석, 시각화, 별도 RL 실험

```bash
# 생성
python3.12 -m venv envs/rl-venv

# 활성화
source envs/rl-venv/bin/activate

# 설치
pip install -r requirements/rl.txt
```

### 3. 의존성 관리

```bash
# requirements/rl.in 수정 후
make lock

# 생성된 rl.txt 확인 및 커밋
git add requirements/rl.txt
git commit -m "Update RL dependencies"
```

---

## 🔍 Preflight 검증 (scripts/env_setup/preflight.sh)

### 검증 항목

1. ✅ **GPU 확인**: `nvidia-smi` 작동
2. ✅ **Vulkan 확인**: `vulkaninfo` 작동
3. ✅ **Isaac Sim 경로**: `$ISAAC_SIM_ROOT` 존재
4. ✅ **Python 버전**: Isaac Sim Python 3.11 확인
5. ✅ **환경 변수**: `.env` 로드 확인
6. ✅ **디스크 공간**: 최소 10GB 여유 확인
7. ✅ **메모리**: 최소 16GB RAM 확인

### 출력 예시

```
=== RoArm M3 Environment Preflight Check ===

✅ GPU: NVIDIA GeForce RTX 5090 (Driver 550.90.07)
✅ Vulkan: Version 1.3.275
✅ Isaac Sim: /home/roarm_m3/isaacsim (found)
✅ Isaac Python: 3.11.8
✅ Environment variables: 12 loaded from .env
✅ Disk space: 250GB free (sufficient)
✅ Memory: 64GB RAM (sufficient)

=== All checks passed! Ready to work. ===

Next steps:
  - Train RL: make train-quick
  - Isaac GUI: make isaac-gui
  - Diagnose: make diagnose
```

---

## 🛠️ Makefile 통합 명령

### 기본 명령

```makefile
# 환경 검증 (매일 첫 실행)
make preflight

# 초기 설정
make setup-all          # 모든 환경 설정
make setup-isaac        # Isaac Sim 환경만
make setup-rl           # RL 환경만

# Isaac Sim
make isaac-gui          # GUI 모드
make isaac-headless     # Headless 모드

# 강화학습
make train-quick        # 10K steps (빠른 테스트)
make train              # 50K steps
make train-long         # 500K steps
make play               # 학습된 모델 재생
make diagnose           # 환경 진단

# 의존성 관리
make lock               # requirements 업데이트
make freeze             # 현재 설치된 패키지 목록

# 정리
make clean              # 로그/캐시 삭제
make clean-all          # 가상환경 포함 전체 삭제
```

### 고급 명령

```makefile
# TensorBoard
make tensorboard        # 로그 시각화

# 테스트
make test-env           # 환경 단위 테스트
make test-urdf          # URDF 테스트

# 백업
make backup             # 모델/로그 백업

# Docker
make docker-build       # 컨테이너 빌드
make docker-train       # 컨테이너에서 훈련
```

---

## 🖥️ VS Code 통합

### Tasks (.vscode/tasks.json)

**사용법**: `Ctrl+Shift+B` → 작업 선택

**제공 작업**:
1. **Preflight Check** - 환경 검증
2. **Isaac Sim GUI** - GUI 실행
3. **Train RL (Quick)** - 10K 빠른 훈련
4. **Train RL (50K)** - 정규 훈련
5. **Play Policy** - 모델 재생
6. **Diagnose Environment** - 7단계 진단
7. **TensorBoard** - 로그 시각화

### Launch Configurations (.vscode/launch.json)

**디버깅 설정**:
- Isaac Sim 스크립트 디버깅
- RL 훈련 디버깅
- 환경 진단 디버깅

---

## 🐳 Docker 지원 (선택사항)

### Dockerfile

```dockerfile
FROM nvcr.io/nvidia/isaac-sim:2023.1.1

WORKDIR /workspace

# 프로젝트 복사
COPY . .

# 의존성 설치
RUN pip install -r requirements/rl.txt

# 환경 변수
ENV ISAAC_SIM_ROOT=/isaac-sim
ENV PYTHONUNBUFFERED=1

CMD ["bash"]
```

### docker-compose.yml

```yaml
version: '3.8'
services:
  isaac-train:
    build: .
    runtime: nvidia
    volumes:
      - .:/workspace
      - ./data:/data
      - ./logs:/logs
    environment:
      - DISPLAY=$DISPLAY
      - KIT_USE_EGL=1
    command: make train
```

**사용법**:
```bash
# 빌드
make docker-build

# 훈련
make docker-train
```

---

## 📝 일일 체크리스트 (DAILY_CHECKLIST.md)

### 아침 루틴 (5분)

```bash
cd ~/roarm_isaac_clean

# 1. Git 상태 확인
git status
git pull

# 2. 환경 검증 ⭐
make preflight

# 3. 로그 확인
tail -20 /tmp/last_training.log

# 4. 디스크 공간 확인
df -h
```

### 작업 전 체크

- [ ] `make preflight` 통과
- [ ] `.env` 경로 확인
- [ ] GPU 사용률 확인 (`nvidia-smi`)
- [ ] 이전 프로세스 종료 확인

### 작업 후 체크

- [ ] 로그 파일 백업
- [ ] 모델 파일 커밋
- [ ] 실험 결과 문서화
- [ ] `git push`

---

## 🆘 트러블슈팅

### 문제 1: ModuleNotFoundError

**증상**:
```
ModuleNotFoundError: No module named 'envs.roarm_pick_place_env'
```

**원인**: Python 경로 문제

**해결**:
```bash
# 1. 환경 검증
make preflight

# 2. PYTHONPATH 확인
echo $PYTHONPATH

# 3. 스크립트 수정
export PYTHONPATH=$PROJECT_ROOT:$PYTHONPATH
```

### 문제 2: Isaac Sim Python 충돌

**증상**:
```
AttributeError: _ARRAY_API not found
```

**원인**: 시스템 Python과 Isaac Sim Python 충돌

**해결**:
```bash
# 항상 Isaac Sim Python 사용
~/isaacsim/python.sh your_script.py

# 또는 Makefile 사용
make isaac-headless
```

### 문제 3: GPU 접근 불가

**증상**:
```
RuntimeError: CUDA not available
```

**해결**:
```bash
# 1. GPU 확인
nvidia-smi

# 2. Vulkan 확인
vulkaninfo | head -20

# 3. 환경 변수 확인
echo $CUDA_VISIBLE_DEVICES
echo $VULKAN_DEVICE
```

### 문제 4: 환경 변수 로드 실패

**증상**: 경로를 찾을 수 없음

**해결**:
```bash
# 1. .env 확인
cat .env

# 2. 수동 로드
source .env

# 3. Makefile 사용 (자동 로드)
make train
```

---

## 📚 관련 문서

1. **ENVIRONMENT_SETUP.md** (이 문서) - 환경 설정 완전 가이드
2. **DAILY_CHECKLIST.md** - 매일 확인 체크리스트
3. **TROUBLESHOOTING.md** - 문제 해결 가이드
4. **../rl/DIAGNOSTIC_CHECKLIST.md** - RL 환경 7단계 진단

---

## 🔗 다음 단계

### 1. 초기 설정 (지금 바로!)

```bash
cd ~/roarm_isaac_clean

# 전체 환경 구축
make setup-all

# 검증
make preflight
```

### 2. 매일 루틴

```bash
# 아침에 터미널 열면
make preflight

# ✅ 통과 → 작업 시작
# ❌ 실패 → TROUBLESHOOTING.md 참조
```

### 3. 빠른 테스트

```bash
# 10K steps (10-15분)
make train-quick

# 결과 확인 후 본격 학습 결정
make train
```

---

**작성일**: 2025-10-20  
**최종 업데이트**: 2025-10-20  
**다음 업데이트**: 환경 구축 완료 후 실제 사용 피드백 반영
