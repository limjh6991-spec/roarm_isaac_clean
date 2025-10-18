# 개발 인프라 가이드

**작성일**: 2025년 10월 17일  
**목적**: 안정적인 Isaac Sim 개발을 위한 DevOps 인프라

---

## 📋 목차

1. [개요](#개요)
2. [디렉토리 구조](#디렉토리-구조)
3. [Preflight 검사](#preflight-검사)
4. [로그 관리](#로그-관리)
5. [테스트](#테스트)
6. [VS Code 통합](#vs-code-통합)
7. [문제 해결](#문제-해결)

---

## 개요

이전 프로젝트에서 경험한 주요 문제점:
1. **터미널과 함께 죽어서 로그가 없음**
2. **Copilot이 원인을 못 잡음** (로그 확인 부족)
3. **API/Extension 최신 지식 격차**

### 해결 방법

| 문제 | 해결책 | 구현 위치 |
|------|--------|-----------|
| 로그 없이 죽음 | 프로세스 격리 + 강제 로그화 | `devops/run_isaac_supervised.sh` |
| 원인 미파악 | 사전 진단 + 스모크 테스트 | `devops/preflight/` |
| API 격차 | 버전 고정 + 회귀 테스트 | `tests/`, `requirements.txt` |

---

## 디렉토리 구조

```
roarm_isaac_clean/
├── devops/                          # 개발 인프라
│   ├── preflight/                   # 사전 검사 스크립트
│   │   ├── check_system.sh          # GPU, Driver, Vulkan 검사
│   │   ├── check_isaac_extensions.py # Isaac Sim 확장 검사
│   │   └── check_usd_integrity.py   # USD 무결성 검사
│   ├── preflight_all.sh             # 통합 Preflight 실행
│   └── run_isaac_supervised.sh      # Isaac Sim 감시 실행 래퍼
├── logs/                            # 로그 저장소
│   ├── isaac/                       # Isaac Sim 실행 로그
│   │   ├── isaac_YYYYMMDD_HHMMSS.log
│   │   └── env_YYYYMMDD_HHMMSS.txt  # 환경 스냅샷
│   └── core/                        # 코어 덤프 (크래시 시)
├── tests/                           # 테스트 스위트
│   ├── test_kit_boot.py             # SimulationApp 부팅 테스트
│   └── test_urdf_validation.py      # URDF 무결성 테스트
└── .vscode/
    └── tasks.json                   # VS Code Task 통합
```

---

## Preflight 검사

**목적**: 실행 전 환경 문제를 사전에 차단 (Fail-Fast)

### 1. System Check

```bash
bash devops/preflight/check_system.sh
```

**검사 항목**:
- ✅ GPU & NVIDIA Driver (≥ 550.x)
- ✅ CUDA Toolkit
- ✅ Vulkan (vulkaninfo)
- ✅ Python 버전 (3.10 or 3.11)
- ✅ 환경변수 (`CUDA_VISIBLE_DEVICES` 충돌 감지)

**출력 예시**:
```
[1/5] GPU & Driver Check
Driver Version: 580.95.05
✓ Driver version OK (>= 550)

[5/5] Environment Variables Check
CUDA_VISIBLE_DEVICES: <not set>
✓ CUDA_VISIBLE_DEVICES not set (good - avoid conflicts)

PREFLIGHT PASSED! System ready for Isaac Sim.
```

### 2. Isaac Sim Extensions Check

```bash
source ~/isaacsim-venv/bin/activate
python devops/preflight/check_isaac_extensions.py
```

**검사 항목**:
- ✅ SimulationApp headless 부팅 (5초 제한)
- ✅ 필수 확장 로드 확인:
  - `isaacsim.asset.importer.urdf`
  - `isaacsim.core.api`
  - `omni.isaac.core`
  - `omni.kit.viewport.window`
  - `omni.physx`
  - `omni.usd`

**실패 시**: 
- API 변경 감지
- Extension 누락 즉시 확인

### 3. USD Integrity Check

```bash
bash devops/preflight/check_usd_integrity.sh assets/roarm_m3/usd/roarm_m3.usd
```

**검사 항목**:
- ✅ USD Stage 로드 가능 여부 (pxr 모듈 사용)
- ✅ Stage 메타데이터 (metersPerUnit, upAxis, defaultPrim)
- ✅ RigidBody에 MassAPI 적용 확인
- ✅ ArticulationRootAPI 중복 검사
- ✅ Joint DriveAPI 설정 확인

**출력 예시**:
```
[3/5] Mass & Inertia Check
  Found 7 rigid bodies
✓ All rigid bodies have valid mass/inertia

[4/5] Articulation Check
  Found 1 articulation root(s)
✓ Single articulation root: /World/roarm_m3

PREFLIGHT PASSED! USD model ready for simulation.
```

### 4. 통합 실행

```bash
bash devops/preflight_all.sh
```

모든 검사를 순차 실행:
1. System → 2. Extensions → 3. USD Integrity

**VS Code**: `Ctrl+Shift+P` → `Tasks: Run Task` → `Preflight: All Checks`

---

## ⚠️ CRITICAL: pxr 모듈 환경 설정 (pip 설치 시)

### 문제 배경

**pip으로 설치한 Isaac Sim 5.0**은 표준 설치본과 달리 `python.sh` 런처가 없으며, **pxr (USD Python 바인딩) 모듈이 기본 PYTHONPATH에 포함되지 않습니다.**

USD 파일 무결성 검사, URDF→USD 변환 등의 작업은 **반드시 Isaac Sim 번들에 포함된 pxr 모듈을 사용해야 합니다.**

### 🚫 금지 사항: pip install usd-core

```bash
# ❌ 절대 실행 금지!
pip install usd-core
```

**이유**:
- PyPI의 `usd-core`는 Isaac Sim과 **ABI(Application Binary Interface)가 다름**
- PhysX, Omniverse 확장과 충돌 발생
- 런타임 크래시, 스키마 불일치 유발

### ✅ 올바른 방법: Isaac 번들 pxr 사용

#### 1. pxr 모듈 위치 확인

pip으로 설치한 경우:
```bash
# Isaac Sim venv 활성화
source ~/isaacsim-venv/bin/activate

# pxr 모듈 위치 찾기
find ~/isaacsim-venv/lib/python3.11/site-packages -path "*/omni.usd.libs*/pxr" -type d

# 예시 출력:
# ~/isaacsim-venv/lib/python3.11/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311/pxr
```

**중요**: 라이브러리 경로는 **`/bin`** 디렉토리에 있습니다 (**`/lib64` 아님!**)

```bash
# 올바른 경로 예시:
export LD_LIBRARY_PATH="/home/user/isaacsim-venv/lib/python3.11/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311/bin:${LD_LIBRARY_PATH:-}"
```

#### 2. 환경변수 설정 (수동 실행 시)

```bash
# USD 라이브러리 경로 찾기
USD_LIBS_DIR=$(find ~/isaacsim-venv/lib/python3.11/site-packages \
  -path "*/omni.usd.libs*/pxr" -type d | head -1)
USD_LIBS_DIR=$(dirname "$USD_LIBS_DIR")

# PYTHONPATH 설정
export PYTHONPATH="$USD_LIBS_DIR:${PYTHONPATH:-}"

# LD_LIBRARY_PATH 설정 (주의: /bin 사용!)
export LD_LIBRARY_PATH="$USD_LIBS_DIR/bin:${LD_LIBRARY_PATH:-}"

# Python 실행
python your_usd_script.py
```

#### 3. 자동 래퍼 스크립트 사용 (권장)

**방법 A: isaac_python.sh 사용**
```bash
# devops/isaac_python.sh가 자동으로 환경 설정
bash devops/isaac_python.sh -c "import pxr; print(pxr.__file__)"
bash devops/isaac_python.sh your_script.py
```

**방법 B: preflight 스크립트 직접 실행**
```bash
# check_usd_integrity.sh는 내장 환경 감지 기능 포함
bash devops/preflight/check_usd_integrity.sh assets/roarm_m3/usd/roarm_m3.usd
```

#### 4. pxr 모듈 로드 검증

**진단 스크립트**:
```python
import sys
import importlib.util

# pxr.Usd 모듈 위치 확인
spec = importlib.util.find_spec("pxr.Usd")
if spec and spec.origin:
    print(f"✓ pxr.Usd found at: {spec.origin}")
    # 예시: .../omni.usd.libs-1.0.1+.../pxr/Usd/_usd.so
else:
    print("✗ pxr.Usd not found in PYTHONPATH")
    sys.exit(1)

# 실제 import 테스트
try:
    from pxr import Usd, UsdPhysics, PhysxSchema
    print("✓ pxr modules loaded successfully")
except ImportError as e:
    print(f"✗ Import failed: {e}")
    sys.exit(1)
```

#### 5. 트러블슈팅

**증상 1: ModuleNotFoundError: No module named 'pxr'**
```bash
# 해결: PYTHONPATH 확인
echo $PYTHONPATH
# omni.usd.libs가 포함되어 있어야 함

# 자동 수정
source devops/isaac_python.sh
```

**증상 2: ImportError: libusd_*.so: cannot open shared object file**
```bash
# 해결: LD_LIBRARY_PATH 확인
echo $LD_LIBRARY_PATH
# .../omni.usd.libs-*/bin이 포함되어 있어야 함 (/lib64 아님!)

# 경로 확인
ls -la $USD_LIBS_DIR/bin/libusd_*.so
```

**증상 3: Python subprocess에서 pxr import 실패**
```bash
# 문제: 환경변수가 서브프로세스로 전달되지 않음
# 해결: exec 전에 export 실행
export PYTHONPATH="..."
export LD_LIBRARY_PATH="..."
python -c "import pxr"  # 이제 성공
```

### 구현 참고: check_usd_integrity.sh

devops/preflight/check_usd_integrity.sh는 다음 로직을 사용합니다:

```bash
find_isaac_python() {
  # 1. 표준 설치 (python.sh 존재)
  if [[ -f "$ISAAC_PATH/python.sh" ]]; then
    echo "standard:$ISAAC_PATH/python.sh"
    return
  fi
  
  # 2. pip 설치 (venv 감지)
  if python -c "import isaacsim" 2>/dev/null; then
    echo "venv:$(which python)"
    return
  fi
  
  echo "none"
}

run_with_isaac_python() {
  local isaac_py="$1"
  local usd_file="$2"
  
  if [[ "$isaac_py" == venv:* ]]; then
    # pxr 경로 자동 탐색
    local pxr_path=$(find "$VIRTUAL_ENV/lib" -path "*/omni.usd.libs*/pxr" | head -1)
    local usd_libs_dir=$(dirname "$pxr_path")
    
    # 환경변수 설정 후 Python 실행
    export PYTHONPATH="$usd_libs_dir:${PYTHONPATH:-}"
    export LD_LIBRARY_PATH="$usd_libs_dir/bin:${LD_LIBRARY_PATH:-}"
    
    python - "$usd_file" <<'PYTHON'
import sys
from pxr import Usd, UsdPhysics
# USD 검증 로직...
PYTHON
  fi
}
```

### 핵심 교훈

| 항목 | 내용 |
|------|------|
| **금지** | `pip install usd-core` (ABI 불일치) |
| **필수** | Isaac 번들 pxr 사용 (omni.usd.libs) |
| **경로** | 라이브러리는 `/bin` 디렉토리 (**`/lib64` 아님!**) |
| **환경변수** | PYTHONPATH + LD_LIBRARY_PATH 모두 설정 |
| **진단** | `importlib.util.find_spec("pxr.Usd").origin` |
| **자동화** | `devops/isaac_python.sh` 래퍼 사용 |

---

## 로그 관리

### Supervised Runner 사용

```bash
bash devops/run_isaac_supervised.sh scripts/usd/convert_to_usd.py [args...]
```

**기능**:
1. **Preflight 자동 실행**: 문제가 있으면 즉시 중단
2. **환경 스냅샷 저장**: `logs/isaac/env_YYYYMMDD_HHMMSS.txt`
   - Python 버전, pip freeze, 환경변수 전체 저장
3. **실시간 로그 수집**: `tee`로 터미널 + 파일 동시 저장
4. **코어 덤프 활성화**: `ulimit -c unlimited`

**실행 예시**:
```bash
# GUI 실행
bash devops/run_isaac_supervised.sh scripts/launch_gui.py

# USD 변환
bash devops/run_isaac_supervised.sh scripts/usd/convert_to_usd.py \
  assets/roarm_m3/urdf/roarm_m3_complete.urdf \
  assets/roarm_m3/usd/roarm_m3.usd
```

### 로그 파일 위치

```
logs/isaac/
├── isaac_20251017_184451.log   # Isaac Sim 실행 로그
├── env_20251017_184451.txt     # 환경 스냅샷
└── ...

logs/core/
└── core.isaac.*.dump            # 크래시 발생 시 (있는 경우)
```

### tmux 사용 (권장)

IDE와 프로세스 분리:

```bash
# tmux 세션 시작
tmux new -s isaac

# Supervised runner 실행
bash devops/run_isaac_supervised.sh scripts/launch_gui.py

# Detach: Ctrl+B, D
# Reattach: tmux attach -t isaac
```

**장점**:
- IDE가 죽어도 Isaac Sim은 계속 실행
- 로그는 파일에 저장되어 안전
- 언제든지 다시 접속 가능

---

## 테스트

### 1. URDF Validation Test

```bash
pytest tests/test_urdf_validation.py -v
```

**검사 항목**:
- URDF 파일 존재
- XML 구문 유효성
- Link/Joint 존재
- Mesh 파일 실제 존재

### 2. Isaac Sim Boot Test

```bash
source ~/isaacsim-venv/bin/activate
pytest tests/test_kit_boot.py -v
```

**검사 항목**:
- Headless 부팅 성공
- 필수 확장 로드 확인

### 3. 전체 테스트 실행

```bash
pytest tests/ -v
```

---

## VS Code 통합

### Task 단축키

1. **Preflight: All Checks**
   - `Ctrl+Shift+P` → `Tasks: Run Test Task` (기본 테스트)
   - 모든 사전 검사 실행

2. **Validate URDF**
   - `Ctrl+Shift+P` → `Tasks: Run Task` → `Validate URDF`
   - URDF 무결성 검사

3. **Convert URDF to USD**
   - `Ctrl+Shift+P` → `Tasks: Run Task` → `Convert URDF to USD`
   - Preflight → Validation → Conversion 자동 실행

4. **Run: Isaac Sim (Supervised)**
   - `Ctrl+Shift+P` → `Tasks: Run Build Task`
   - 스크립트 경로 입력 요청 → 감시 실행

### Task 체인

```
Validate URDF
    ↓
Convert URDF to USD
    ↓ (dependsOn)
Preflight: All Checks
    ↓
Supervised Execution
```

---

## 문제 해결

### 1. "CUDA_VISIBLE_DEVICES 경고"

**증상**: Preflight에서 CUDA_VISIBLE_DEVICES 충돌 경고

**해결**:
```bash
# 환경변수 제거
unset CUDA_VISIBLE_DEVICES

# 또는 .bashrc에서 제거
nano ~/.bashrc
# export CUDA_VISIBLE_DEVICES=0 주석 처리
```

### 2. "Driver Version Too Old"

**증상**: Driver < 550.x

**해결**:
```bash
# Driver 업데이트 (Ubuntu)
sudo ubuntu-drivers install nvidia:580

# 재부팅
sudo reboot
```

### 3. "Vulkan Not Found"

**증상**: vulkaninfo 명령어 없음

**해결**:
```bash
sudo apt install vulkan-utils
```

### 4. "Extension Not Enabled"

**증상**: `check_isaac_extensions.py` 실패

**해결**:
```bash
# Isaac Sim venv 확인
source ~/isaacsim-venv/bin/activate
which python  # ~/isaacsim-venv/bin/python 확인

# pip 재설치 (필요 시)
pip install --upgrade isaacsim
```

### 5. "USD File Errors: 0, Warnings: N"

**증상**: USD integrity check에서 경고

**해결**:
- DriveAPI 누락: 정상 (post-import에서 설정)
- MassAPI 누락: URDF에 inertia 추가 필요

### 6. "pxr 모듈 import 실패"

**증상**: `ModuleNotFoundError: No module named 'pxr'`

**해결**:
```bash
# 1. Isaac Sim venv 활성화 확인
source ~/isaacsim-venv/bin/activate

# 2. devops/isaac_python.sh 래퍼 사용
bash devops/isaac_python.sh -c "import pxr"

# 3. 수동 환경변수 설정 (고급)
# 위 "⚠️ CRITICAL: pxr 모듈 환경 설정" 섹션 참조
```

**주의**: `pip install usd-core` 절대 금지!

### 7. "로그 파일이 너무 큼"

**증상**: logs/isaac/*.log 파일이 수백 MB

**해결**:
```bash
# 로그 압축
gzip logs/isaac/isaac_*.log

# 오래된 로그 정리 (30일 이상)
find logs/isaac -name "*.log" -mtime +30 -delete
```

---

## 체크리스트

### 개발 시작 전

- [ ] `bash devops/preflight_all.sh` 실행
- [ ] Driver 버전 확인 (≥ 550)
- [ ] Python 버전 확인 (3.10 or 3.11)
- [ ] Isaac Sim venv 활성화

### 스크립트 실행 시

- [ ] Supervised runner 사용
- [ ] tmux 세션에서 실행 (장기 작업)
- [ ] 로그 파일 경로 확인

### 문제 발생 시

- [ ] `logs/isaac/` 최신 로그 확인
- [ ] `logs/isaac/env_*.txt` 환경 스냅샷 확인
- [ ] Preflight 재실행으로 환경 검증
- [ ] 코어 덤프 존재 시 분석

---

## 추가 개선 사항

### 향후 추가 예정

1. **Context Bridge 통합**
   - 로그 + 환경 스냅샷 자동 번들링
   - AI 분석용 컨텍스트 자동 생성

2. **버전 고정**
   - `requirements.lock` 생성
   - `extension.toml.lock` 관리

3. **API Stub 생성**
   - Isaac Sim 설치본에서 `.pyi` 자동 생성
   - Copilot 자동완성 향상

4. **CI/CD**
   - GitHub Actions로 Preflight 자동화
   - USD 무결성 회귀 테스트

---

**작성**: GitHub Copilot  
**프로젝트**: roarm_isaac_clean  
**버전**: 1.0
