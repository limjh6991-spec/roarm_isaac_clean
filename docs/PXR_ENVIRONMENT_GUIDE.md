# pxr (USD Python 바인딩) 환경 설정 가이드

**작성일**: 2025-10-18  
**대상**: Isaac Sim 5.0 pip 설치 사용자  
**중요도**: ⚠️ **CRITICAL** - USD 관련 모든 작업의 필수 전제조건

---

## 🎯 이 가이드를 읽어야 하는 경우

다음 증상 중 하나라도 해당되면 **반드시** 이 가이드를 따르세요:

- ✅ `ModuleNotFoundError: No module named 'pxr'` 오류 발생
- ✅ USD 파일 검증 스크립트 실행 실패
- ✅ URDF→USD 변환 시 pxr import 오류
- ✅ `ImportError: libusd_*.so: cannot open shared object file`
- ✅ Isaac Sim을 **pip으로 설치**했고 `python.sh` 런처가 없음

---

## 🚫 절대 금지: pip install usd-core

### 왜 금지인가?

```bash
# ❌ 절대 실행하지 마세요!
pip install usd-core
```

**이유**:
1. **ABI (Application Binary Interface) 불일치**
   - PyPI의 `usd-core`는 표준 컴파일러/설정으로 빌드됨
   - Isaac Sim의 pxr는 Omniverse 전용 설정으로 빌드됨
   - 서로 다른 C++ 런타임 라이브러리 사용

2. **스키마 호환성 문제**
   - PhysxSchema, UsdPhysics 등 Isaac 전용 스키마가 누락됨
   - 스키마 버전 불일치로 USD 파일 검증 실패

3. **런타임 크래시**
   - 혼재된 라이브러리 버전으로 세그폴트 발생
   - 디버깅 매우 어려움 (Stack trace가 C++ 내부까지 파고듦)

### 올바른 방법

✅ **Isaac Sim 번들에 포함된 pxr 모듈 사용**

---

## 📂 pxr 모듈 위치

### pip 설치본 구조

```
~/isaacsim-venv/
└── lib/
    └── python3.11/
        └── site-packages/
            └── isaacsim/
                └── extscache/
                    └── omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311/
                        ├── pxr/              # Python 모듈
                        │   ├── Usd/
                        │   ├── UsdPhysics/
                        │   ├── PhysxSchema/
                        │   └── ...
                        └── bin/              # 공유 라이브러리 (.so 파일)
                            ├── libusd_tf.so
                            ├── libusd_usd.so
                            └── ...
```

**주의**: 버전 문자열 (`1.0.1+8131b85d.lx64.r.cp311`)은 설치마다 다를 수 있음

### 자동 경로 탐색

```bash
# Isaac Sim venv 활성화
source ~/isaacsim-venv/bin/activate

# pxr 모듈 위치 찾기
find ~/isaacsim-venv/lib/python3.11/site-packages \
  -path "*/omni.usd.libs*/pxr" -type d

# 예시 출력:
# /home/user/isaacsim-venv/lib/python3.11/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311/pxr
```

---

## 🔧 환경변수 설정

### 필수 환경변수

| 변수 | 용도 | 설정 값 |
|------|------|---------|
| `PYTHONPATH` | pxr 모듈 경로 | `$USD_LIBS_DIR` (pxr의 부모 디렉토리) |
| `LD_LIBRARY_PATH` | 공유 라이브러리 경로 | `$USD_LIBS_DIR/bin` (**`/lib64` 아님!**) |

### 수동 설정 (일회성 실행)

```bash
# 1. Isaac Sim venv 활성화
source ~/isaacsim-venv/bin/activate

# 2. pxr 모듈 경로 찾기
USD_LIBS_DIR=$(find ~/isaacsim-venv/lib/python3.11/site-packages \
  -path "*/omni.usd.libs*/pxr" -type d | head -1)
USD_LIBS_DIR=$(dirname "$USD_LIBS_DIR")

# 3. PYTHONPATH 설정
export PYTHONPATH="$USD_LIBS_DIR:${PYTHONPATH:-}"

# 4. LD_LIBRARY_PATH 설정 (주의: /bin 사용!)
export LD_LIBRARY_PATH="$USD_LIBS_DIR/bin:${LD_LIBRARY_PATH:-}"

# 5. 검증
python -c "import pxr; print('✓ pxr loaded:', pxr.__file__)"
```

### 자동 설정 (권장)

#### 방법 1: isaac_python.sh 래퍼 사용

```bash
# Python 스크립트 실행
bash devops/isaac_python.sh your_usd_script.py

# Python 코드 직접 실행
bash devops/isaac_python.sh -c "import pxr; print(pxr.__version__)"

# Python 인터랙티브 셸
bash devops/isaac_python.sh
```

**동작 원리**:
- `isaac_python.sh`가 자동으로 omni.usd.libs 경로 탐색
- PYTHONPATH, LD_LIBRARY_PATH 자동 설정
- `exec python "$@"`로 Python 실행

#### 방법 2: preflight 스크립트 직접 실행

```bash
# USD 무결성 검사 (내장 환경 감지)
bash devops/preflight/check_usd_integrity.sh assets/roarm_m3/usd/roarm_m3.usd
```

**동작 원리**:
- `find_isaac_python()` 함수가 설치 타입 자동 감지 (standard vs pip)
- `run_with_isaac_python()` 함수가 pip 설치 시 환경변수 자동 설정

---

## 🔍 진단 및 검증

### 1. pxr 모듈 위치 확인

```python
import importlib.util

# pxr.Usd 모듈의 실제 .so 파일 위치 확인
spec = importlib.util.find_spec("pxr.Usd")
if spec and spec.origin:
    print(f"✓ pxr.Usd found at: {spec.origin}")
    # 예시: .../omni.usd.libs-1.0.1+.../pxr/Usd/_usd.so
else:
    print("✗ pxr.Usd not found in PYTHONPATH")
```

### 2. PYTHONPATH 검증

```bash
# 현재 PYTHONPATH 확인
echo "$PYTHONPATH" | tr ':' '\n'

# omni.usd.libs가 포함되어 있어야 함
# 예시:
# /home/user/isaacsim-venv/lib/python3.11/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+.../
```

### 3. LD_LIBRARY_PATH 검증

```bash
# 현재 LD_LIBRARY_PATH 확인
echo "$LD_LIBRARY_PATH" | tr ':' '\n'

# .../omni.usd.libs-*/bin이 포함되어 있어야 함 (/lib64 아님!)

# 라이브러리 파일 존재 확인
ls -la $USD_LIBS_DIR/bin/libusd_*.so | head -5
# 예시 출력:
# libusd_ar.so
# libusd_gf.so
# libusd_tf.so
# libusd_usd.so
# ...
```

### 4. pxr import 전체 테스트

```python
import sys
import importlib.util

# 진단 출력
print("=== pxr Module Diagnostic ===")

# 1. pxr.Usd 모듈 위치
spec = importlib.util.find_spec("pxr.Usd")
if spec and spec.origin:
    print(f"✓ pxr.Usd found at: {spec.origin}")
else:
    print("✗ pxr.Usd not found")
    sys.exit(1)

# 2. 실제 import 테스트
try:
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema
    print("✓ Core modules imported successfully")
except ImportError as e:
    print(f"✗ Import failed: {e}")
    sys.exit(1)

# 3. USD Stage 생성 테스트
try:
    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/World", "Xform")
    print("✓ USD Stage creation successful")
except Exception as e:
    print(f"✗ Stage creation failed: {e}")
    sys.exit(1)

print("\n✅ All pxr diagnostics passed!")
```

---

## ❌ 트러블슈팅

### 문제 1: ModuleNotFoundError: No module named 'pxr'

**증상**:
```python
>>> import pxr
ModuleNotFoundError: No module named 'pxr'
```

**해결**:
```bash
# 1. PYTHONPATH 확인
echo $PYTHONPATH

# 2. omni.usd.libs가 포함되어 있지 않으면 수동 설정
USD_LIBS_DIR=$(find ~/isaacsim-venv/lib/python3.11/site-packages \
  -path "*/omni.usd.libs*/pxr" -type d | head -1)
USD_LIBS_DIR=$(dirname "$USD_LIBS_DIR")
export PYTHONPATH="$USD_LIBS_DIR:${PYTHONPATH:-}"

# 3. 재시도
python -c "import pxr"
```

**또는 래퍼 사용**:
```bash
bash devops/isaac_python.sh -c "import pxr"
```

---

### 문제 2: ImportError: libusd_*.so: cannot open shared object file

**증상**:
```python
>>> from pxr import Usd
ImportError: libusd_usd.so: cannot open shared object file: No such file or directory
```

**원인**: LD_LIBRARY_PATH가 설정되지 않았거나 잘못된 경로

**해결**:
```bash
# 1. LD_LIBRARY_PATH 확인
echo $LD_LIBRARY_PATH

# 2. 올바른 경로 설정 (주의: /bin 사용!)
USD_LIBS_DIR=$(find ~/isaacsim-venv/lib/python3.11/site-packages \
  -path "*/omni.usd.libs*/pxr" -type d | head -1)
USD_LIBS_DIR=$(dirname "$USD_LIBS_DIR")
export LD_LIBRARY_PATH="$USD_LIBS_DIR/bin:${LD_LIBRARY_PATH:-}"

# 3. 라이브러리 파일 확인
ls -la $USD_LIBS_DIR/bin/libusd_*.so | head -3

# 4. 재시도
python -c "from pxr import Usd"
```

**흔한 실수**: `/lib64` 디렉토리를 사용하는 경우
```bash
# ❌ 잘못된 예시
export LD_LIBRARY_PATH="$USD_LIBS_DIR/lib64:..."  # lib64는 없음!

# ✅ 올바른 예시
export LD_LIBRARY_PATH="$USD_LIBS_DIR/bin:..."    # bin 디렉토리 사용
```

---

### 문제 3: Python subprocess에서 pxr import 실패

**증상**:
```bash
# 직접 실행은 성공
$ python -c "import pxr"
# (출력 없음 = 성공)

# 스크립트 내 subprocess는 실패
$ python my_script.py
# ModuleNotFoundError: No module named 'pxr'
```

**원인**: 환경변수가 서브프로세스로 전달되지 않음

**해결**:
```python
# ❌ 잘못된 방법
import subprocess
subprocess.run(["python", "-c", "import pxr"])  # 실패

# ✅ 올바른 방법 1: env 딕셔너리 전달
import subprocess
import os

env = os.environ.copy()
env["PYTHONPATH"] = f"{usd_libs_dir}:{env.get('PYTHONPATH', '')}"
env["LD_LIBRARY_PATH"] = f"{usd_libs_dir}/bin:{env.get('LD_LIBRARY_PATH', '')}"

subprocess.run(["python", "-c", "import pxr"], env=env)  # 성공

# ✅ 올바른 방법 2: bash 스크립트로 환경 설정 후 exec
# devops/isaac_python.sh 참조
```

**Bash 스크립트 패턴**:
```bash
#!/bin/bash
# isaac_python.sh

# 1. pxr 경로 탐색
USD_LIBS_DIR=$(find $VIRTUAL_ENV/lib -path "*/omni.usd.libs*/pxr" | head -1)
USD_LIBS_DIR=$(dirname "$USD_LIBS_DIR")

# 2. 환경변수 설정
export PYTHONPATH="$USD_LIBS_DIR:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$USD_LIBS_DIR/bin:${LD_LIBRARY_PATH:-}"

# 3. exec로 Python 실행 (subprocess 아님!)
exec python "$@"
```

---

### 문제 4: Isaac Sim venv 미활성화

**증상**:
```bash
$ python -c "import pxr"
ModuleNotFoundError: No module named 'pxr'
```

**원인**: 시스템 Python을 사용 중

**해결**:
```bash
# 1. 현재 Python 경로 확인
which python
# /usr/bin/python (시스템 Python - 잘못됨!)

# 2. Isaac Sim venv 활성화
source ~/isaacsim-venv/bin/activate

# 3. 다시 확인
which python
# ~/isaacsim-venv/bin/python (정확함!)

# 4. 재시도
python -c "import pxr"
```

---

## 📚 참고 구현

### devops/isaac_python.sh

```bash
#!/bin/bash
# Isaac Sim Python 환경 자동 설정 래퍼

set -euo pipefail

# Isaac Sim venv 활성화 확인
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
  echo "Error: Isaac Sim venv not activated"
  echo "Run: source ~/isaacsim-venv/bin/activate"
  exit 1
fi

# pxr 모듈 경로 찾기
PXR_PATH=$(find "$VIRTUAL_ENV/lib" -path "*/omni.usd.libs*/pxr" -type d | head -1)
if [[ -z "$PXR_PATH" ]]; then
  echo "Error: omni.usd.libs not found in venv"
  exit 1
fi

USD_LIBS_DIR=$(dirname "$PXR_PATH")

# 환경변수 설정
export PYTHONPATH="$USD_LIBS_DIR:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$USD_LIBS_DIR/bin:${LD_LIBRARY_PATH:-}"

# Python 실행
exec python "$@"
```

### devops/preflight/check_usd_integrity.sh

```bash
#!/bin/bash
# USD 무결성 검사 (pxr 환경 자동 감지)

find_isaac_python() {
  # 1. 표준 설치 (python.sh 존재)
  if [[ -f "/path/to/isaac-sim/python.sh" ]]; then
    echo "standard:/path/to/isaac-sim/python.sh"
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
from pxr import Usd, UsdPhysics, PhysxSchema

usd_file = sys.argv[1]
stage = Usd.Stage.Open(usd_file)
# USD 검증 로직...
PYTHON
  fi
}

# 메인 실행
ISAAC_PY=$(find_isaac_python)
run_with_isaac_python "$ISAAC_PY" "$1"
```

---

## ✅ 체크리스트

### 개발 시작 전
- [ ] Isaac Sim venv 활성화 확인
- [ ] pxr 모듈 위치 확인 (omni.usd.libs)
- [ ] PYTHONPATH 설정 확인
- [ ] LD_LIBRARY_PATH 설정 확인 (`/bin` 디렉토리!)

### 스크립트 작성 시
- [ ] `devops/isaac_python.sh` 래퍼 사용
- [ ] 또는 수동 환경변수 설정 후 `exec python`
- [ ] subprocess 사용 시 `env` 딕셔너리 전달

### 문제 발생 시
- [ ] `importlib.util.find_spec("pxr.Usd")` 실행
- [ ] PYTHONPATH, LD_LIBRARY_PATH 출력 확인
- [ ] `.../omni.usd.libs-*/bin/libusd_*.so` 파일 존재 확인
- [ ] 절대 `pip install usd-core` 실행 금지 확인

---

## 🎯 핵심 요약

| 항목 | 내용 |
|------|------|
| **금지** | `pip install usd-core` (ABI 불일치) |
| **필수** | Isaac 번들 pxr 사용 (omni.usd.libs) |
| **경로** | 라이브러리는 `/bin` 디렉토리 (**`/lib64` 아님!**) |
| **환경변수** | PYTHONPATH + LD_LIBRARY_PATH 모두 설정 |
| **진단** | `importlib.util.find_spec("pxr.Usd").origin` |
| **자동화** | `devops/isaac_python.sh` 래퍼 사용 |

---

**작성**: GitHub Copilot  
**프로젝트**: roarm_isaac_clean  
**버전**: 1.0  
**마지막 업데이트**: 2025-10-18
