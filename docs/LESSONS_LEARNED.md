# 이전 프로젝트(codex_mcp)에서 배운 교훈

**작성일**: 2025년 10월 15일  
**출처**: ~/codex_mcp 프로젝트 분석  
**목적**: 동일한 실수 반복 방지 및 올바른 접근 방법 확립

---

## 📊 프로젝트 개요

### 목표
- **RoArm M3 로봇팔**(6 DOF) Isaac Sim 5.0 환경에서 **강화학습(PPO)** 기반 제어 학습
- 도메인 랜덤라이제이션을 통한 **Sim-to-Real 전이** 준비
- 원격 GUI/headless 양쪽에서 **정책 재생 및 검증** 가능한 통합 파이프라인 구축

### 기술 스택
| 구성 요소 | 버전/상세 | 경로 |
|----------|----------|-----|
| **Isaac Sim** | 5.0 (pip, Python 3.11 전용) | `~/isaacsim-venv` |
| **GPU** | NVIDIA RTX 5090, CUDA 12.8 | - |
| **RL Framework** | Stable-Baselines3 2.x + Gymnasium | - |
| **로봇 모델** | RoArm M3 USD | `assets/roarm_m3/usd/` |
| **Physics** | PhysX 107.3.18 (TGS solver, 1/120 Hz) | - |

---

## ❌ 핵심 문제점 (3가지)

### 문제 1: USD CollisionAPI 누락 → PhysX 크래시 ⚡ **최우선**

**증상:**
- GUI 재생 후 타임라인 재생 버튼 클릭 시 즉시 세그폴트
- 에러 로그: `"Invalid PhysX transform"` → `"Illegal BroadPhaseUpdateData"` → 크래시

**근본 원인:**
```python
# USD 건강검진 결과
CollisionAPI missing on: /World/roarm/base_link
CollisionAPI missing on: /World/roarm/link_base
CollisionAPI missing on: /World/roarm/link_shoulder
CollisionAPI missing on: /World/roarm/link_elbow
CollisionAPI missing on: /World/roarm/link_wrist1
CollisionAPI missing on: /World/roarm/link_wrist2
CollisionAPI missing on: /World/roarm/link_gripper
```
- **모든 링크**에 충돌 형상(Collision Shape)이 없어 PhysX BroadPhase 초기화 실패
- 시뮬레이션 시작 불가능 → 학습/재생 모두 위험

**영향 범위:**
- ✅ Headless 학습: 타임라인 자동 재생이 없어 일부 우회 가능 (10k 스텝 학습 성공)
- ❌ GUI 재생: 타임라인 재생 시 100% 크래시
- ❌ 실제 물리 검증: 충돌 없이는 정상 시뮬레이션 불가능

**교훈:**
- ✅ **USD 파일은 반드시 CollisionAPI와 함께 생성해야 함**
- ✅ **URDF → USD 변환 후 즉시 검증 스크립트 실행 필수**
- ✅ **PhysX 관련 에러는 초기에 해결하지 않으면 디버깅 시간 폭증**

---

### 문제 2: 원격 GUI 렌더링 실패 🖥️ **높은 우선순위**

**증상:**
- 원격 SSH 세션에서 Isaac Sim GUI 창은 뜨지만 **내용이 비어 있음** (검은 화면/회색 화면)
- 콘솔 로그는 정상 진행 (500스텝 완료, 보상 -2644.434 기록)

**시도한 해결책:**
1. **VirtualGL 설치 시도 → 실패** (Ubuntu 24.04 의존성 충돌)
2. **환경변수 조정** (KIT_USE_EGL, OMNI_KIT_FORCE_VULKAN 등)

**근본 원인:**
- 원격 X11 포워딩 시 OpenGL/Vulkan 컨텍스트가 로컬 GPU에서 원격 디스플레이로 제대로 스트리밍되지 않음
- Isaac Sim의 Omniverse UI는 고성능 3D 렌더링을 요구하므로 일반 X11 포워딩으로는 한계

**권장 대안:**
| 방법 | 장점 | 단점 | 권장도 |
|-----|------|------|--------|
| **WebRTC 스트리밍** | 브라우저 접속만으로 가능, 60fps 지원 | 설정 필요 | ⭐⭐⭐⭐⭐ |
| **NICE DCV** | 저지연 원격 데스크톱, GPU 가속 지원 | 별도 설치 필요 | ⭐⭐⭐⭐ |
| **VNC + TurboVNC** | 오픈소스, 범용성 | 성능 제한적 | ⭐⭐⭐ |

**교훈:**
- ✅ **원격 환경에서는 처음부터 WebRTC 또는 NICE DCV 설정 계획**
- ✅ **X11 포워딩은 Isaac Sim GUI 렌더링에 부적합**
- ✅ **Headless 모드 + 로깅/메트릭 수집을 기본 전략으로 고려**

---

### 문제 3: 구형 Isaac API 사용 → 종료 시 AttributeError 🐛

**증상:**
```python
AttributeError: 'Articulation' object has no attribute '_callbacks'
```

**근본 원인:**
```python
# 구형 API (Isaac Sim 4.x 스타일)
from omni.isaac.core.articulations import Articulation
self._articulation = Articulation(prim_path=..., name="roarm")
```
- `omni.isaac.core.articulations.Articulation`은 Isaac Sim 5.0에서 **deprecated**
- 내부적으로 `_callbacks` 속성을 누락하여 종료 시 에러 발생

**권장 마이그레이션:**
```python
# 최신 API (Isaac Sim 5.0 권장)
from isaacsim.core.prims import SingleArticulation
self._articulation = SingleArticulation(prim_path=...)
```

**교훈:**
- ✅ **Isaac Sim 5.0에서는 `isaacsim.core.*` API 사용 필수**
- ✅ **`omni.isaac.core.*`는 deprecated, 마이그레이션 가이드 참조**
- ✅ **API 변경 사항을 문서화하고 코드 검토 시 확인**

---

### 문제 4: pip 설치 시 pxr 모듈 환경 설정 ⚠️ **신규 프로젝트 핵심**

**증상:**
```python
ModuleNotFoundError: No module named 'pxr'
```
- USD 파일 검증 스크립트 실행 시 pxr 모듈을 찾을 수 없음
- `python -c "import pxr"` 직접 실행 시에도 동일한 오류

**시도한 해결책:**
1. ❌ **pip install usd-core** → ABI 불일치로 실패
   - PyPI의 usd-core는 Isaac Sim 번들과 **다른 컴파일러/설정으로 빌드됨**
   - PhysxSchema, Omniverse 확장과 충돌 발생
   - 런타임 크래시 및 스키마 검증 실패

2. ✅ **Isaac 번들 pxr 사용** → 성공
   - pip 설치본의 omni.usd.libs 경로 자동 탐색
   - PYTHONPATH + LD_LIBRARY_PATH 설정

**근본 원인:**
- **pip으로 설치한 Isaac Sim 5.0**은 표준 설치본과 달리 `python.sh` 런처가 없음
- pxr 모듈은 `$VENV/lib/python3.11/site-packages/isaacsim/extscache/omni.usd.libs-*/pxr`에 있음
- **기본 PYTHONPATH에 포함되지 않아** 수동 환경 설정 필수

**성공한 해결책:**
```bash
# 1. pxr 모듈 위치 자동 탐색
USD_LIBS_DIR=$(find ~/isaacsim-venv/lib/python3.11/site-packages \
  -path "*/omni.usd.libs*/pxr" -type d | head -1)
USD_LIBS_DIR=$(dirname "$USD_LIBS_DIR")

# 2. PYTHONPATH 설정
export PYTHONPATH="$USD_LIBS_DIR:${PYTHONPATH:-}"

# 3. LD_LIBRARY_PATH 설정 (주의: /bin 디렉토리!)
export LD_LIBRARY_PATH="$USD_LIBS_DIR/bin:${LD_LIBRARY_PATH:-}"

# 4. Python 실행
python -c "import pxr; print(pxr.__file__)"
```

**핵심 발견:**
- 라이브러리 경로는 **`/bin`** 디렉토리 (**`/lib64` 아님!**)
- Python subprocess 실행 시 환경변수가 자동 전달되지 않음
- `export` 후 `exec python`으로 실행해야 함 (sourced script는 불충분)

**검증 방법:**
```python
import importlib.util

# pxr.Usd 모듈 실제 위치 확인
spec = importlib.util.find_spec("pxr.Usd")
print(f"pxr.Usd location: {spec.origin}")
# 예시: .../omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311/pxr/Usd/_usd.so
```

**자동화 도구:**
- `devops/isaac_python.sh`: pxr 환경 자동 설정 래퍼
- `devops/preflight/check_usd_integrity.sh`: 설치 타입 자동 감지 (standard vs pip)

**교훈:**
- 🚫 **절대 금지**: `pip install usd-core` (ABI 불일치)
- ✅ **필수**: Isaac 번들 pxr 사용 (omni.usd.libs)
- ✅ **경로 주의**: 라이브러리는 `/bin` (**`/lib64` 아님!**)
- ✅ **환경변수**: PYTHONPATH + LD_LIBRARY_PATH 모두 설정
- ✅ **진단 도구**: `importlib.util.find_spec()` 사용
- ✅ **자동화**: 래퍼 스크립트로 환경 설정 캡슐화

**영향 범위:**
- ✅ USD 무결성 검사: 성공 (pxr from Isaac bundle)
- ✅ URDF→USD 변환: pxr 의존성 있는 모든 작업
- ✅ 프리플라이트 시스템: check_usd_integrity.sh 정상 동작

**참고 문서:**
- `docs/DEVOPS_GUIDE.md` → "⚠️ CRITICAL: pxr 모듈 환경 설정" 섹션
- `devops/preflight/check_usd_integrity.sh` → `find_isaac_python()`, `run_with_isaac_python()` 함수

---

## 🏗️ 아키텍처 결정사항

### Dual Environment (Python 3.11 / 3.12 분리)

**배경:**
- Isaac Sim 5.0은 Python 3.11 전용 (번들 포함)
- RL 학습/분석은 Python 3.12 최신 기능 활용
- 단일 환경 통합 시도는 반복적으로 실패 (SRE module mismatch)

**결정:**
```
[ RL/Policy (Py3.12) ]  <--JSON/IPC-->  [ Isaac Sim Runtime (Py3.11) ]
 |  SB3, analysis                        | SimulationApp, physics
 |  Rollout logger / metrics             | Joint state, randomization
```

**장점:**
- ✅ 충돌 최소화
- ✅ 최신 Python 기능 유지
- ✅ Isaac 업스트림 변경 영향 최소화

**단점:**
- ⚠️ IPC 오버헤드 (serialization + context switch)
- ⚠️ 디버깅 경계 증가

**완화책:**
- Correlation ID 기반 로그 통합
- Latency 모니터링 (p95 < 60ms 목표)
- ZeroMQ/SharedMemory 전환 준비 (필요시)

**교훈:**
- ✅ **환경 분리는 때로는 최선의 선택**
- ✅ **IPC 레이턴시를 처음부터 모니터링**
- ✅ **Correlation ID로 분산 로그 추적**

---

## 🎯 성공한 부분

### ✅ 완료된 작업
1. **Isaac Sim 5.0 설치 및 실행** (pip 설치 방식)
2. **URDF → USD 기본 변환** (isaac-sim urdf 도구 사용)
3. **CollisionAPI 자동 추가 스크립트** 작성
4. **Ground Plane 추가 스크립트** 작성
5. **SB3 정책 로드 및 실행** (Exit Code 0)
6. **Dual Environment 아키텍처** 확립
7. **IPC Gateway 구현** (TCP JSON 기반)
8. **Latency 모니터링** (p50/p90/p95/p99)
9. **Domain Randomization 시스템** 구축
10. **Observability 도구** (로그, 메트릭, 알림)

### ✅ 학습된 Best Practices
1. **USD 검증 스크립트** 필수 실행
2. **단계별 접근** (URDF → USD → Physics → Control)
3. **로깅 인프라 우선** 구축
4. **환경 분리** 전략
5. **WebRTC 스트리밍** 초기 계획

---

## ⚠️ 미해결 이슈

### Issue 1: 로봇 회전 및 튕겨나감 문제
**증상:**
- 재생 시작하면 로봇이 뱅글뱅글 회전하면서 화면 밖으로 튕겨나감
- 제어 불가능한 움직임

**시도한 해결책:**
1. Kinematic Body로 Base 고정 → 실패 (Articulation 내부 kinematic 미지원)
2. Fixed Joint로 World에 고정 → 실패 (static body 간 joint 불가)

**추정 원인:**
- USD 구조가 Isaac Sim 5.0의 Articulation 시스템과 호환되지 않음
- URDF → USD 변환 과정에서 구조적 문제 발생
- Base link 고정 방법 불명확

**교훈:**
- ✅ **Isaac Sim 5.0의 Articulation 시스템 깊이 이해 필요**
- ✅ **Fixed-base robot 설정 방법 공식 문서 참조 필수**
- ✅ **땜질식 해결은 더 큰 문제 야기**

### Issue 2: Visual Geometry 단순화
**증상:**
- RoArm M3이 단순한 막대 형태로 표시됨

**원인:**
- URDF → USD 변환 시 visual mesh 누락 가능성

**교훈:**
- ✅ **Visual mesh 경로 확인 필수**
- ✅ **변환 후 시각적 검증 단계 포함**

### Issue 3: 학습 환경과 재생 환경의 불일치
**증상:**
- 학습은 성공했으나 재생 시 보상이 매우 낮음 (-2644.434 / 500스텝)
- 목표 달성률 0%

**원인:**
- 학습 환경과 재생 환경의 physics 설정 차이 가능성
- Observation space 불일치

**교훈:**
- ✅ **학습과 재생에 동일한 USD 파일 사용**
- ✅ **Physics 설정 명시적 기록**
- ✅ **Observation space 버전 관리**

---

## 📋 권장 접근 방법

### Phase 1: 자료 수집 (필수)
1. **Isaac Sim 5.0 공식 문서**
   - Articulation API 가이드
   - URDF Import best practices
   - Physics 설정 튜토리얼
   - 마이그레이션 가이드 (4.x → 5.0)

2. **RoArm M3 자료**
   - 공식 URDF/USD 파일
   - 제조사 스펙
   - 커뮤니티 예제

3. **커뮤니티 자료**
   - Reddit (r/IsaacSim, r/reinforcementlearning)
   - GitHub Issues (nvidia/isaac-sim)
   - NVIDIA Forums

### Phase 2: 단계별 검증 (필수)
```
Step 1: URDF → USD 올바른 변환
        ↓
Step 2: USD 구조 검증 (verify script)
        ↓
Step 3: Physics 단순 테스트 (중력만)
        ↓
Step 4: Articulation 제어 테스트
        ↓
Step 5: 학습 환경 구축
        ↓
Step 6: 정책 학습 및 재생
```

### Phase 3: 관측성 우선 (필수)
- 로깅 인프라 먼저 구축
- 메트릭 수집 자동화
- 알림 시스템 설정
- 디버깅 도구 준비

---

## 💡 핵심 질문 (답을 찾아야 함)

1. ✅ Isaac Sim 5.0에서 fixed base robot을 어떻게 설정하나?
2. ✅ ArticulationRootAPI와 RigidBodyAPI의 올바른 조합은?
3. ✅ URDF의 `<link name="base_link">`를 어떻게 world에 고정하나?
4. ✅ `isaacsim.core.prims.SingleArticulation` 사용법은?
5. ✅ Physics scene의 권장 설정은?
6. ✅ Visual mesh를 포함한 URDF 변환 방법은?

---

## 📊 메트릭 및 목표

### 성능 목표
| 지표 | 목표 | 현재 상태 |
|------|------|-----------|
| IPC Latency (p95) | < 60ms | 미측정 (정책 추론만 측정) |
| Deadline Miss Rate | < 2% | 완료 (모니터링 시스템 구축) |
| Randomization Hash Mismatch | < 0.1% | 완료 (forensic 시스템 구축) |
| GPU PPO 학습 | 가능 | ✅ 완료 (PyTorch CUDA 12.8) |

### 품질 목표
- ✅ 모든 링크에 CollisionAPI 존재
- ✅ USD 검증 스크립트 통과
- ✅ Physics 시뮬레이션 크래시 없음
- ⚠️ GUI 재생 정상 동작 (미달성)
- ⚠️ 원격 시각화 가능 (미달성)

---

## 🔧 도구 및 스크립트

### 검증 도구
```bash
# USD 구조 검증
python scripts/verify_usd_roarm_m3.py --usd <path>

# Isaac 환경 점검
source scripts/activate_isaacsim_env.sh
python scripts/check_isaac_import.py
bash scripts/isaac_precheck.sh

# IPC 레이턴시 측정
python scripts/measure_round_trip.py
python scripts/bench_transport.py

# Randomization 드리프트 분석
python scripts/analyze_hash_drift.py
```

### 자동화 스크립트
```bash
# CollisionAPI 추가
python scripts/add_collision_api_clean.py

# Ground Plane 추가
python scripts/add_ground_plane.py

# USD 변환
isaac-sim urdf --input <urdf> --output <usd>
```

---

## 📚 참고 문서

### codex_mcp 프로젝트 문서
- `README.md`: 프로젝트 개요 및 최신 업데이트
- `docs/STATUS.md`: 현재 상태 및 TODO 보드
- `docs/CURRENT_STATUS_AND_ISSUES.md`: 이슈 종합 정리
- `docs/comprehensive_analysis_2025-10-15.md`: 종합 분석 보고서
- `docs/ARCH_DECISION_DUAL_ENV.md`: Dual Environment 아키텍처
- `docs/ROADMAP.md`: 프로젝트 로드맵
- `docs/SIM2REAL_GUIDE.md`: Sim2Real 가이드
- `docs/daily/2025-10-14.md`: 최근 일일 로그

### 로그 파일
- `headless_run.log`: Headless 실행 로그
- `logs/train_*.log`: 학습 로그
- `logs/ipc_gateway_events.jsonl`: IPC 이벤트 로그
- `logs/ipc_metrics.json`: 메트릭 통계

---

## 🎯 종합 결론

### 현재 상태
- 🟡 **학습 가능** (Headless, 일부 우회)
- 🔴 **GUI 재생 불가** (PhysX 크래시)
- 🔴 **원격 시각화 불가** (X11 포워딩 한계)

### 핵심 교훈
1. ✅ **USD 파일은 CollisionAPI 필수**
2. ✅ **Isaac Sim 5.0 API 사용 필수** (`isaacsim.core.*`)
3. ✅ **원격 환경은 WebRTC/NICE DCV 필수**
4. ✅ **환경 분리 전략 유효** (Dual Environment)
5. ✅ **단계별 검증 접근 필수** (땜질식 해결 금지)
6. ✅ **관측성 우선 구축** (로깅, 메트릭, 알림)

### 다음 프로젝트 (roarm_isaac_clean)에서 해야 할 일
1. ✅ **자료 수집부터 시작** (Isaac Sim 5.0, RoArm M3)
2. ✅ **올바른 URDF → USD 변환**
3. ✅ **단계별 검증** (각 단계 완료 확인)
4. ✅ **CollisionAPI 포함 USD 생성**
5. ✅ **WebRTC 스트리밍 초기 설정**
6. ✅ **최신 API 사용** (`isaacsim.core.*`)
7. ✅ **관측성 인프라 우선 구축**

---

**작성 완료**: 2025년 10월 15일  
**다음 단계**: 자료 수집 → 올바른 접근 → 단계별 검증
