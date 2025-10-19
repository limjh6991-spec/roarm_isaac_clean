# RoArm-M3 Isaac Sim Integration# RoArm M3 + Isaac Sim 5.0 - Production Ready



**NVIDIA Isaac Sim 5.0 기반 RoArm-M3 로봇 팔 시뮬레이션 및 강화학습 프로젝트****생성일**: 2025년 10월 15일  

**최종 업데이트**: 2025년 10월 19일 (오후)  

[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-5.0-green)](https://developer.nvidia.com/isaac-sim)**상태**: ✅ **프리플라이트 PASS** + 🧠 **온톨로지 구축** + 🎯 **멀티-프리미티브 URDF** + 🤖 **강화학습 환경 완료**  

[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)**목적**: RoArm M3 로봇팔을 Isaac Sim 5.0에서 처음부터 올바르게 설정하고 강화학습 수행

[![RL](https://img.shields.io/badge/RL-Stable--Baselines3-orange)](https://stable-baselines3.readthedocs.io/)

---

---

## 🎉 현재 상태: Pick and Place 강화학습 준비 완료!

## 🎯 프로젝트 개요

### ✅ 완료된 작업

RoArm-M3 로봇 팔을 Isaac Sim에서 시뮬레이션하고, Pick and Place 작업을 강화학습으로 학습합니다.

- [x] **Pick and Place 강화학습 환경** (2025-10-19 오전~오후) 🆕

**주요 기능**:  - ✅ RoArmPickPlaceEnv 환경 구현 (400+ lines)

- ✅ Isaac Sim 5.0 URDF Import (공식 문서 기반)  - ✅ Observation(15) / Action(8) space 정의

- ✅ Pick and Place 환경 구현 (기본 + Isaac Assets)  - ✅ Distance-based + Success bonus reward 설계

- ✅ PPO/SAC 강화학습 알고리즘  - ✅ PPO 학습 스크립트 (Stable-Baselines3)

- ✅ Curriculum Learning 지원  - ✅ TensorBoard 로깅, 체크포인트 자동 저장

- ✅ TensorBoard 모니터링  - ✅ 데모 스크립트로 환경 검증 가능

- ✅ Isaac Assets (YCB 물체, 테이블, Warehouse)  - 🎮 **다음**: 데모 실행 → 학습 시작



---- [x] **멀티-프리미티브 URDF 구현** (2025-10-19 오전) 

  - ✅ STL 추출 문제 근본 해결: 프리미티브 조합(box, cylinder, capsule)으로 형상 구현

## 📁 프로젝트 구조  - ✅ 자동 생성 스크립트 개발 (scripts/generate_multiprim_urdf.py)

  - ✅ 9개 링크, 23개 visual, 10개 collision 프리미티브로 구성

```  - ✅ 파일 크기 10.8 KB (vs. STL 기반 ~50 MB, 4,630배 감소)

roarm_isaac_clean/  - ✅ 100% Isaac Sim 호환, Placement offset 문제 원천 차단

├── envs/                        # RL 환경  - ✅ 그리퍼 비율 조정 (radius 8mm→5mm, length 50mm→35mm)

│   ├── roarm_pick_place_env.py           # 기본 환경

│   └── roarm_pickplace_isaac_assets.py   # Isaac Assets 환경- [x] **URDF 표준화 프로세스 구축** (2025-10-18)

├── scripts/                     # 실행 스크립트  - Isaac Sim 5.0 호환 경로 형식 확립 (file:// → 절대 경로)

│   ├── demo_roarm_fixed.py              # 데모  - mm→m 단위 변환 스케일 적용 (scale="0.001")

│   ├── train_roarm_rl.py                # 기본 RL 학습  - urdf_autopatch_standard.py 자동화 스크립트 개발

│   ├── train_roarm_isaac_assets.py      # Isaac Assets RL 학습  - Joint origin 보존 로직 구현

│   ├── run_train_isaac_assets.sh        # 학습 실행 스크립트  - ✅ **해결**: STL 대신 프리미티브 사용으로 전략 전환

│   └── test_basic_isaac.py              # 기본 테스트

├── docs/                        # 문서- [x] **온톨로지 기반 지식 관리 시스템** (2025-10-18)

│   ├── README.md                        # 문서 인덱스 ⭐  - 문제-솔루션 자동 추적

│   ├── URDF_IMPORT_GUIDE.md             # URDF Import 가이드 ⭐  - SPARQL 질의로 빠른 정보 검색

│   ├── ISAAC_ASSETS_RL_GUIDE.md         # RL 학습 가이드 ⭐  - 재발 문제 자동 감지

│   ├── ISAAC_SIM_PYTHON_GUIDE.md        # Python 스크립팅 가이드 ⭐  - pxr 환경 문제 완전 문서화

│   └── REFERENCES.md                    # 참고 자료

├── assets/                      # URDF/USD/Mesh 파일- [x] **프리플라이트 시스템 구축** (2025-10-17)

│   └── roarm_m3/  - 3단계 자동 검증: System Check, Isaac Extensions, USD Integrity

├── resources/                   # 참고 자료  - 타임아웃 처리, JSON 로그 아카이브, 색상 출력

├── tests/                       # 테스트  - pip 설치 방식 Isaac Sim 환경 완벽 지원

├── devops/                      # DevOps 스크립트  - Isaac 번들 pxr 사용 (pip install usd-core 금지)

├── logs/                        # 작업 로그  

└── goal/                        # 프로젝트 목표- [x] **USD 무결성 검사 강화** (2025-10-17)

```  - Stage metadata 검증 (metersPerUnit, upAxis)

  - PhysX Articulation 검증

---  - Rigid Body/Mass/Collision 검증

  - Joint DriveAPI 검증

## 🚀 빠른 시작  - 경고/실패 구분 및 자동 진단



### 1. 환경 설정- [x] **Isaac Python 환경 자동 설정**

  - pxr 모듈 경로 자동 탐색

```bash  - PYTHONPATH/LD_LIBRARY_PATH 자동 설정

# Isaac Sim 설치 확인  - venv와 표준 설치 모두 지원

ls ~/isaacsim/python.sh

---

# Python 환경 설정

bash scripts/setup_rl_env.sh## 📁 프로젝트 구조

```

```

### 2. 데모 실행 (랜덤 액션)roarm_isaac_clean/

├── README.md                    # 이 파일

```bash├── docs/                        # 문서 모음

# 기본 환경 데모│   ├── LESSONS_LEARNED.md      # 이전 프로젝트에서 배운 교훈

PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/demo_roarm_fixed.py \│   ├── SETUP_GUIDE.md          # 단계별 설정 가이드

  --episodes 3 --steps 100│   ├── DEVOPS_GUIDE.md         # DevOps 인프라 가이드

│   ├── MULTI_PRIMITIVE_URDF_STRATEGY.md  # 멀티-프리미티브 전략 (NEW)

# 화면에 로봇과 환경이 표시되고, 랜덤 액션으로 움직입니다│   └── TROUBLESHOOTING.md      # 문제 해결 가이드

```├── devops/                      # DevOps 인프라

│   ├── preflight_all.sh        # 마스터 프리플라이트 (타임아웃, JSON 로깅)

**예상 출력**:│   ├── preflight/              # 개별 프리플라이트 검사

```│   │   ├── check_system.sh            # GPU, 드라이버, Vulkan, Python

🎮 RoArm-M3 Pick and Place 데모│   │   ├── check_isaac_extensions.py  # Isaac 확장 로드 검사

  Episodes: 3│   │   └── check_usd_integrity.sh     # USD 스키마 무결성 검사

  Max steps per episode: 100│   ├── isaac_python.sh         # pxr 환경 설정 래퍼

│   ├── setup_isaac_python_env.sh      # 환경 변수 자동 설정

📺 Episode 1/3│   └── diagnose_python_env.sh  # 환경 진단 도구

  - Distance to target: 0.449m├── ontology/                    # 온톨로지 시스템 (NEW)

  📊 Step 50:│   ├── roarm_domain.ttl        # 코어 온톨로지 (도메인 개념)

     - Distance to target: 0.460m│   ├── instances/              # 실제 문제/솔루션 데이터

     - Total reward: -229.58│   │   └── pxr_environment_problem.ttl

  📊 Step 100:│   ├── queries/                # SPARQL 질의 모음

     - Total steps: 100│   │   └── diagnostics.sparql

     - Total reward: -459.43│   └── README.md               # 온톨로지 사용 가이드

     - 상태: ❌ Failed├── scripts/                     # Python 스크립트

│   ├── generate_multiprim_urdf.py     # 멀티-프리미티브 URDF 자동 생성 (NEW)

✅ 데모 완료!│   ├── test_multiprim_isaac.py        # Isaac Sim 자동 테스트 (NEW)

```│   ├── convert_urdf_to_usd.py  # URDF → USD 변환

│   ├── ontology/               # 온톨로지 도구

### 3. RL 학습 시작│   │   └── query_ontology.py  # SPARQL 질의 실행

│   ├── setup/                  # 초기 설정 스크립트

```bash│   └── usd/                    # USD 관련 유틸리티

# Isaac Assets 환경으로 PPO 학습│       └── verify_usd_quick.py # USD 빠른 검증

bash scripts/run_train_isaac_assets.sh├── resources/                   # 수집한 자료 모음

│   ├── isaac_sim/              # Isaac Sim 5.0 관련 자료

# 또는 직접 실행│   ├── roarm_m3/               # RoArm M3 관련 자료

PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/train_roarm_isaac_assets.py \│   ├── community/              # 커뮤니티 자료

  --mode train --timesteps 50000 --algo ppo│   └── RESOURCE_INDEX.md       # 자료 검색 인덱스

├── scripts/                     # Python 스크립트

# TensorBoard 모니터링 (별도 터미널)│   ├── convert_urdf_to_usd.py  # URDF → USD 변환

tensorboard --logdir logs/│   ├── setup/                  # 초기 설정 스크립트

```│   └── usd/                    # USD 관련 유틸리티

│       └── verify_usd_quick.py # USD 빠른 검증 (NEW)

---├── assets/                      # 로봇 모델 및 에셋

│   └── roarm_m3/

## 📚 문서│       ├── urdf/               # URDF 파일

│       ├── meshes/             # STL 메시 파일

**모든 문서는 [`docs/`](docs/) 폴더에 있습니다.**│       └── usd/                # USD 파일 (변환 결과)

├── configs/                     # 설정 파일

### 필수 가이드 ⭐│   ├── robot_config.yaml       # 로봇 설정

│   └── training_config.yaml    # 학습 설정

1. **[docs/README.md](docs/README.md)** - 문서 인덱스 및 사용 시나리오├── logs/                        # 로그 디렉토리 (NEW)

2. **[docs/URDF_IMPORT_GUIDE.md](docs/URDF_IMPORT_GUIDE.md)** - URDF Import 완전 가이드│   ├── preflight/              # 프리플라이트 로그 + JSON 아카이브

3. **[docs/ISAAC_ASSETS_RL_GUIDE.md](docs/ISAAC_ASSETS_RL_GUIDE.md)** - RL 학습 완전 가이드│   └── isaac/                  # Isaac Sim 실행 로그

4. **[docs/ISAAC_SIM_PYTHON_GUIDE.md](docs/ISAAC_SIM_PYTHON_GUIDE.md)** - Python 스크립팅 가이드└── tests/                       # 테스트 코드

    └── test_robot_load.py      # 로봇 로딩 테스트

### 시나리오별 가이드```



| 목표 | 읽을 문서 |---

|------|----------|

| 처음 시작 | ISAAC_SIM_PYTHON_GUIDE.md → URDF_IMPORT_GUIDE.md |## 📚 문서 가이드

| RL 학습 시작 | ISAAC_ASSETS_RL_GUIDE.md |

| 커스텀 환경 제작 | ISAAC_ASSETS_RL_GUIDE.md + URDF_IMPORT_GUIDE.md |### ⚠️ CRITICAL: 필독 문서

| 트러블슈팅 | ISAAC_SIM_PYTHON_GUIDE.md (트러블슈팅 섹션) |

1. **`docs/PXR_ENVIRONMENT_GUIDE.md`** 🔴 **최우선 필독**

---   - pxr (USD Python 바인딩) 환경 설정 완전 가이드

   - pip install usd-core 금지 이유 상세 설명

## 🎮 주요 스크립트   - PYTHONPATH + LD_LIBRARY_PATH 설정 방법

   - 트러블슈팅: ModuleNotFoundError, ImportError 해결

### 데모   - **USD 관련 모든 작업 전 반드시 읽어야 함**

```bash

# 랜덤 액션 데모 (환경 검증용)### 필독 문서 (작업 시작 전)

PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/demo_roarm_fixed.py \2. **`docs/DEVOPS_GUIDE.md`** ⭐⭐⭐⭐⭐

  --episodes 3 --steps 100   - DevOps 인프라 완전 가이드

```   - **pxr 모듈 환경 설정 섹션 포함** (⚠️ CRITICAL)

   - 프리플라이트 시스템 상세 설명

### RL 학습   - 로그 관리, 트러블슈팅

```bash

# PPO 학습 (기본 환경)3. **`docs/LESSONS_LEARNED.md`** ⭐⭐⭐⭐⭐

PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/train_roarm_rl.py \   - 이전 프로젝트에서 배운 교훈

  --mode train --timesteps 50000 --algo ppo   - **핵심 문제 4가지**: USD CollisionAPI, 원격 GUI, API 변경, **pxr 환경 설정**

   - 반드시 읽어야 하는 최우선 문서

# PPO 학습 (Isaac Assets 환경)

PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/train_roarm_isaac_assets.py \4. **`docs/ISSUES_AND_SOLUTIONS.md`** ⭐⭐⭐⭐

  --mode train --timesteps 50000 --algo ppo   - 8개 이슈 상세 분석 (P0-P3 우선순위)

```   - 각 이슈의 해결 방안 (코드 포함)

   - 문제 발생 시 참조

### 테스트

```bash5. **`docs/ENVIRONMENT_STATUS.md`** ⭐⭐⭐

# 기본 Isaac Sim 테스트   - 현재 시스템 환경 상태

PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_basic_isaac.py   - 하드웨어/소프트웨어 스펙

```   - 확인 명령어 모음



---6. **`docs/PROJECT_RESTART_SUMMARY.md`** ⭐⭐⭐⭐⭐

   - 프로젝트 재시작 종합 보고서

## 🔧 핵심 기술   - 단계별 액션 플랜

   - 타임라인 및 체크리스트

### Isaac Sim 5.0 URDF Import

```python### 참고 문서 (필요시)

from isaacsim.asset.importer.urdf import _urdf- `~/codex_mcp/README.md` - 이전 프로젝트 개요

import omni.kit.commands- `~/codex_mcp/docs/STATUS.md` - 이전 프로젝트 최신 상태

- `~/codex_mcp/docs/comprehensive_analysis_2025-10-15.md` - 종합 분석

import_config = _urdf.ImportConfig()

success, prim_path = omni.kit.commands.execute(---

    "URDFParseAndImportFile",

    urdf_path=urdf_path,## 🚀 빠른 시작

    import_config=import_config,

    get_articulation_root=True,  # ⭐ 필수!### 1. 프리플라이트 검사 실행

)```bash

```# 전체 프리플라이트 (시스템, 확장, USD 무결성)

bash devops/preflight_all.sh

### RL 환경 구현

```python# 개별 검사

from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfgbash devops/preflight/check_system.sh

python devops/preflight/check_isaac_extensions.py

cfg = RoArmPickPlaceEnvCfg()bash devops/preflight/check_usd_integrity.sh assets/roarm_m3/usd/roarm_m3.usd

env = RoArmPickPlaceEnv(cfg)```



obs = env.reset()**성공 시 출력**:

for step in range(100):```

    action = policy(obs)  # 또는 랜덤 액션========================================================================

    obs, reward, done, info = env.step(action)Total: 3  PASS:3  FAIL:0  SKIP:0

```ALL PREFLIGHT CHECKS PASSED!

System ready for Isaac Sim development.

---```



## 📊 현재 상태### 2. 멀티-프리미티브 URDF 생성 및 테스트 🆕

```bash

| 항목 | 상태 |# URDF 생성 (자동화 스크립트)

|------|------|python scripts/generate_multiprim_urdf.py --output assets/roarm_m3/urdf/roarm_m3_multiprim.urdf

| URDF Import | ✅ 완료 (Isaac Sim 5.0) |

| 기본 환경 | ✅ 완료 |# 출력:

| Isaac Assets 환경 | ✅ 완료 |# ✅ URDF 생성 완료

| 데모 실행 | ✅ 완료 (3 episodes × 100 steps) |# 📊 통계:

| RL 학습 스크립트 | ✅ 완료 (PPO/SAC) |#   - 총 링크: 9개

| 문서화 | ✅ 완료 |#   - 총 조인트: 9개

| Easy Mode 학습 | 🟡 진행 중 |#   - Visual 프리미티브: 23개

#   - Collision 프리미티브: 10개

---#   - 파일 크기: 10.8 KB



## 🐛 트러블슈팅# Isaac Sim 자동 테스트

~/isaac-sim.sh -m scripts/test_multiprim_isaac.py

### 문제: URDF import 실패

**해결**: [docs/URDF_IMPORT_GUIDE.md](docs/URDF_IMPORT_GUIDE.md) 참고# 수동 테스트 (Isaac Sim GUI)

~/isaac-sim.sh &

### 문제: Python 스크립트 출력이 안 보임# File → Import → URDF → 경로: /home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_multiprim.urdf

**해결**: `PYTHONUNBUFFERED=1` 환경 변수 사용  # Joint 슬라이더로 동작 확인

자세한 내용: [docs/ISAAC_SIM_PYTHON_GUIDE.md](docs/ISAAC_SIM_PYTHON_GUIDE.md)```



### 문제: 학습이 안 됨**멀티-프리미티브 방식 특징**:

**해결**: [docs/ISAAC_ASSETS_RL_GUIDE.md](docs/ISAAC_ASSETS_RL_GUIDE.md)의 트러블슈팅 섹션- ✅ STL 파일 불필요 (Placement offset 문제 원천 차단)

- ✅ 파일 크기 10.8 KB (vs. STL 기반 ~50 MB)

---- ✅ 100% Isaac Sim 네이티브 호환

- ✅ 물리 시뮬레이션 안정성 향상

## 📖 참고 자료- ✅ 링크당 3-8개 프리미티브로 실루엣 재현



- [Isaac Sim 공식 문서](https://docs.omniverse.nvidia.com/isaacsim/latest/)### 3. URDF → USD 변환

- [Stable Baselines3 문서](https://stable-baselines3.readthedocs.io/)```bash

- [URDF 명세](http://wiki.ros.org/urdf)source ~/isaacsim-venv/bin/activate

- [더 많은 자료](docs/REFERENCES.md)python scripts/convert_urdf_to_usd.py

```

---

### 4. USD 빠른 검증

## 📝 주요 변경 사항```bash

python scripts/usd/verify_usd_quick.py assets/roarm_m3/usd/roarm_m3.usd

### 2025-10-19```

- ✅ Isaac Sim 5.0 URDF Import 문제 해결

  - `omni.isaac.urdf` → `isaacsim.asset.importer.urdf`### 5. 온톨로지 지식 검색 🆕

  - `get_articulation_root=True` 파라미터 추가```bash

- ✅ stdout buffering 문제 해결 (`PYTHONUNBUFFERED=1`)# rdflib, networkx, matplotlib 설치 (최초 1회)

- ✅ 데모 실행 성공 (3 episodes × 100 steps)pip install rdflib networkx matplotlib

- ✅ 프로젝트 정리 (312 → 188 파일, -40%)

- ✅ 완전한 문서화 완료# 프로젝트 상태 조회

python scripts/ontology/query_ontology.py --query project_status

---

# pxr 문제 솔루션 찾기

## 🎯 다음 단계python scripts/ontology/query_ontology.py --query pxr_solutions



1. **Easy Mode RL 학습** (진행 중)# 사용 가능한 질의 목록

   - PPO 50K timestepspython scripts/ontology/query_ontology.py --list-queries

   - 성공률 60%+ 목표

# 지식 그래프 시각화 (PNG 생성)

2. **Curriculum Learning 적용**python scripts/ontology/visualize_graph.py --focus pxr_environment_problem --output pxr_graph.png

   - 3단계: Easy → Medium → Hardpython scripts/ontology/visualize_graph.py --output full_graph.png



3. **성능 최적화**# 브라우저로 시각화 보기 (추천!) 🆕

   - 하이퍼파라미터 튜닝python scripts/ontology/view_ontology.py

   - 다양한 RL 알고리즘 실험# → http://localhost:8000 자동으로 열림

# → VS Code Simple Browser에서도 확인 가능

---```



## 📧 문의**생성된 시각화**:

- 📊 **pxr_subgraph.png**: pxr 문제 중심 서브그래프 (17노드, 29엣지)

- GitHub Issues- 📊 **full_ontology_graph.png**: 전체 지식 그래프 (128노드, 127엣지)

- [docs/README.md](docs/README.md)의 트러블슈팅 섹션 참고

**VS Code에서 이미지 보기**:
```bash
# VS Code에서 파일 탐색기 열기 → docs/ontology/ 폴더
# pxr_subgraph.png 또는 full_ontology_graph.png 클릭
# 또는 터미널에서:
code docs/ontology/pxr_subgraph.png
code docs/ontology/full_ontology_graph.png
```

### 6. GUI로 USD 확인
```bash
# (작업 예정 - GUI 래퍼 스크립트 추가 예정)
```

---

## 🧠 온톨로지 시스템 (NEW)

### 개요

**온톨로지(Ontology)**는 프로젝트의 지식을 구조화하여 관리하는 시스템입니다.

**핵심 가치**:
- ✅ 문제 발생 시 기존 솔루션 즉시 검색 (10분 → 10초)
- ✅ 재발 문제 자동 감지 및 과거 해결책 제공
- ✅ 문제-솔루션-문서 관계 명시적 연결
- ✅ SPARQL 질의로 복잡한 지식 탐색

### 빠른 사용

```bash
# pxr 환경 문제 솔루션 찾기
python scripts/ontology/query_ontology.py --query pxr_solutions

# 재발 문제 확인
python scripts/ontology/query_ontology.py --query recurring_problems

# 프로젝트 전체 상태
python scripts/ontology/query_ontology.py --query project_status
```

### 예제 출력

```
✅ pxr 문제 해결책:

1. isaac_python.sh 래퍼 스크립트
   성공률: 100%
   경로: /home/roarm_m3/roarm_isaac_clean/devops/isaac_python.sh

2. 수동 PYTHONPATH/LD_LIBRARY_PATH 설정
   성공률: 80%
```

**자세한 내용**: `ontology/README.md` 참조

---

## 📊 프리플라이트 시스템

### 마스터 프리플라이트
- **경로**: `devops/preflight_all.sh`
- **기능**:
  - 3단계 검사를 순차 실행 (System → Extensions → USD)
  - 각 단계별 타임아웃 설정 (30s/120s/45s)
  - 실패해도 계속 진행 (모든 이슈 수집)
  - JSON 로그 자동 아카이브 (`logs/preflight/*.json`)
  - 색상 코드 출력 (PASS=녹색, FAIL=빨강, SKIP=노랑)

### 개별 프리플라이트 검사

#### 1. System Check (`devops/preflight/check_system.sh`)
- NVIDIA 드라이버 (≥550)
- GPU 메모리
- Vulkan 지원
- Python 버전 (3.10 또는 3.11)
- 환경 변수 (CUDA_VISIBLE_DEVICES, VK_ICD_FILENAMES)

#### 2. Isaac Extensions (`devops/preflight/check_isaac_extensions.py`)
- SimulationApp headless 부팅 (5초 이내)
- 필수 확장 로드 확인:
  - `isaacsim.asset.importer.urdf`
  - `isaacsim.core.api`
  - `omni.isaac.core`
  - `omni.physx`
  - `omni.usd`

#### 3. USD Integrity (`devops/preflight/check_usd_integrity.sh`)
- Isaac 번들 pxr 자동 탐색 및 사용
- Stage metadata (metersPerUnit=1.0, upAxis=Z)
- PhysX Articulation Root (정확히 1개, 중첩 금지)
- Rigid Body MassAPI 존재
- Collision 메시 존재
- Joint DriveAPI 바인딩

---

## 🎯 다음 단계

### Phase 3: GUI 테스트 및 시각적 검증
- [ ] GUI 래퍼 스크립트 작성 (`devops/run_isaac_supervised.sh`)
- [ ] USD 파일 시각적 검증 (STL 메시, 조인트 동작)
- [ ] Collision 메시 표시 확인
- [ ] 물리 시뮬레이션 테스트

### Phase 4: URDF → USD 변환 개선
- [ ] Collision 메시 자동 생성
- [ ] JointDriveAPI 자동 적용
- [ ] MassAPI 자동 계산
- [ ] 변환 스크립트 리팩토링

### Phase 5: 강화학습 파이프라인
- [ ] Gymnasium 환경 구현
- [ ] Observation/Action space 정의
- [ ] Reward 함수 설계
- [ ] PPO 학습 실행

---

## ⚠️ 주의사항

### ❌ 피해야 할 것
1. 검증 없는 USD 파일 사용
2. 구형 API (`omni.isaac.core.*`) 사용
3. X11 포워딩으로 GUI 렌더링 시도
4. 땜질식 문제 해결
5. 단계 건너뛰기

### ✅ 반드시 해야 할 것
1. USD 검증 스크립트 항상 실행
2. 최신 API (`isaacsim.core.*`) 사용
3. WebRTC 또는 Headless 모드 사용
4. 각 단계 완료 후 다음 단계 진행
5. 모든 결정과 문제 문서화

---

## 📞 도움 요청

**Phase 2 (자료 수집) 진행 전 확인 요청 부탁드립니다.**

현재 상태:
- ✅ Phase 1 완료 (환경 분석)
- ⏳ Phase 2 준비 완료 (자료 수집 대기)

다음 단계를 진행하시겠습니까?

---

## 📋 현재 상태

### Phase 1: 환경 분석 완료 ✅

- [x] 프로젝트 폴더 생성
- [x] 이전 프로젝트(codex_mcp) 분석 완료
- [x] 주요 이슈 및 교훈 문서화
- [x] 환경 상태 파악 및 기록
- [x] 해결 방안 정리

**생성된 문서:**
- `docs/LESSONS_LEARNED.md` - 이전 프로젝트에서 배운 교훈
- `docs/ENVIRONMENT_STATUS.md` - 현재 환경 상태 및 설정
- `docs/ISSUES_AND_SOLUTIONS.md` - 주요 이슈 및 해결 방안

### Phase 2: 자료 수집 완료 ✅

- [x] Isaac Sim 5.0 공식 문서 조사
- [x] RoArm M3 공식 자료 수집 (Wiki, 스펙, 다운로드)
- [x] 커뮤니티 자료 검색 (Forums, Reddit, GitHub)
- [x] REFERENCES.md 작성 완료

**수집된 자료:**
- `resources/roarm_m3/waveshare_wiki_summary.md` - RoArm M3 Wiki 요약
- `resources/isaac_sim/documentation_links.md` - Isaac Sim 문서 링크
- `resources/community/isaac_sim_resources.md` - 커뮤니티 자료
- `docs/REFERENCES.md` - 전체 참고 자료 정리

**주요 발견:**
- ✅ RoArm M3는 ROS2 지원 (URDF 존재 가능성 높음)
- ✅ STEP 3D 모델, 2D 치수도 다운로드 가능
- ✅ LeRobot 통합 지원
- ⚠️ Isaac Sim 5.0 공식 문서 URL 변경 (로컬 확인 필요)

### 다음 단계: Phase 3 USD 파일 생성 ⏳
- [ ] 디렉토리 구조 구축
- [ ] 이전 프로젝트 분석 문서 작성
- [ ] Isaac Sim 5.0 자료 수집
- [ ] RoArm M3 자료 수집
- [ ] 기본 설정 가이드 작성

---

## 🚀 다음 단계

### Phase 1: 준비 및 자료 수집 (오늘)
1. 이전 프로젝트(codex_mcp) 분석 및 교훈 문서화
2. Isaac Sim 5.0 공식 문서 수집
3. RoArm M3 공식 자료 수집
4. 커뮤니티 자료 수집 (Reddit, GitHub)
5. Resources 폴더 구축 및 인덱싱

### Phase 2: 기초 설정 (내일)
1. URDF 검증 및 수정
2. USD 변환 (올바른 방법)
3. Isaac Sim에서 로봇 로딩 테스트
4. 기본 물리 시뮬레이션 확인

### Phase 3: 강화학습 환경 구축
1. Gym 환경 구현
2. 보상 함수 설계
3. PPO 학습 파이프라인 구축

### Phase 4: 원격 렌더링 및 배포
1. WebRTC 스트리밍 설정
2. 학습된 정책 테스트
3. 문서화 및 정리

---

## 📚 참고 자료

### Isaac Sim 5.0
- 공식 문서: https://docs.omniverse.nvidia.com/isaacsim/latest/
- API 레퍼런스: (수집 예정)
- 마이그레이션 가이드: (수집 예정)

### RoArm M3
- 공식 사이트: (수집 예정)
- GitHub: (수집 예정)
- URDF: (수집 예정)

### 커뮤니티
- Reddit r/IsaacSim: (수집 예정)
- GitHub Discussions: (수집 예정)

---

## ⚠️ 이전 프로젝트(codex_mcp)에서 배운 교훈

자세한 내용은 `docs/LESSONS_LEARNED.md` 참조

**핵심 문제:**
1. CollisionAPI 누락 → PhysX 크래시
2. ArticulationAPI 설정 오류 → 로봇 인식 실패
3. Fixed Joint vs Kinematic Body 혼동
4. URDF → USD 변환 과정의 불완전성

**피해야 할 것:**
- 검증 없이 USD 변환
- Physics API 설정 건너뛰기
- 테스트 없이 복잡한 기능 추가
- 문서화 부족

---

## 💡 핵심 원칙

1. **단계별 검증**: 각 단계마다 철저히 테스트
2. **문서 우선**: 모든 결정과 문제를 문서화
3. **공식 문서 참조**: 추측하지 말고 공식 문서 확인
4. **간단하게 시작**: 복잡한 기능은 나중에 추가

---


**작성**: 2025-10-15  
**업데이트**: 진행 상황에 따라 지속 업데이트
