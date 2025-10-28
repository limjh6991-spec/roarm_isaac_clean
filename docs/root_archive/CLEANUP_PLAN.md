# 프로젝트 정리 계획

**작성일**: 2025-10-19  
**목적**: 불필요한 파일 삭제 및 프로젝트 구조 정리

---

## 📋 현재 상태 분석

### 1. **scripts/** (58개 파일) ⚠️ 정리 필요
**유지할 파일 (핵심)**:
- ✅ `demo_roarm_fixed.py` - 데모 스크립트 (최신, 작동 확인)
- ✅ `train_roarm_rl.py` - 기본 RL 학습
- ✅ `train_roarm_isaac_assets.py` - Isaac Assets RL 학습
- ✅ `convert_urdf_to_usd.py` - URDF → USD 변환
- ✅ `test_basic_isaac.py` - Isaac Sim 기본 테스트
- ✅ `run_train_isaac_assets.sh` - 학습 실행 스크립트
- ✅ `setup_rl_env.sh` - 환경 설정

**삭제할 파일 (구버전/중복/테스트)**:
- ❌ `demo_roarm_env.py` - 구버전 (demo_roarm_fixed.py로 대체)
- ❌ `demo_simple_isaac_assets.py` - 임시 테스트
- ❌ `run_demo.sh` - 구버전 스크립트
- ❌ `run_train.sh` - 구버전 스크립트
- ❌ `test_*.py` (10개) - 디버깅용 임시 파일
- ❌ `launch_gui.py` - 사용 안 함
- ❌ STEP/STL/URDF 변환 관련 (14개) - URDF 생성 완료, 더 이상 필요 없음
  - `convert_step_to_stl.py`
  - `export_stl_*.py` (3개)
  - `center_stl_files.py`
  - `fix_stl_center.sh`
  - `split_complete_stl.py`
  - `apply_transform_to_stl.py`
  - `analyze_*.py` (2개)
  - `extract_*.py` (2개)
  - `freecad_*.py` (2개)
  - `generate_multiprim_urdf.py`
  - `patch_urdf_with_merged_stls.py`
- ❌ URDF validation 관련 (8개) - 검증 완료
  - `validate_urdf.py`
  - `verify_*.py` (4개)
  - `urdf_autopatch_*.py` (2개)
  - `fix_urdf_paths_absolute.py`
  - `run_urdf_standardization.sh`
- ❌ import 테스트 (2개)
  - `import_urdf_to_isaac.py`
  - `import_and_merge_urdf.py`
- ❌ ontology 스크립트 (4개) - Phase 2 완료, 필요 없음
  - `scripts/ontology/*`

**정리 후**: 7개 유지, 51개 삭제

---

### 2. **docs/** (30개 파일) ⚠️ 정리 필요
**유지할 파일 (핵심 가이드)**:
- ✅ `URDF_IMPORT_GUIDE.md` - URDF import 공식 가이드
- ✅ `ISAAC_ASSETS_RL_GUIDE.md` - RL 학습 완전 가이드
- ✅ `ISAAC_SIM_PYTHON_GUIDE.md` - Python 스크립팅 가이드
- ✅ `REFERENCES.md` - 참고 자료
- ✅ `README.md` (새로 작성 예정)

**삭제할 파일 (구버전/Phase 1-2 관련)**:
- ❌ `CURRENT_ISSUE_ANALYSIS.md` - 임시 분석 문서
- ❌ `ENVIRONMENT_STATUS.md` - 구버전
- ❌ `ISSUES_AND_SOLUTIONS.md` - 구버전
- ❌ `LESSONS_LEARNED.md` - 구버전
- ❌ `PHASE_DECISION.md` - Phase 2 완료
- ❌ `PHASE2_SUMMARY.md` - Phase 2 완료
- ❌ `PROJECT_RESTART_SUMMARY.md` - 구버전
- ❌ `WARMUP_SESSION_20251018.md` - 임시 문서
- ❌ STEP/STL/URDF 관련 (10개) - 더 이상 필요 없음
  - `FREECAD_*.md` (2개)
  - `MESH_ORIGIN_FIX.md`
  - `MULTI_PRIMITIVE_URDF_STRATEGY.md`
  - `JARVIS_URDF_SOLUTION.md`
  - `STEP_*.md` (2개)
  - `URDF_STANDARD_GUIDE.md`
  - `URDF_TO_USD_GUI_GUIDE.md`
  - `RoArm_M3_Link_STL_Status_v1.md`
- ❌ `DEVOPS_GUIDE.md` - devops 폴더로 통합
- ❌ `PXR_ENVIRONMENT_GUIDE.md` - 사용 안 함
- ❌ `ISAAC_ASSETS_IMPLEMENTATION.md` - RL_GUIDE로 통합
- ❌ ontology 관련 (5개) - Phase 2 완료
  - `docs/ontology/*`

**정리 후**: 5개 유지, 25개 삭제

---

### 3. **logs/** (30개 파일) ⚠️ 정리 필요
**유지할 파일**:
- ✅ `rl_implementation_20251019.md` - 최신 작업 로그

**삭제할 파일**:
- ❌ `urdf_standardization_20251018.md` - 구버전
- ❌ `freecad_*.log` (1개)
- ❌ `batch_export_*.log` (2개)
- ❌ `preflight/*` (26개) - Phase 1 디버깅 로그

**정리 후**: 1개 유지, 29개 삭제

---

### 4. **ontology/** (9개 파일) ❌ 폴더 전체 삭제
**이유**: Phase 2 완료, RL 학습에서 사용 안 함
- ❌ `ontology/*` 전체 삭제

---

### 5. **devops/** (9개 파일) ⚠️ 정리 필요
**유지할 파일**:
- ✅ `isaac_python.sh` - Isaac Sim Python wrapper
- ✅ `setup_isaac_python_env.sh` - 환경 설정

**삭제할 파일**:
- ❌ `preflight/*` (4개) - Phase 1 디버깅
- ❌ `diagnose_python_env.sh` - 디버깅 완료
- ❌ `preflight_all.sh` - 디버깅 완료
- ❌ `run_isaac_supervised.sh` - 사용 안 함

**정리 후**: 2개 유지, 7개 삭제

---

### 6. **envs/** (6개 파일) ✅ 유지
**모두 필요**:
- ✅ `roarm_pick_place_env.py` - 기본 환경
- ✅ `roarm_pickplace_isaac_assets.py` - Isaac Assets 환경
- ✅ `__init__.py`
- ✅ `README.md`
- ✅ `__pycache__/*` (자동 생성)

---

### 7. **assets/** (137개 파일) ✅ 유지
**URDF/USD/Mesh 파일 - 모두 필요**

---

### 8. **configs/** (0개 파일) ⚠️
**상태**: 비어있음
**조치**: 폴더 삭제 또는 RL config 추가 예정

---

### 9. **resources/** (28개 파일) ✅ 대부분 유지
**유지**:
- ✅ `RESOURCE_INDEX.md`
- ✅ `community/*`
- ✅ `isaac_sim/*`
- ✅ `roarm_m3/waveshare_wiki_summary.md`

**삭제 가능**:
- ❓ `roarm_m3/*.zip` (3개) - 압축 해제 완료, 백업용으로 유지?
- ❓ `roarm_m3/*.step` - STL 변환 완료, 백업용으로 유지?

**조치**: 백업용으로 유지

---

### 10. **tests/** (3개 파일) ⚠️ 정리 필요
**유지할 파일**:
- ✅ `test_urdf_validation.py` - URDF 검증 유틸리티

**삭제할 파일**:
- ❌ `test_kit_boot.py` - Phase 1 디버깅
- ❌ `__pycache__/*`

**정리 후**: 1개 유지, 2개 삭제

---

### 11. **goal/** (1개 파일) ✅ 유지
- ✅ `Pepper_One_Gochu_Harvester_v0.1_2025-10-18.md`

---

### 12. **기타 폴더** ❌ 삭제
- ❌ `스크린샷/` - 임시 폴더

---

## 📊 정리 요약

| 폴더 | 현재 | 유지 | 삭제 | 비율 |
|------|------|------|------|------|
| scripts/ | 58 | 7 | 51 | -88% |
| docs/ | 30 | 5 | 25 | -83% |
| logs/ | 30 | 1 | 29 | -97% |
| ontology/ | 9 | 0 | 9 | -100% |
| devops/ | 9 | 2 | 7 | -78% |
| envs/ | 6 | 6 | 0 | 0% |
| assets/ | 137 | 137 | 0 | 0% |
| configs/ | 0 | 0 | 0 | - |
| resources/ | 28 | 28 | 0 | 0% |
| tests/ | 3 | 1 | 2 | -67% |
| goal/ | 1 | 1 | 0 | 0% |
| 기타 | 1 | 0 | 1 | -100% |
| **합계** | **312** | **188** | **124** | **-40%** |

---

## 🎯 정리 후 최종 구조

```
roarm_isaac_clean/
├── README.md                    # 프로젝트 개요 (새로 작성)
├── envs/                        # RL 환경 (6개)
│   ├── roarm_pick_place_env.py
│   └── roarm_pickplace_isaac_assets.py
├── scripts/                     # 핵심 스크립트 (7개)
│   ├── demo_roarm_fixed.py
│   ├── train_roarm_rl.py
│   ├── train_roarm_isaac_assets.py
│   ├── convert_urdf_to_usd.py
│   ├── test_basic_isaac.py
│   ├── run_train_isaac_assets.sh
│   └── setup_rl_env.sh
├── docs/                        # 핵심 문서 (5개)
│   ├── README.md
│   ├── URDF_IMPORT_GUIDE.md
│   ├── ISAAC_ASSETS_RL_GUIDE.md
│   ├── ISAAC_SIM_PYTHON_GUIDE.md
│   └── REFERENCES.md
├── assets/                      # URDF/USD/Mesh (137개)
│   └── roarm_m3/
├── resources/                   # 참고 자료 (28개)
├── tests/                       # 테스트 (1개)
│   └── test_urdf_validation.py
├── devops/                      # DevOps 스크립트 (2개)
│   ├── isaac_python.sh
│   └── setup_isaac_python_env.sh
├── logs/                        # 작업 로그 (1개)
│   └── rl_implementation_20251019.md
└── goal/                        # 프로젝트 목표 (1개)
    └── Pepper_One_Gochu_Harvester_v0.1_2025-10-18.md
```

---

## ✅ 실행 단계

1. **백업 생성** (안전을 위해)
2. **파일 삭제 스크립트 생성**
3. **삭제 실행**
4. **docs/README.md 작성** (프로젝트 구조 설명)
5. **루트 README.md 업데이트**
6. **Git commit**

---

## 🚨 주의사항

- `.git/`, `.venv/`, `.vscode/` 폴더는 그대로 유지
- `assets/` 폴더의 URDF/USD/Mesh 파일은 절대 삭제 금지
- 삭제 전 백업 생성
