# 프로젝트 정리 및 백업 계획

**작성일**: 2025-11-02  
**목적**: Vision RL 구현 전 프로젝트 최소화 및 백업

---

## 백업 대상

### 1. 보관할 핵심 파일 (백업 안 함)

#### Python 소스 코드
```
envs/
├── __init__.py
├── roarm_pick_place_env.py          # v4.2 현재 환경
├── observation/
├── reward/
└── robot/

scripts/
├── train/
│   └── train_v4_2_quick.py         # v4.2 학습 스크립트
├── test/
│   ├── view_home_position.py       # 홈 포지션 뷰어
│   └── view_trained_model.py       # 모델 시각화
└── utils/
    └── urdf_to_usd.py              # URDF 변환

test_v3_7_2.py                       # 테스트 스크립트 (루트)
```

#### 문서 (핵심만 보관)
```
docs/
├── vision_rl/
│   ├── ALGORITHM_COMPARISON.md      # Vision RL 알고리즘 비교
│   └── CAMERA_INTEGRATION_GUIDE.md  # 카메라 통합 가이드
├── PHASE2_VISION_BASED_RL_PLAN.md   # Phase 2 로드맵
└── README.md

README.md                             # 프로젝트 메인 README
DOCUMENTATION_INDEX.md                # 문서 인덱스
```

#### 설정 파일
```
.env
.env.example
.gitignore
.vscode/
Makefile
```

#### 리소스 (외부 자료)
```
resources/                            # 전체 보관 (Vision RL 자료)
├── vision_rl/
├── grippers/
│   └── realsense-ros/               # RealSense ROS2 패키지
└── community/
```

#### Assets
```
assets/
├── roarm_m3/
│   ├── urdf/                        # URDF 파일들
│   ├── meshes/                      # 메시 파일들
│   └── usd/                         # USD 파일들
└── objects/                         # 물체 모델
```

---

### 2. 백업할 파일 (backup/ 폴더로 이동)

#### 오래된 문서 (docs/)
```
docs/
├── 10M_TRAINING_DETAILED_ANALYSIS.md
├── DAILY_LOG_20251020.md
├── ISAAC_ASSETS_RL_GUIDE.md
├── ISAAC_SIM_PYTHON_GUIDE.md
├── ISSUE_ANALYSIS_20251019.md
├── PHOBOS_SETUP_GUIDE.md
├── PROJECT_RESTRUCTURE_PROPOSALS.md
├── RESTRUCTURE_QUICK_GUIDE.md
├── TODO_20251020.md
├── TODO_20251022.md
├── TRAINING_LOG_20251019.md
├── TRAINING_PROGRESS.md
├── TRAINING_SUMMARY.md
├── URDF_ENHANCEMENT_COMPLETE.md
├── URDF_ENHANCEMENT_REPORT.md
├── URDF_IMPORT_GUIDE.md
├── URDF_VIEWER_GUI_GUIDE.md
├── URDF_VIEWER_USAGE.md
├── URDF_VISUALIZATION_COMPLETE.md
├── VISUALIZATION_ISSUE_ANALYSIS.md
├── daily_log_2025-10-24.md
├── env_fix_v2.md
├── joint_comparison_analysis.md
├── learning_analysis_grip_issue.md
├── observation_space.md
├── quick_test_results.md
├── rl_training_ready.md
├── train_script_improvements.md
├── training_100k_results.md
├── usd_schema_check_result.md
├── v3.1_grip_relaxation_complete.md
├── v3.5_CHANGELOG.md
├── v3.8.0_training_status.md
├── v3.8.1_grip_strengthening_plan.md
├── v3.9.0_code_analysis_and_refactoring.md
├── v3.9.0_implementation_report.md
├── v3.9.0_robust_grip_plan.md
├── v3.9.1_attach_fix_plan.md
├── v3.9.2_attach_condition_relaxation.md
├── v3.9.2_training_analysis.md
├── v3.9.3_training_analysis.md
├── v3.9.6_file_refactoring_plan.md
└── video_creation_guide.md
```

#### 사용하지 않는 스크립트 (scripts/)
```
scripts/
├── debug/                          # 디버그 스크립트들
├── env/                            # 환경 테스트 스크립트들
├── env_setup/                      # 환경 설정 스크립트들
├── rl/                             # RL 관련 스크립트들
├── urdf/                           # URDF 관련 스크립트들 (일부)
└── verification/                   # 검증 스크립트들
```

#### 기타
```
archive/                             # 이미 아카이브된 파일들
_backup_20251019/                    # 이전 백업
README.md.backup                     # README 백업
cleanup.sh                           # 정리 스크립트
monitor_training.sh                  # 모니터링 스크립트
view_urdf.sh                        # URDF 뷰어 스크립트
dense_reward_training_backup_20251019_174606.zip
```

---

## 백업 구조

```
backup_20251102_vision_rl_prep/
├── docs/                           # 오래된 문서들
├── scripts/                        # 사용하지 않는 스크립트들
├── archive/                        # 기존 아카이브
├── _backup_20251019/              # 이전 백업
└── misc/                          # 기타 파일들
    ├── README.md.backup
    ├── cleanup.sh
    ├── monitor_training.sh
    └── view_urdf.sh
```

---

## 최종 프로젝트 구조 (정리 후)

```
roarm_isaac_clean/
├── .env
├── .env.example
├── .gitignore
├── .vscode/
├── Makefile
├── README.md
├── DOCUMENTATION_INDEX.md
├── test_v3_7_2.py
│
├── assets/                         # URDF, Meshes, USD
│   ├── roarm_m3/
│   └── objects/
│
├── configs/                        # 설정 파일
│
├── envs/                          # RL 환경 (핵심)
│   ├── roarm_pick_place_env.py
│   ├── observation/
│   ├── reward/
│   └── robot/
│
├── docs/                          # 핵심 문서만
│   ├── vision_rl/
│   │   ├── ALGORITHM_COMPARISON.md
│   │   └── CAMERA_INTEGRATION_GUIDE.md
│   ├── PHASE2_VISION_BASED_RL_PLAN.md
│   ├── PHASE2_SUMMARY.md
│   ├── REFERENCES.md
│   └── README.md
│
├── logs/                          # 학습 로그
│
├── resources/                     # 외부 자료 (보관)
│   ├── vision_rl/
│   ├── grippers/
│   └── community/
│
├── scripts/                       # 핵심 스크립트만
│   ├── train/
│   │   └── train_v4_2_quick.py
│   ├── test/
│   │   ├── view_home_position.py
│   │   └── view_trained_model.py
│   └── utils/
│       └── urdf_to_usd.py
│
├── tests/                         # 테스트 코드
│
└── backup_20251102_vision_rl_prep/  # 백업 폴더
    ├── docs/
    ├── scripts/
    ├── archive/
    └── misc/
```

---

## 실행 계획

### Step 1: 백업 폴더 생성
```bash
mkdir -p backup_20251102_vision_rl_prep/{docs,scripts,misc}
```

### Step 2: 문서 백업
```bash
cd /home/roarm_m3/roarm_isaac_clean
mv docs/10M_TRAINING_DETAILED_ANALYSIS.md backup_20251102_vision_rl_prep/docs/
mv docs/DAILY_LOG_20251020.md backup_20251102_vision_rl_prep/docs/
# ... (모든 오래된 문서)
```

### Step 3: 스크립트 백업
```bash
mv scripts/debug/ backup_20251102_vision_rl_prep/scripts/
mv scripts/env/ backup_20251102_vision_rl_prep/scripts/
mv scripts/env_setup/ backup_20251102_vision_rl_prep/scripts/
mv scripts/rl/ backup_20251102_vision_rl_prep/scripts/
mv scripts/urdf/ backup_20251102_vision_rl_prep/scripts/
mv scripts/verification/ backup_20251102_vision_rl_prep/scripts/
```

### Step 4: 기타 파일 백업
```bash
mv archive/ backup_20251102_vision_rl_prep/
mv _backup_20251019/ backup_20251102_vision_rl_prep/
mv README.md.backup backup_20251102_vision_rl_prep/misc/
mv cleanup.sh backup_20251102_vision_rl_prep/misc/
mv monitor_training.sh backup_20251102_vision_rl_prep/misc/
mv view_urdf.sh backup_20251102_vision_rl_prep/misc/
mv dense_reward_training_backup_20251019_174606.zip backup_20251102_vision_rl_prep/misc/
```

### Step 5: 백업 압축 (선택사항)
```bash
tar -czf backup_20251102_vision_rl_prep.tar.gz backup_20251102_vision_rl_prep/
```

---

## 보관 기준

✅ **보관**:
- 현재 사용 중인 v4.2 코드
- Vision RL 관련 문서 (Phase 2)
- 외부 자료 (resources/)
- Assets (URDF, Meshes)
- 설정 파일

⏸️ **백업**:
- v3.x 관련 문서 (10M 이전)
- 사용하지 않는 스크립트
- 이전 백업 폴더
- 정리 스크립트

❌ **삭제 안 함**:
- logs/ (학습 로그 보존)
- .git/ (Git 히스토리 보존)
- .venv/ (가상환경 보존)

---

## 예상 효과

- **docs/ 크기**: 50+ 파일 → 10개 이하
- **scripts/ 구조**: 명확한 train/test/utils
- **프로젝트 가독성**: 크게 향상
- **백업 안정성**: 모든 파일 보존

**다음 작업**: D405 카메라 URDF 생성 및 Vision RL 구현
