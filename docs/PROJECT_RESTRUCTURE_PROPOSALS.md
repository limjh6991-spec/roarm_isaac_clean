# 프로젝트 구조 개선 방안 (3가지)

## 📊 현재 상황 분석 (2025-10-28)

### 프로젝트 규모:
- **전체 파일 수**: 3,816개 (Python, Markdown, URDF, YAML, JSON)
- **전체 크기**: 4.2 GB
- **마크다운 문서**: 230개+ (docs/ + root/)
- **10M 학습 진행**: 2,130,000 steps (21.3%)

### 주요 디렉토리 구조:
```
roarm_isaac_clean/
├── 루트 디렉토리: 24개 마크다운 파일 (정리 필요 🔴)
├── docs/: 200+ 마크다운 파일 (버전별, 날짜별 분산 🔴)
├── resources/: RealSense ROS (19.62 MB), Robotiq (4.05 MB) + 문서들
├── assets/: URDF, USD, meshes
├── logs/: 학습 로그 (2.13M steps, 995K 체크포인트)
├── envs/, scripts/, configs/: 코드 파일들
└── _backup_20251019/: 백업 파일
```

### 문제점:
1. ❌ **루트 디렉토리 혼잡**: 24개 마크다운 문서가 루트에 분산
2. ❌ **문서 중복/유사**: 날짜별, 버전별로 유사한 내용 반복
3. ❌ **리소스 분산**: resources/, assets/, docs/ 간 명확한 구분 부족
4. ❌ **백업 관리 부재**: _backup_20251019/ 단일 백업만 존재
5. ❌ **학습 로그 누적**: logs/ 디렉토리 지속 증가 (4.2GB 중 상당 부분)

---

## 🎯 개선 방안 3가지

---

## 방안 1: **문서 중심 정리 (Documentation-First Cleanup)**

### 목표: 문서 체계를 깔끔하게 정리하여 탐색성 향상

### 변경 사항:

#### 1.1 루트 디렉토리 정리
```bash
# Before (24개 마크다운)
roarm_isaac_clean/
├── API_CHECKLIST.md
├── CLEANUP_PLAN.md
├── CLEANUP_REPORT.md
├── CODING_RULES.md
├── DENSE_REWARD_TRAINING_STATUS.md
├── EASY_MODE_TRAINING_GUIDE.md
├── HEADLESS_TRAINING_GUI_TESTING.md
├── IMPLEMENTATION_PLAN.md
├── MODEL_VISUALIZATION_GUIDE.md
├── PROJECT_STRUCTURE.md
├── QUICK_VISUALIZATION.md
├── README.md
├── RESTRUCTURE_STATUS.md
├── RL_IMPROVEMENT_PLAN.md
├── ROARM_M3_HARDWARE_CONTROL.md
├── ROARM_WIFI_DETAILED_GUIDE.md
├── ROBOT_FALLING_FIX.md
├── TODAY_SUMMARY.md
└── ... (6개 더)

# After (5개만 유지)
roarm_isaac_clean/
├── README.md                    # 프로젝트 개요
├── QUICK_START.md               # 빠른 시작 가이드 (신규)
├── CHANGELOG.md                 # 버전 히스토리 (통합)
├── CONTRIBUTING.md              # 개발 가이드 (신규)
└── LICENSE
```

#### 1.2 docs/ 디렉토리 재구성
```bash
docs/
├── README.md                    # 문서 인덱스
├── setup/                       # 환경 설정
│   ├── environment_setup.md
│   ├── isaac_sim_guide.md
│   └── daily_checklist.md
│
├── development/                 # 개발 가이드
│   ├── coding_rules.md         # 기존 CODING_RULES.md
│   ├── api_reference.md        # 기존 API_CHECKLIST.md
│   ├── urdf_enhancement.md     # URDF 관련 통합
│   └── testing_guide.md
│
├── training/                    # RL 학습 관련
│   ├── curriculum_training.md  # 커리큘럼 학습 가이드
│   ├── visualization.md        # MODEL_VISUALIZATION_GUIDE.md
│   ├── troubleshooting.md      # ROBOT_FALLING_FIX.md 등 통합
│   └── performance_analysis.md # 학습 분석 문서들 통합
│
├── hardware/                    # 하드웨어 관련
│   ├── roarm_m3_control.md     # ROARM_M3_HARDWARE_CONTROL.md
│   └── wifi_guide.md           # ROARM_WIFI_DETAILED_GUIDE.md
│
├── archive/                     # 날짜별/버전별 기록
│   ├── 2025/
│   │   ├── 10/
│   │   │   ├── 20251019_cleanup_report.md
│   │   │   ├── 20251020_daily_log.md
│   │   │   └── 20251028_urdf_enhancement.md
│   │   └── ...
│   └── versions/
│       ├── v3.1_grip_relaxation.md
│       ├── v3.5_changelog.md
│       ├── v3.9.0_refactoring.md
│       └── ...
│
└── phase2/                      # Phase 2: Vision RL
    ├── plan.md                  # PHASE2_VISION_BASED_RL_PLAN.md
    ├── summary.md               # PHASE2_SUMMARY.md
    └── urdf_enhancement/
        ├── complete.md
        ├── report.md
        ├── viewer_usage.md
        └── visualization_complete.md
```

#### 1.3 이동 스크립트
```bash
#!/bin/bash
# scripts/utils/reorganize_docs.sh

# 루트 → docs/development/
mv CODING_RULES.md docs/development/coding_rules.md
mv API_CHECKLIST.md docs/development/api_reference.md

# 루트 → docs/training/
mv MODEL_VISUALIZATION_GUIDE.md docs/training/visualization.md
mv QUICK_VISUALIZATION.md docs/training/quick_visualization.md
mv EASY_MODE_TRAINING_GUIDE.md docs/training/curriculum_training.md
mv ROBOT_FALLING_FIX.md docs/training/troubleshooting.md

# 루트 → docs/hardware/
mv ROARM_M3_HARDWARE_CONTROL.md docs/hardware/roarm_m3_control.md
mv ROARM_WIFI_DETAILED_GUIDE.md docs/hardware/wifi_guide.md

# 루트 → docs/archive/
mv CLEANUP_REPORT.md docs/archive/2025/10/20251019_cleanup_report.md
mv TODAY_SUMMARY.md docs/archive/2025/10/20251028_today_summary.md

# docs/ → docs/archive/versions/
mv docs/v3.*.md docs/archive/versions/

# docs/ → docs/archive/2025/10/
mv docs/DAILY_LOG_*.md docs/archive/2025/10/
mv docs/TRAINING_LOG_*.md docs/archive/2025/10/
```

### 장점:
- ✅ **탐색 용이**: 카테고리별로 명확히 분리
- ✅ **유지보수 간편**: 문서 위치가 직관적
- ✅ **히스토리 보존**: archive/에서 과거 기록 유지

### 단점:
- ⚠️ **초기 작업 큰**: 230개 문서 이동/정리 필요
- ⚠️ **링크 깨짐 위험**: 기존 링크 수정 필요

### 예상 소요 시간: **2-3시간**

---

## 방안 2: **모듈 중심 정리 (Module-Based Organization)**

### 목표: 기능 모듈별로 관련 파일들을 하나로 묶어 응집도 향상

### 변경 사항:

#### 2.1 새로운 디렉토리 구조
```bash
roarm_isaac_clean/
├── README.md
├── QUICK_START.md
│
├── modules/                     # 기능 모듈 (신규)
│   ├── training/
│   │   ├── envs/                # 기존 envs/ 이동
│   │   ├── rewards/             # 기존 rewards/ 이동
│   │   ├── controllers/         # 기존 controllers/ 이동
│   │   ├── configs/             # 학습 관련 config만
│   │   ├── scripts/
│   │   │   ├── train.py
│   │   │   ├── evaluate.py
│   │   │   └── monitor.py
│   │   ├── docs/
│   │   │   ├── README.md
│   │   │   ├── curriculum.md
│   │   │   └── troubleshooting.md
│   │   └── tests/               # 학습 관련 테스트
│   │
│   ├── robot/
│   │   ├── assets/              # 기존 assets/ 이동
│   │   │   └── roarm_m3/
│   │   │       ├── urdf/
│   │   │       ├── usd/
│   │   │       └── meshes/
│   │   ├── controllers/         # 로봇 제어 로직
│   │   ├── hardware/            # 하드웨어 인터페이스
│   │   ├── docs/
│   │   │   ├── README.md
│   │   │   ├── control_guide.md
│   │   │   ├── urdf_enhancement.md
│   │   │   └── wifi_setup.md
│   │   └── scripts/
│   │       ├── deploy.py
│   │       └── calibrate.py
│   │
│   ├── vision/                  # Phase 2: Vision RL (준비)
│   │   ├── sensors/
│   │   │   └── realsense_d455.py
│   │   ├── processing/
│   │   ├── docs/
│   │   └── configs/
│   │
│   └── simulation/
│       ├── isaac_sim/
│       ├── configs/             # 시뮬레이션 설정
│       └── docs/
│
├── data/                        # 데이터 관리 (신규)
│   ├── logs/                    # 기존 logs/ 이동
│   │   ├── training/
│   │   │   └── curriculum/
│   │   └── evaluation/
│   ├── checkpoints/             # 체크포인트 별도 관리
│   │   ├── latest/
│   │   ├── best/
│   │   └── milestones/
│   └── videos/                  # 기존 videos/ 이동
│
├── resources/                   # 외부 리소스만
│   ├── external/
│   │   ├── realsense-ros/
│   │   └── robotiq/
│   └── documentation/
│       ├── isaac_sim/
│       └── papers/
│
├── tools/                       # 유틸리티 (신규)
│   ├── visualization/
│   │   ├── urdf_viewer.py
│   │   └── training_plots.py
│   ├── analysis/
│   │   └── performance_analyzer.py
│   └── deployment/
│       └── model_exporter.py
│
└── archive/                     # 히스토리 보관
    ├── docs/
    ├── backups/
    └── deprecated/
```

#### 2.2 모듈별 README.md
각 모듈마다 자체 문서:
```markdown
# modules/training/README.md

## 개요
RoArm-M3 RL 학습 모듈

## 디렉토리 구조
- `envs/`: Isaac Sim RL 환경
- `rewards/`: 보상 함수
- `controllers/`: 정책 컨트롤러
- `configs/`: 하이퍼파라미터 설정
- `scripts/`: 학습/평가 스크립트
- `docs/`: 상세 문서

## 빠른 시작
```bash
python modules/training/scripts/train.py --config modules/training/configs/curriculum.yaml
```

## 문서
- [커리큘럼 학습 가이드](docs/curriculum.md)
- [문제 해결](docs/troubleshooting.md)
```

### 장점:
- ✅ **응집도 높음**: 관련 코드/문서/설정이 한곳에
- ✅ **확장성 좋음**: 새 모듈(vision/) 추가 용이
- ✅ **팀 협업 편리**: 모듈별로 독립 작업 가능

### 단점:
- ⚠️ **대규모 리팩토링**: 코드 import 경로 전체 수정
- ⚠️ **학습 중단 필요**: 진행 중인 10M 학습 영향 가능

### 예상 소요 시간: **4-6시간** (코드 수정 포함)

---

## 방안 3: **점진적 정리 (Incremental Cleanup)**

### 목표: 학습을 방해하지 않고 점진적으로 정리

### 변경 사항:

#### 3.1 Phase 1: 즉시 실행 가능 (1시간)
```bash
# 1. 루트 디렉토리 정리 (문서 → docs/)
mkdir -p docs/root_archive
mv *.md docs/root_archive/ 2>/dev/null || true
cp docs/root_archive/README.md ./README.md

# 2. 간단한 문서 인덱스 생성
cat > DOCUMENTATION_INDEX.md << 'EOF'
# 문서 인덱스

## 핵심 문서
- [README](README.md) - 프로젝트 개요
- [빠른 시작](docs/root_archive/QUICK_START.md)
- [프로젝트 구조](docs/root_archive/PROJECT_STRUCTURE.md)

## 카테고리별
- **환경 설정**: [docs/environment/](docs/environment/)
- **학습 관련**: [docs/training/](docs/root_archive/EASY_MODE_TRAINING_GUIDE.md)
- **하드웨어**: [docs/hardware/](docs/root_archive/ROARM_M3_HARDWARE_CONTROL.md)
- **Phase 2**: [docs/phase2/](docs/PHASE2_VISION_BASED_RL_PLAN.md)

## 최근 문서
- [URDF Enhancement Complete](docs/URDF_ENHANCEMENT_COMPLETE.md)
- [Phase 2 Summary](docs/PHASE2_SUMMARY.md)

## 히스토리
- **날짜별 로그**: [docs/archive/](docs/root_archive/TODAY_SUMMARY.md)
- **버전별**: [docs/v*.md](docs/)
EOF

# 3. 백업 자동화
mkdir -p archive/backups
tar -czf archive/backups/backup_$(date +%Y%m%d_%H%M%S).tar.gz \
    --exclude='logs' --exclude='resources' --exclude='.venv' \
    .
```

#### 3.2 Phase 2: 학습 완료 후 실행 (2시간)
```bash
# 1. logs/ 아카이브
mkdir -p data/logs/archive
mv logs/rl_training_curriculum_old_* data/logs/archive/

# 2. 체크포인트 정리 (5K steps 간격만 유지)
python scripts/utils/cleanup_checkpoints.py --keep-interval 5000

# 3. 중복 문서 병합
python scripts/utils/merge_duplicate_docs.py \
    --input docs/ \
    --output docs/consolidated/
```

#### 3.3 Phase 3: 최종 정리 (필요시, 3시간)
- 방안 1 또는 방안 2 적용

### 점진적 정리 스크립트 예시:
```python
# scripts/utils/incremental_cleanup.py

import os
import shutil
from pathlib import Path

class IncrementalCleanup:
    def __init__(self, project_root):
        self.root = Path(project_root)
        
    def phase1_urgent_cleanup(self):
        """긴급: 루트 디렉토리 정리 (학습 영향 없음)"""
        print("Phase 1: Urgent cleanup...")
        
        # 1. 루트 마크다운 → docs/root_archive/
        docs_archive = self.root / "docs" / "root_archive"
        docs_archive.mkdir(parents=True, exist_ok=True)
        
        for md_file in self.root.glob("*.md"):
            if md_file.name == "README.md":
                continue
            shutil.move(str(md_file), str(docs_archive / md_file.name))
            print(f"  Moved: {md_file.name}")
        
        # 2. 백업 생성
        self._create_backup()
        
        print("✅ Phase 1 complete!")
    
    def phase2_post_training(self):
        """학습 완료 후: 로그 아카이브"""
        print("Phase 2: Post-training cleanup...")
        
        # 1. 오래된 체크포인트 정리
        self._cleanup_checkpoints()
        
        # 2. 로그 압축
        self._compress_old_logs()
        
        print("✅ Phase 2 complete!")
    
    def _create_backup(self):
        """자동 백업"""
        import tarfile
        from datetime import datetime
        
        backup_dir = self.root / "archive" / "backups"
        backup_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_file = backup_dir / f"backup_{timestamp}.tar.gz"
        
        with tarfile.open(backup_file, "w:gz") as tar:
            for item in self.root.iterdir():
                if item.name not in ['.venv', 'logs', 'resources', '.git']:
                    tar.add(item, arcname=item.name)
        
        print(f"  Backup created: {backup_file}")
    
    def _cleanup_checkpoints(self, keep_interval=5000):
        """체크포인트 정리 (5K steps 간격만 유지)"""
        checkpoint_dir = self.root / "logs" / "rl_training_curriculum" / "checkpoints"
        
        if not checkpoint_dir.exists():
            return
        
        checkpoint_files = list(checkpoint_dir.glob("*_steps.zip"))
        
        # Extract step numbers
        checkpoints = []
        for f in checkpoint_files:
            try:
                steps = int(f.stem.split("_")[-2])
                checkpoints.append((steps, f))
            except:
                continue
        
        # Keep only 5K interval checkpoints + latest
        checkpoints.sort()
        latest_steps = checkpoints[-1][0] if checkpoints else 0
        
        kept = 0
        removed = 0
        
        for steps, filepath in checkpoints:
            # Keep if: multiple of keep_interval OR latest
            if steps % keep_interval == 0 or steps == latest_steps:
                kept += 1
            else:
                filepath.unlink()
                # Also remove corresponding vecnormalize file
                vecnorm_file = filepath.parent / filepath.name.replace("_steps.zip", "_vecnormalize.pkl")
                if vecnorm_file.exists():
                    vecnorm_file.unlink()
                removed += 1
        
        print(f"  Checkpoints: kept {kept}, removed {removed}")

if __name__ == "__main__":
    cleanup = IncrementalCleanup("/home/roarm_m3/roarm_isaac_clean")
    
    # Phase 1 실행 (안전)
    cleanup.phase1_urgent_cleanup()
```

### 장점:
- ✅ **학습 안전**: 진행 중인 10M 학습에 영향 없음
- ✅ **위험 최소**: 단계별로 검증하며 진행
- ✅ **즉시 효과**: Phase 1만으로도 큰 개선

### 단점:
- ⚠️ **완전하지 않음**: 최종 구조까지는 시간 소요
- ⚠️ **관리 필요**: 3단계 모두 실행해야 완전히 정리

### 예상 소요 시간: **1시간 (Phase 1)** → 2-3일 (전체)

---

## 📊 방안 비교표

| 항목 | 방안 1: 문서 중심 | 방안 2: 모듈 중심 | 방안 3: 점진적 |
|------|-------------------|-------------------|---------------|
| **소요 시간** | 2-3시간 | 4-6시간 | 1시간 → 2-3일 |
| **학습 영향** | ⚠️ 중간 (링크 깨짐 가능) | ❌ 높음 (import 수정) | ✅ 없음 |
| **탐색성** | ✅ 매우 좋음 | ✅ 좋음 | ⚠️ 보통 |
| **확장성** | ⚠️ 보통 | ✅ 매우 좋음 | ⚠️ 보통 |
| **유지보수** | ✅ 쉬움 | ✅ 쉬움 | ⚠️ 보통 |
| **협업** | ✅ 좋음 | ✅ 매우 좋음 | ⚠️ 보통 |
| **위험도** | ⚠️ 중간 | ❌ 높음 | ✅ 낮음 |
| **완성도** | ✅ 높음 | ✅ 매우 높음 | ⚠️ 낮음 (Phase 1만) |

---

## 💡 추천 전략

### 🏆 **추천: 방안 3 → 방안 1 조합**

#### Step 1: 지금 즉시 (방안 3 - Phase 1)
```bash
# 10M 학습 진행 중이므로 안전하게 시작
cd /home/roarm_m3/roarm_isaac_clean

# Phase 1: 루트 디렉토리만 정리 (1시간)
python scripts/utils/incremental_cleanup.py --phase 1
```

**효과:**
- 루트 디렉토리 깔끔해짐 (24개 → 5개 파일)
- 학습에 영향 없음
- 즉시 탐색성 향상

#### Step 2: 10M 학습 완료 후 (방안 1)
```bash
# 2,130,000 → 10,000,000 완료 후 (약 2-3일 후)

# Phase 2: 문서 체계화 (2-3시간)
bash scripts/utils/reorganize_docs.sh
```

**효과:**
- docs/ 디렉토리 카테고리별 정리
- 히스토리 보존 (archive/)
- 문서 탐색 용이

#### Step 3: Phase 2 (Vision RL) 준비 시 (선택)
```bash
# Vision RL 시작 전에 모듈화 검토

# 방안 2의 modules/vision/ 구조 도입
mkdir -p modules/vision
# ...
```

---

## 📋 즉시 실행 가능한 스크립트

### 스크립트 1: 루트 정리 (Phase 1)
```bash
#!/bin/bash
# scripts/utils/quick_cleanup_phase1.sh

set -e

PROJECT_ROOT="/home/roarm_m3/roarm_isaac_clean"
cd "$PROJECT_ROOT"

echo "🚀 Phase 1: Quick Root Cleanup"
echo "==============================="

# 1. 백업 생성
echo "📦 Creating backup..."
mkdir -p archive/backups
tar -czf archive/backups/before_cleanup_$(date +%Y%m%d_%H%M%S).tar.gz \
    --exclude='logs' --exclude='resources' --exclude='.venv' --exclude='.git' \
    *.md docs/ scripts/ configs/ 2>/dev/null || true

# 2. docs/root_archive/ 생성
echo "📁 Creating docs/root_archive/..."
mkdir -p docs/root_archive

# 3. 루트 마크다운 이동 (README.md 제외)
echo "📝 Moving root markdown files..."
for file in *.md; do
    if [ "$file" != "README.md" ] && [ -f "$file" ]; then
        mv "$file" docs/root_archive/
        echo "  ✓ Moved: $file"
    fi
done

# 4. DOCUMENTATION_INDEX.md 생성
echo "📄 Creating DOCUMENTATION_INDEX.md..."
cat > DOCUMENTATION_INDEX.md << 'EOF'
# 📚 Documentation Index

> **Quick Links**: [README](README.md) | [Quick Start](docs/root_archive/EASY_MODE_TRAINING_GUIDE.md) | [Project Structure](docs/root_archive/PROJECT_STRUCTURE.md)

## 🎯 주요 문서

### 환경 설정
- [Environment Setup](docs/environment/ENVIRONMENT_SETUP.md)
- [Isaac Sim Python Guide](docs/ISAAC_SIM_PYTHON_GUIDE.md)
- [Daily Checklist](docs/environment/DAILY_CHECKLIST.md)

### RL 학습
- [Easy Mode Training Guide](docs/root_archive/EASY_MODE_TRAINING_GUIDE.md)
- [Model Visualization](docs/root_archive/MODEL_VISUALIZATION_GUIDE.md)
- [RL Improvement Plan](docs/root_archive/RL_IMPROVEMENT_PLAN.md)

### 하드웨어
- [RoArm-M3 Control](docs/root_archive/ROARM_M3_HARDWARE_CONTROL.md)
- [WiFi Setup Guide](docs/root_archive/ROARM_WIFI_DETAILED_GUIDE.md)

### Phase 2: Vision RL
- [Phase 2 Plan](docs/PHASE2_VISION_BASED_RL_PLAN.md)
- [Phase 2 Summary](docs/PHASE2_SUMMARY.md)
- [URDF Enhancement](docs/URDF_ENHANCEMENT_COMPLETE.md)

### 개발
- [Coding Rules](docs/root_archive/CODING_RULES.md)
- [API Checklist](docs/root_archive/API_CHECKLIST.md)

## 📁 디렉토리 구조

```
roarm_isaac_clean/
├── docs/
│   ├── root_archive/          # 기존 루트 문서들
│   ├── environment/           # 환경 설정
│   ├── phase2/                # Vision RL
│   ├── v*.md                  # 버전별 문서
│   └── *.md                   # 기타 문서
├── assets/                    # URDF, USD, meshes
├── envs/                      # RL 환경
├── scripts/                   # 실행 스크립트
├── configs/                   # 설정 파일
└── logs/                      # 학습 로그
```

## 🗂️ 아카이브

- **최근 변경사항**: [docs/root_archive/TODAY_SUMMARY.md](docs/root_archive/TODAY_SUMMARY.md)
- **정리 리포트**: [docs/root_archive/CLEANUP_REPORT.md](docs/root_archive/CLEANUP_REPORT.md)
- **백업**: `archive/backups/`

---

**작성일**: 2025-10-28  
**Phase 1 cleanup 완료**
EOF

# 5. README.md 업데이트 (인덱스 링크 추가)
if ! grep -q "DOCUMENTATION_INDEX.md" README.md; then
    echo "" >> README.md
    echo "## 📚 Documentation" >> README.md
    echo "" >> README.md
    echo "See [Documentation Index](DOCUMENTATION_INDEX.md) for all available documentation." >> README.md
fi

echo ""
echo "✅ Phase 1 Complete!"
echo "===================="
echo "📊 Results:"
echo "  - Root markdown files: moved to docs/root_archive/"
echo "  - Documentation index: DOCUMENTATION_INDEX.md created"
echo "  - Backup: archive/backups/"
echo ""
echo "🎯 Next steps:"
echo "  1. Review: cat DOCUMENTATION_INDEX.md"
echo "  2. Wait for 10M training to complete (currently 21.3%)"
echo "  3. Run Phase 2: bash scripts/utils/reorganize_docs.sh"
echo ""
```

### 스크립트 2: 체크포인트 정리
```bash
#!/bin/bash
# scripts/utils/cleanup_checkpoints.sh

CHECKPOINT_DIR="logs/rl_training_curriculum/checkpoints"
KEEP_INTERVAL=5000  # Keep every 5K steps

echo "🧹 Checkpoint Cleanup"
echo "===================="
echo "Keep interval: every ${KEEP_INTERVAL} steps"
echo ""

cd "$CHECKPOINT_DIR" || exit 1

# Find all checkpoint files
total=0
kept=0
removed=0

for file in *_steps.zip; do
    [ -f "$file" ] || continue
    
    # Extract step number
    steps=$(echo "$file" | grep -oP '\d+(?=_steps\.zip)')
    
    total=$((total + 1))
    
    # Keep if multiple of interval
    if [ $((steps % KEEP_INTERVAL)) -eq 0 ]; then
        kept=$((kept + 1))
        echo "✓ Keep: $file ($steps steps)"
    else
        removed=$((removed + 1))
        rm -f "$file"
        rm -f "${file/_steps.zip/_vecnormalize.pkl}" 2>/dev/null
        echo "✗ Remove: $file"
    fi
done

echo ""
echo "📊 Summary:"
echo "  Total: $total"
echo "  Kept: $kept"
echo "  Removed: $removed"
echo "  Saved space: ~$((removed * 182))KB"
```

---

## 🎯 결론 및 추천

### 즉시 실행 (지금):
```bash
# Phase 1: 루트 정리 (10M 학습 영향 없음)
bash scripts/utils/quick_cleanup_phase1.sh
```

### 10M 학습 완료 후:
```bash
# Phase 2: 문서 체계화
bash scripts/utils/reorganize_docs.sh

# Phase 3: 체크포인트 정리
bash scripts/utils/cleanup_checkpoints.sh
```

### Phase 2 (Vision RL) 준비 시:
- 방안 2의 모듈화 구조 검토
- modules/vision/ 디렉토리 추가

---

**작성일**: 2025-10-28  
**10M 학습 진행**: 2,130,000 steps (21.3%)  
**추천 방안**: 방안 3 (Phase 1) → 방안 1 조합
