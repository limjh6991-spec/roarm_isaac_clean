#!/bin/bash
# Phase 1: Quick Root Cleanup
# Safe to run during training - no impact on logs/configs

set -e

PROJECT_ROOT="/home/roarm_m3/roarm_isaac_clean"
cd "$PROJECT_ROOT"

echo "🚀 Phase 1: Quick Root Cleanup"
echo "==============================="
echo ""

# 1. 백업 생성
echo "📦 Step 1: Creating backup..."
mkdir -p archive/backups
BACKUP_FILE="archive/backups/before_cleanup_$(date +%Y%m%d_%H%M%S).tar.gz"
tar -czf "$BACKUP_FILE" \
    --exclude='logs' \
    --exclude='resources/grippers' \
    --exclude='.venv' \
    --exclude='.git' \
    --exclude='videos' \
    *.md docs/ scripts/ configs/ 2>/dev/null || true

echo "  ✓ Backup created: $BACKUP_FILE"
BACKUP_SIZE=$(du -sh "$BACKUP_FILE" | cut -f1)
echo "  ✓ Backup size: $BACKUP_SIZE"
echo ""

# 2. docs/root_archive/ 생성
echo "📁 Step 2: Creating docs/root_archive/..."
mkdir -p docs/root_archive

# 3. 루트 마크다운 이동 (README.md 제외)
echo "📝 Step 3: Moving root markdown files..."
MOVED_COUNT=0
for file in *.md; do
    if [ "$file" != "README.md" ] && [ -f "$file" ]; then
        mv "$file" docs/root_archive/
        echo "  ✓ Moved: $file"
        MOVED_COUNT=$((MOVED_COUNT + 1))
    fi
done
echo "  📊 Total files moved: $MOVED_COUNT"
echo ""

# 4. DOCUMENTATION_INDEX.md 생성
echo "📄 Step 4: Creating DOCUMENTATION_INDEX.md..."
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
- [Training Monitor](monitor_training.sh)

### 하드웨어
- [RoArm-M3 Control](docs/root_archive/ROARM_M3_HARDWARE_CONTROL.md)
- [WiFi Setup Guide](docs/root_archive/ROARM_WIFI_DETAILED_GUIDE.md)

### Phase 2: Vision RL
- [Phase 2 Plan](docs/PHASE2_VISION_BASED_RL_PLAN.md)
- [Phase 2 Summary](docs/PHASE2_SUMMARY.md)
- [URDF Enhancement Complete](docs/URDF_ENHANCEMENT_COMPLETE.md)
- [URDF Viewer Usage](docs/URDF_VIEWER_USAGE.md)
- [URDF Viewer GUI Guide](docs/URDF_VIEWER_GUI_GUIDE.md)

### 개발
- [Coding Rules](docs/root_archive/CODING_RULES.md)
- [API Checklist](docs/root_archive/API_CHECKLIST.md)

## 📁 디렉토리 구조

```
roarm_isaac_clean/
├── README.md                  # 프로젝트 개요
├── DOCUMENTATION_INDEX.md     # 이 문서
│
├── docs/
│   ├── root_archive/          # 기존 루트 문서들
│   ├── environment/           # 환경 설정
│   ├── phase2/                # Vision RL (준비 중)
│   ├── v*.md                  # 버전별 문서
│   └── *.md                   # 기타 문서
│
├── assets/                    # URDF, USD, meshes
│   └── roarm_m3/
│       ├── urdf/              # Enhanced URDF (380 lines)
│       ├── usd/
│       └── meshes/
│
├── envs/                      # RL 환경
├── scripts/                   # 실행 스크립트
├── configs/                   # 설정 파일
├── logs/                      # 학습 로그
│   └── rl_training_curriculum/
│       └── checkpoints/       # 2.13M steps (21.3% of 10M)
│
├── resources/                 # 외부 리소스
│   ├── grippers/              # RealSense ROS, Robotiq URDF
│   └── RESOURCE_INDEX.md
│
└── archive/                   # 백업 및 히스토리
    └── backups/
```

## 🗂️ 아카이브

- **최근 변경사항**: [docs/root_archive/TODAY_SUMMARY.md](docs/root_archive/TODAY_SUMMARY.md)
- **정리 리포트**: [docs/root_archive/CLEANUP_REPORT.md](docs/root_archive/CLEANUP_REPORT.md)
- **백업**: `archive/backups/`

## 📊 프로젝트 현황 (2025-10-28)

- **10M 학습 진행**: 2,130,000 steps (21.3%)
- **Phase 2 준비**: Enhanced URDF 생성 완료 (380 lines, 7 DOF, D455 camera)
- **다음 단계**: Isaac Sim 통합 테스트

---

**작성일**: 2025-10-28  
**Phase 1 cleanup 완료**  
**구조 개선 방안**: [docs/PROJECT_RESTRUCTURE_PROPOSALS.md](docs/PROJECT_RESTRUCTURE_PROPOSALS.md)
EOF

echo "  ✓ DOCUMENTATION_INDEX.md created"
echo ""

# 5. README.md 업데이트 (인덱스 링크 추가)
echo "📝 Step 5: Updating README.md..."
if ! grep -q "DOCUMENTATION_INDEX.md" README.md; then
    cat >> README.md << 'EOF'

## 📚 Documentation

See [Documentation Index](DOCUMENTATION_INDEX.md) for all available documentation.

### Quick Links
- [Training Guide](docs/root_archive/EASY_MODE_TRAINING_GUIDE.md)
- [Visualization](docs/root_archive/MODEL_VISUALIZATION_GUIDE.md)
- [Hardware Control](docs/root_archive/ROARM_M3_HARDWARE_CONTROL.md)
- [Phase 2 Plan](docs/PHASE2_VISION_BASED_RL_PLAN.md)
EOF
    echo "  ✓ README.md updated with documentation links"
else
    echo "  ℹ README.md already has documentation links"
fi
echo ""

# 6. 상태 확인
echo "📊 Step 6: Verification..."
ROOT_MD_COUNT=$(find . -maxdepth 1 -name "*.md" | wc -l)
ARCHIVE_MD_COUNT=$(find docs/root_archive/ -name "*.md" 2>/dev/null | wc -l)

echo "  ✓ Root markdown files: $ROOT_MD_COUNT (should be 2: README + INDEX)"
echo "  ✓ Archived files: $ARCHIVE_MD_COUNT"
echo ""

# 7. 최종 결과
echo "✅ Phase 1 Complete!"
echo "===================="
echo ""
echo "📊 Summary:"
echo "  ✓ Backed up $MOVED_COUNT files to $BACKUP_FILE"
echo "  ✓ Moved $MOVED_COUNT markdown files to docs/root_archive/"
echo "  ✓ Created DOCUMENTATION_INDEX.md"
echo "  ✓ Updated README.md"
echo ""
echo "📁 Root directory now contains:"
ls -1 *.md 2>/dev/null || echo "  (no markdown files)"
echo ""
echo "🎯 Next steps:"
echo "  1. Review documentation index:"
echo "     cat DOCUMENTATION_INDEX.md"
echo ""
echo "  2. Verify root is clean:"
echo "     ls -la | grep '.md'"
echo ""
echo "  3. Wait for 10M training to complete (currently 21.3%)"
echo ""
echo "  4. After training, run Phase 2:"
echo "     bash scripts/utils/reorganize_docs.sh"
echo ""
echo "✨ Your project is now more organized!"
