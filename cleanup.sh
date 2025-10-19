#!/bin/bash
# 프로젝트 정리 스크립트
# 작성일: 2025-10-19

set -e

BACKUP_DIR="_backup_20251019"
echo "🗂️  프로젝트 정리 시작..."
echo ""

# 1. scripts/ 정리 (51개 삭제)
echo "📁 scripts/ 정리 중..."
cd ~/roarm_isaac_clean/scripts

# 구버전 데모
rm -f demo_roarm_env.py demo_simple_isaac_assets.py
rm -f run_demo.sh run_train.sh

# 테스트 파일들
rm -f test_flush.py test_mesh_urdf.py test_minimal.py test_scale_one.py
rm -f test_urdf_import_simple.py test_urdf_loading.py test_multiprim_isaac.py

# STEP/STL/URDF 변환 관련
rm -f convert_step_to_stl.py export_stl_by_link_headless.py
rm -f export_stl_with_transform.py center_stl_files.py
rm -f fix_stl_center.sh split_complete_stl.py
rm -f apply_transform_to_stl.py analyze_step_structure.py
rm -f analyze_stl_center.py extract_fcstd_parts.py
rm -f extract_freecad_parts_list.py freecad_auto_export_stl.py
rm -f freecad_list_objects.FCMacro generate_multiprim_urdf.py
rm -f patch_urdf_with_merged_stls.py inspect_step_structure.py

# URDF validation 관련
rm -f validate_urdf.py verify_split_urdf.py verify_urdf_isaac.py
rm -f verify_urdf_standard.py verify_usd.py urdf_autopatch_by_link.py
rm -f urdf_autopatch_standard.py fix_urdf_paths_absolute.py
rm -f run_urdf_standardization.sh

# import 테스트
rm -f import_urdf_to_isaac.py import_and_merge_urdf.py

# launch_gui
rm -f launch_gui.py

# ontology 스크립트
rm -rf ontology/

echo "  ✅ scripts/: 51개 파일 삭제 완료"
echo ""

# 2. docs/ 정리 (25개 삭제)
echo "📁 docs/ 정리 중..."
cd ~/roarm_isaac_clean/docs

rm -f CURRENT_ISSUE_ANALYSIS.md ENVIRONMENT_STATUS.md
rm -f ISSUES_AND_SOLUTIONS.md LESSONS_LEARNED.md
rm -f PHASE_DECISION.md PHASE2_SUMMARY.md
rm -f PROJECT_RESTART_SUMMARY.md WARMUP_SESSION_20251018.md
rm -f FREECAD_AUTO_EXPORT_GUIDE.md FREECAD_EXPORT_CHECKLIST.md
rm -f MESH_ORIGIN_FIX.md MULTI_PRIMITIVE_URDF_STRATEGY.md
rm -f JARVIS_URDF_SOLUTION.md STEP_PARTS_MAPPING.md
rm -f STEP_TO_STL_GUIDE.md URDF_STANDARD_GUIDE.md
rm -f URDF_TO_USD_GUI_GUIDE.md RoArm_M3_Link_STL_Status_v1.md
rm -f DEVOPS_GUIDE.md PXR_ENVIRONMENT_GUIDE.md
rm -f ISAAC_ASSETS_IMPLEMENTATION.md
rm -rf ontology/

echo "  ✅ docs/: 25개 파일 삭제 완료"
echo ""

# 3. logs/ 정리 (29개 삭제)
echo "📁 logs/ 정리 중..."
cd ~/roarm_isaac_clean/logs

rm -f urdf_standardization_20251018.md
rm -f freecad_transform_20251018_164739.log
rm -f batch_export_20251018_155919.log
rm -f batch_export_20251018_155621.log
rm -rf preflight/

echo "  ✅ logs/: 29개 파일 삭제 완료"
echo ""

# 4. ontology/ 폴더 전체 삭제
echo "📁 ontology/ 삭제 중..."
cd ~/roarm_isaac_clean
rm -rf ontology/

echo "  ✅ ontology/: 폴더 전체 삭제 완료"
echo ""

# 5. devops/ 정리 (7개 삭제)
echo "📁 devops/ 정리 중..."
cd ~/roarm_isaac_clean/devops

rm -f diagnose_python_env.sh preflight_all.sh run_isaac_supervised.sh
rm -rf preflight/

echo "  ✅ devops/: 7개 파일 삭제 완료"
echo ""

# 6. tests/ 정리 (2개 삭제)
echo "📁 tests/ 정리 중..."
cd ~/roarm_isaac_clean/tests

rm -f test_kit_boot.py
rm -rf __pycache__/

echo "  ✅ tests/: 2개 파일 삭제 완료"
echo ""

# 7. 기타 폴더 삭제
echo "📁 기타 폴더 삭제 중..."
cd ~/roarm_isaac_clean
rm -rf "스크린샷/"

echo "  ✅ 스크린샷/: 폴더 삭제 완료"
echo ""

# 8. configs/ 폴더 삭제 (비어있음)
echo "📁 configs/ 폴더 삭제 중..."
rm -rf configs/

echo "  ✅ configs/: 폴더 삭제 완료"
echo ""

# 9. __pycache__ 정리
echo "📁 __pycache__ 정리 중..."
cd ~/roarm_isaac_clean
find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find . -type f -name "*.pyc" -delete 2>/dev/null || true

echo "  ✅ __pycache__ 정리 완료"
echo ""

# 10. .pytest_cache 정리
rm -rf .pytest_cache/

echo "  ✅ .pytest_cache 정리 완료"
echo ""

echo "✨ 정리 완료!"
echo ""
echo "📊 정리 요약:"
echo "  - scripts/: 58 → 7 파일 (-88%)"
echo "  - docs/: 30 → 5 파일 (-83%)"
echo "  - logs/: 30 → 1 파일 (-97%)"
echo "  - ontology/: 삭제됨"
echo "  - devops/: 9 → 2 파일 (-78%)"
echo "  - tests/: 3 → 1 파일 (-67%)"
echo "  - configs/: 삭제됨"
echo "  - 스크린샷/: 삭제됨"
echo ""
echo "🎯 총 124개 파일/폴더 삭제 완료"
