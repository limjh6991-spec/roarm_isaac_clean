#!/usr/bin/env python3
"""
URDF 표준화 검증 스크립트
생성된 URDF가 모든 표준을 만족하는지 자동 검증

사용법:
    python scripts/verify_urdf_standard.py assets/roarm_m3/urdf/roarm_m3_standard.urdf
"""

import xml.etree.ElementTree as ET
import sys
from pathlib import Path


def verify_urdf_standard(urdf_path):
    """URDF 표준 검증"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    checks = {
        "total": 0,
        "passed": 0,
        "failed": 0,
        "warnings": 0,
    }

    print("╔════════════════════════════════════════════════════════════════╗")
    print("║              URDF 표준화 검증                                  ║")
    print("╚════════════════════════════════════════════════════════════════╝\n")

    # 1. 링크 수 검증
    links = root.findall(".//link")
    link_count = len([l for l in links if l.get("name") != "world"])
    print(f"[1] 링크 수 검증")
    if link_count == 7:
        print(f"    ✅ {link_count}개 링크 (world 제외) - OK")
        checks["passed"] += 1
    else:
        print(f"    ❌ {link_count}개 링크 (기대: 7개)")
        checks["failed"] += 1
    checks["total"] += 1
    print()

    # 2. Visual Origin 검증
    print("[2] Visual Origin 검증")
    visual_origins = root.findall(".//visual/origin")
    non_zero_origins = []
    for vo in visual_origins:
        xyz = vo.get("xyz", "0 0 0")
        rpy = vo.get("rpy", "0 0 0")
        if xyz != "0 0 0" or rpy != "0 0 0":
            link = vo.getparent().getparent()
            non_zero_origins.append((link.get("name"), xyz, rpy))

    if not non_zero_origins:
        print("    ✅ 모든 visual origin이 [0,0,0] [0,0,0] - OK")
        checks["passed"] += 1
    else:
        print(f"    ❌ {len(non_zero_origins)}개 링크에서 origin≠0:")
        for link_name, xyz, rpy in non_zero_origins:
            print(f"       {link_name}: xyz={xyz}, rpy={rpy}")
        checks["failed"] += 1
    checks["total"] += 1
    print()

    # 3. Scale 검증
    print("[3] Scale 검증")
    meshes = root.findall(".//mesh")
    non_unit_scales = []
    for mesh in meshes:
        scale = mesh.get("scale", "1 1 1")
        if scale != "1 1 1":
            # 부모 링크 찾기
            parent = mesh.getparent()
            while parent is not None and parent.tag != "link":
                parent = parent.getparent()
            link_name = parent.get("name") if parent is not None else "unknown"
            non_unit_scales.append((link_name, scale))

    if not non_unit_scales:
        print("    ✅ 모든 mesh scale이 '1 1 1' - OK")
        checks["passed"] += 1
    else:
        print(f"    ❌ {len(non_unit_scales)}개 메시에서 scale≠1:")
        for link_name, scale in non_unit_scales:
            print(f"       {link_name}: scale={scale}")
        checks["failed"] += 1
    checks["total"] += 1
    print()

    # 4. STL 경로 검증
    print("[4] STL 파일 존재 검증")
    urdf_dir = Path(urdf_path).parent
    missing_stls = []
    for mesh in meshes:
        filename = mesh.get("filename", "")
        if filename.startswith("../"):
            # 상대 경로 해석 (../)
            stl_path = (urdf_dir / filename).resolve()
        elif not filename.startswith("/"):
            # 상대 경로 해석 (URDF 기준)
            stl_path = (urdf_dir.parent / filename).resolve()
        else:
            # 절대 경로
            stl_path = Path(filename)

        if not stl_path.exists():
            missing_stls.append(str(stl_path))

    if not missing_stls:
        print(f"    ✅ {len(meshes)}개 STL 파일 모두 존재 - OK")
        checks["passed"] += 1
    else:
        print(f"    ⚠️  {len(missing_stls)}개 STL 파일 누락:")
        for stl in missing_stls[:5]:  # 최대 5개만 출력
            print(f"       {stl}")
        checks["warnings"] += 1
    checks["total"] += 1
    print()

    # 5. Inertial 검증
    print("[5] Inertial 검증")
    inertials = root.findall(".//inertial")
    missing_inertials = []
    for link in links:
        if link.get("name") == "world":
            continue
        if link.find("inertial") is None:
            missing_inertials.append(link.get("name"))

    if not missing_inertials:
        print(f"    ✅ {len(inertials)}개 링크에 inertial 정의됨 - OK")
        checks["passed"] += 1
    else:
        print(f"    ⚠️  {len(missing_inertials)}개 링크에 inertial 누락:")
        for link_name in missing_inertials:
            print(f"       {link_name}")
        checks["warnings"] += 1
    checks["total"] += 1
    print()

    # 6. Joint 검증
    print("[6] Joint 검증")
    joints = root.findall(".//joint")
    joint_count = len([j for j in joints if j.get("type") != "fixed" or "world" not in j.get("name", "")])
    if joint_count >= 6:
        print(f"    ✅ {joint_count}개 조인트 정의됨 - OK")
        checks["passed"] += 1
    else:
        print(f"    ⚠️  {joint_count}개 조인트 (기대: 6개 이상)")
        checks["warnings"] += 1
    checks["total"] += 1
    print()

    # 최종 결과
    print("╔════════════════════════════════════════════════════════════════╗")
    print("║              검증 결과 요약                                    ║")
    print("╚════════════════════════════════════════════════════════════════╝\n")

    pass_rate = checks["passed"] / checks["total"] * 100
    print(f"총 검증: {checks['total']}개")
    print(f"통과: {checks['passed']}개 ✅")
    print(f"실패: {checks['failed']}개 ❌")
    print(f"경고: {checks['warnings']}개 ⚠️")
    print(f"통과율: {pass_rate:.1f}%\n")

    if checks["failed"] == 0 and checks["warnings"] == 0:
        print("🎉 완벽! 모든 검증 통과")
        print("→ Isaac Sim 임포트 준비 완료\n")
        return 0
    elif checks["failed"] == 0:
        print("⚠️  경고 사항이 있지만 사용 가능")
        print("→ Isaac Sim 임포트 시도 가능\n")
        return 0
    else:
        print("❌ 실패 항목 수정 필요")
        print("→ URDF 패처 재실행 또는 수동 수정\n")
        return 1


def main():
    if len(sys.argv) < 2:
        print("사용법: python verify_urdf_standard.py <urdf_file>")
        sys.exit(1)

    urdf_path = Path(sys.argv[1])
    if not urdf_path.exists():
        print(f"❌ 파일 없음: {urdf_path}")
        sys.exit(1)

    exit_code = verify_urdf_standard(urdf_path)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
