#!/usr/bin/env python3
"""
URDF 검증 스크립트

URDF 파일의 구조와 내용을 검증합니다:
- 모든 링크에 collision geometry 존재 확인
- 모든 링크에 inertial 정의 확인
- Joint 한계값 확인
- Mesh 파일 존재 확인

사용법:
    python scripts/urdf/validate_urdf.py <urdf_file>
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple


def validate_urdf(urdf_path: Path) -> Tuple[bool, List[str]]:
    """URDF 파일 검증
    
    Args:
        urdf_path: URDF 파일 경로
        
    Returns:
        (검증 성공 여부, 경고/에러 메시지 리스트)
    """
    issues = []
    
    if not urdf_path.exists():
        return False, [f"❌ URDF 파일을 찾을 수 없습니다: {urdf_path}"]
    
    # XML 파싱
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
    except ET.ParseError as e:
        return False, [f"❌ XML 파싱 오류: {e}"]
    
    print("=" * 70)
    print("URDF 검증")
    print("=" * 70)
    print(f"파일: {urdf_path}")
    print()
    
    # 링크 수집
    links = root.findall('link')
    joints = root.findall('joint')
    
    print(f"📊 통계:")
    print(f"   - 링크 수: {len(links)}개")
    print(f"   - 조인트 수: {len(joints)}개")
    print()
    
    # 1. 링크 검증
    print("🔍 링크 검증...")
    for link in links:
        link_name = link.get('name')
        
        # Inertial 확인
        inertial = link.find('inertial')
        if inertial is None:
            issues.append(f"⚠️  링크 '{link_name}': inertial 정의 없음")
        else:
            mass = inertial.find('mass')
            if mass is None or float(mass.get('value', 0)) <= 0:
                issues.append(f"⚠️  링크 '{link_name}': 질량이 0 이하")
        
        # Collision 확인
        collision = link.find('collision')
        if collision is None:
            issues.append(f"⚠️  링크 '{link_name}': collision geometry 없음")
        else:
            geometry = collision.find('geometry')
            if geometry is None:
                issues.append(f"❌ 링크 '{link_name}': collision geometry가 비어있음")
            else:
                # Mesh 파일 확인
                mesh = geometry.find('mesh')
                if mesh is not None:
                    mesh_file = mesh.get('filename')
                    mesh_path = urdf_path.parent / mesh_file
                    if not mesh_path.exists():
                        issues.append(f"❌ 링크 '{link_name}': Mesh 파일 없음 - {mesh_file}")
                    else:
                        print(f"   ✅ {link_name}: Mesh 파일 존재 ({mesh_file})")
        
        # Visual 확인
        visual = link.find('visual')
        if visual is None:
            issues.append(f"⚠️  링크 '{link_name}': visual geometry 없음")
    
    print()
    
    # 2. 조인트 검증
    print("🔍 조인트 검증...")
    for joint in joints:
        joint_name = joint.get('name')
        joint_type = joint.get('type')
        
        # 축 확인
        axis = joint.find('axis')
        if axis is None and joint_type not in ['fixed']:
            issues.append(f"⚠️  조인트 '{joint_name}': axis 정의 없음")
        
        # Limit 확인 (revolute, prismatic)
        if joint_type in ['revolute', 'prismatic']:
            limit = joint.find('limit')
            if limit is None:
                issues.append(f"❌ 조인트 '{joint_name}': limit 정의 없음 ({joint_type})")
            else:
                lower = float(limit.get('lower', 0))
                upper = float(limit.get('upper', 0))
                if lower >= upper:
                    issues.append(f"❌ 조인트 '{joint_name}': lower >= upper ({lower} >= {upper})")
                effort = float(limit.get('effort', 0))
                velocity = float(limit.get('velocity', 0))
                if effort <= 0 or velocity <= 0:
                    issues.append(f"⚠️  조인트 '{joint_name}': effort 또는 velocity가 0")
                else:
                    print(f"   ✅ {joint_name}: {joint_type}, range=[{lower:.2f}, {upper:.2f}]")
        
        # Dynamics 확인
        dynamics = joint.find('dynamics')
        if dynamics is None:
            issues.append(f"⚠️  조인트 '{joint_name}': dynamics 정의 없음 (damping/friction 권장)")
    
    print()
    
    # 3. 결과 출력
    print("=" * 70)
    if not issues:
        print("✅ 모든 검증 통과!")
        print("=" * 70)
        return True, []
    else:
        print(f"⚠️  {len(issues)}개 이슈 발견:")
        print("=" * 70)
        for issue in issues:
            print(f"  {issue}")
        print("=" * 70)
        
        # 치명적 에러 확인
        critical_errors = [i for i in issues if i.startswith("❌")]
        if critical_errors:
            return False, issues
        else:
            return True, issues


def main():
    """메인 함수"""
    if len(sys.argv) < 2:
        print("사용법: python validate_urdf.py <urdf_file>")
        print()
        print("예시:")
        print("  python scripts/urdf/validate_urdf.py assets/roarm_m3/urdf/roarm_m3_complete.urdf")
        sys.exit(1)
    
    urdf_path = Path(sys.argv[1])
    success, issues = validate_urdf(urdf_path)
    
    if not success:
        print()
        print("💡 다음 작업 권장:")
        print("   1. Mesh 파일 경로 확인")
        print("   2. Inertial/Collision geometry 추가")
        print("   3. Joint limits 및 dynamics 설정")
        sys.exit(1)
    else:
        print()
        print("✨ URDF가 Isaac Sim import 준비 완료!")
        sys.exit(0)


if __name__ == "__main__":
    main()
