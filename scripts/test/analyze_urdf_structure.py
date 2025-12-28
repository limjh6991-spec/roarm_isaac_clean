#!/usr/bin/env python3
"""
URDF Structure Analyzer

원본 URDF와 카메라 포함 URDF를 완전히 분석하여
구조적 차이점과 물리적 속성을 비교합니다.

목적:
1. Joint 계층 구조 비교
2. Link 물리 속성 (mass, inertia, COM) 비교
3. Fixed joint 처리 방식 확인
4. World-base 연결 구조 확인

작성일: 2025-11-02
"""

import xml.etree.ElementTree as ET
import os
from pathlib import Path
from typing import Dict, List, Tuple
import numpy as np


class URDFAnalyzer:
    """URDF 파일을 분석하고 구조를 비교하는 클래스"""
    
    def __init__(self, urdf_path: str):
        self.urdf_path = urdf_path
        self.tree = ET.parse(urdf_path)
        self.root = self.tree.getroot()
        
    def get_links(self) -> Dict[str, Dict]:
        """모든 링크와 그 속성을 추출"""
        links = {}
        for link in self.root.findall('link'):
            name = link.get('name')
            link_info = {
                'name': name,
                'has_inertial': link.find('inertial') is not None,
                'has_visual': link.find('visual') is not None,
                'has_collision': link.find('collision') is not None,
            }
            
            # Inertial 정보
            inertial = link.find('inertial')
            if inertial is not None:
                origin = inertial.find('origin')
                mass = inertial.find('mass')
                inertia = inertial.find('inertia')
                
                if origin is not None:
                    link_info['com'] = origin.get('xyz', '0 0 0')
                    link_info['com_rpy'] = origin.get('rpy', '0 0 0')
                
                if mass is not None:
                    link_info['mass'] = float(mass.get('value', 0))
                
                if inertia is not None:
                    link_info['inertia'] = {
                        'ixx': float(inertia.get('ixx', 0)),
                        'ixy': float(inertia.get('ixy', 0)),
                        'ixz': float(inertia.get('ixz', 0)),
                        'iyy': float(inertia.get('iyy', 0)),
                        'iyz': float(inertia.get('iyz', 0)),
                        'izz': float(inertia.get('izz', 0)),
                    }
            
            links[name] = link_info
        
        return links
    
    def get_joints(self) -> Dict[str, Dict]:
        """모든 조인트와 그 속성을 추출"""
        joints = {}
        for joint in self.root.findall('joint'):
            name = joint.get('name')
            joint_type = joint.get('type')
            
            parent = joint.find('parent')
            child = joint.find('child')
            origin = joint.find('origin')
            axis = joint.find('axis')
            limit = joint.find('limit')
            
            joint_info = {
                'name': name,
                'type': joint_type,
                'parent': parent.get('link') if parent is not None else None,
                'child': child.get('link') if child is not None else None,
            }
            
            if origin is not None:
                joint_info['origin_xyz'] = origin.get('xyz', '0 0 0')
                joint_info['origin_rpy'] = origin.get('rpy', '0 0 0')
            
            if axis is not None:
                joint_info['axis'] = axis.get('xyz', '0 0 1')
            
            if limit is not None:
                joint_info['limit'] = {
                    'lower': float(limit.get('lower', 0)),
                    'upper': float(limit.get('upper', 0)),
                    'effort': float(limit.get('effort', 0)),
                    'velocity': float(limit.get('velocity', 0)),
                }
            
            joints[name] = joint_info
        
        return joints
    
    def build_joint_tree(self, joints: Dict[str, Dict]) -> Dict[str, List[str]]:
        """Joint 계층 구조를 트리로 구축"""
        tree = {}
        for joint_name, joint_info in joints.items():
            parent = joint_info['parent']
            child = joint_info['child']
            
            if parent not in tree:
                tree[parent] = []
            tree[parent].append((joint_name, child))
        
        return tree
    
    def print_tree(self, tree: Dict[str, List[str]], link: str = 'world', indent: int = 0):
        """트리 구조를 출력"""
        if link in tree:
            for joint_name, child_link in tree[link]:
                joint = self.get_joints()[joint_name]
                origin = joint.get('origin_xyz', '0 0 0')
                print(f"{'  ' * indent}├─ [{joint['type']}] {joint_name}")
                print(f"{'  ' * indent}│  origin: {origin}")
                print(f"{'  ' * indent}└─> {child_link}")
                self.print_tree(tree, child_link, indent + 1)
    
    def calculate_total_mass(self, links: Dict[str, Dict]) -> float:
        """전체 질량 계산"""
        total_mass = 0
        for link_info in links.values():
            if 'mass' in link_info:
                total_mass += link_info['mass']
        return total_mass
    
    def calculate_com(self, links: Dict[str, Dict], joints: Dict[str, Dict]) -> np.ndarray:
        """전체 질량 중심 계산 (간단한 버전)"""
        total_mass = 0
        com = np.array([0.0, 0.0, 0.0])
        
        for link_name, link_info in links.items():
            if 'mass' in link_info and 'com' in link_info:
                mass = link_info['mass']
                link_com = np.array([float(x) for x in link_info['com'].split()])
                com += mass * link_com
                total_mass += mass
        
        if total_mass > 0:
            com /= total_mass
        
        return com
    
    def print_summary(self):
        """URDF 요약 정보 출력"""
        links = self.get_links()
        joints = self.get_joints()
        tree = self.build_joint_tree(joints)
        
        print(f"\n{'='*80}")
        print(f"URDF Analysis: {os.path.basename(self.urdf_path)}")
        print(f"{'='*80}")
        
        print(f"\n📊 Summary:")
        print(f"  Total Links: {len(links)}")
        print(f"  Total Joints: {len(joints)}")
        print(f"  Total Mass: {self.calculate_total_mass(links):.6f} kg")
        print(f"  System COM: {self.calculate_com(links, joints)}")
        
        print(f"\n📦 Links:")
        for name, info in links.items():
            status = []
            if info['has_inertial']:
                status.append(f"mass={info.get('mass', 0):.4f}kg")
            if info['has_visual']:
                status.append("visual")
            if info['has_collision']:
                status.append("collision")
            
            status_str = ", ".join(status) if status else "EMPTY"
            print(f"  - {name:30s} [{status_str}]")
            
            if 'com' in info:
                print(f"    COM: {info['com']}")
        
        print(f"\n🔗 Joint Tree:")
        self.print_tree(tree, 'world', 0)
        
        print(f"\n🔧 Joint Details:")
        for name, info in joints.items():
            print(f"\n  {name}:")
            print(f"    Type: {info['type']}")
            print(f"    {info['parent']} -> {info['child']}")
            print(f"    Origin: {info.get('origin_xyz', 'N/A')}")
            if 'limit' in info:
                lim = info['limit']
                print(f"    Limit: [{lim['lower']:.4f}, {lim['upper']:.4f}]")


def compare_urdfs(urdf1_path: str, urdf2_path: str):
    """두 URDF 파일을 비교"""
    print("\n" + "="*80)
    print("🔍 URDF COMPARISON")
    print("="*80)
    
    analyzer1 = URDFAnalyzer(urdf1_path)
    analyzer2 = URDFAnalyzer(urdf2_path)
    
    links1 = analyzer1.get_links()
    links2 = analyzer2.get_links()
    joints1 = analyzer1.get_joints()
    joints2 = analyzer2.get_joints()
    
    # Links 비교
    print(f"\n📦 Link Differences:")
    all_links = set(links1.keys()) | set(links2.keys())
    
    for link in sorted(all_links):
        if link in links1 and link not in links2:
            print(f"  - {link}: Only in {os.path.basename(urdf1_path)}")
        elif link in links2 and link not in links1:
            print(f"  + {link}: Only in {os.path.basename(urdf2_path)}")
        else:
            # 속성 비교
            info1 = links1[link]
            info2 = links2[link]
            
            diffs = []
            if info1.get('mass') != info2.get('mass'):
                diffs.append(f"mass: {info1.get('mass')} vs {info2.get('mass')}")
            if info1.get('com') != info2.get('com'):
                diffs.append(f"COM: {info1.get('com')} vs {info2.get('com')}")
            
            if diffs:
                print(f"  ~ {link}: {', '.join(diffs)}")
    
    # Joints 비교
    print(f"\n🔗 Joint Differences:")
    all_joints = set(joints1.keys()) | set(joints2.keys())
    
    for joint in sorted(all_joints):
        if joint in joints1 and joint not in joints2:
            print(f"  - {joint}: Only in {os.path.basename(urdf1_path)}")
        elif joint in joints2 and joint not in joints1:
            print(f"  + {joint}: Only in {os.path.basename(urdf2_path)}")
        else:
            # 속성 비교
            info1 = joints1[joint]
            info2 = joints2[joint]
            
            diffs = []
            if info1['type'] != info2['type']:
                diffs.append(f"type: {info1['type']} vs {info2['type']}")
            if info1.get('origin_xyz') != info2.get('origin_xyz'):
                diffs.append(f"origin: {info1.get('origin_xyz')} vs {info2.get('origin_xyz')}")
            if info1['parent'] != info2['parent']:
                diffs.append(f"parent: {info1['parent']} vs {info2['parent']}")
            if info1['child'] != info2['child']:
                diffs.append(f"child: {info1['child']} vs {info2['child']}")
            
            if diffs:
                print(f"  ~ {joint}:")
                for diff in diffs:
                    print(f"      {diff}")


def main():
    """메인 함수"""
    # 경로 설정
    project_dir = Path(__file__).resolve().parents[2]
    urdf_dir = project_dir / "assets" / "roarm_m3" / "urdf"
    
    original_urdf = urdf_dir / "roarm_m3.generated.urdf"
    camera_urdf = urdf_dir / "roarm_m3_with_d405.urdf"
    
    # 1. 원본 URDF 분석
    print("\n" + "="*80)
    print("1️⃣  ORIGINAL URDF ANALYSIS")
    print("="*80)
    analyzer1 = URDFAnalyzer(str(original_urdf))
    analyzer1.print_summary()
    
    # 2. 카메라 URDF 분석
    print("\n" + "="*80)
    print("2️⃣  CAMERA URDF ANALYSIS")
    print("="*80)
    analyzer2 = URDFAnalyzer(str(camera_urdf))
    analyzer2.print_summary()
    
    # 3. 비교
    compare_urdfs(str(original_urdf), str(camera_urdf))
    
    print("\n" + "="*80)
    print("✅ Analysis Complete")
    print("="*80)


if __name__ == "__main__":
    main()
