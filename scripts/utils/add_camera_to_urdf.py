#!/usr/bin/env python3
"""
Add D405 Camera to RoArm-M3 URDF

원본 로봇 구조를 유지하면서 카메라만 추가합니다.
기존 kinematic chain을 변경하지 않고 gripper_link에 camera를 fixed joint로 부착합니다.

목적:
1. 원본 roarm_m3.generated.urdf 구조 100% 유지
2. gripper_link에 camera_link를 fixed joint로 추가
3. 카메라 optical frames 추가

작성일: 2025-11-02
"""

import xml.etree.ElementTree as ET
from pathlib import Path
import argparse


def create_camera_link() -> ET.Element:
    """D405 카메라 링크 생성"""
    link = ET.Element('link', name='camera_link')
    
    # Inertial properties (D405 실제 무게: ~60g)
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.SubElement(inertial, 'mass', value='0.06')
    ET.SubElement(inertial, 'inertia',
                  ixx='0.0001', ixy='0', ixz='0',
                  iyy='0.0001', iyz='0',
                  izz='0.0001')
    
    # Visual
    visual = ET.SubElement(link, 'visual')
    ET.SubElement(visual, 'origin', xyz='0 0 0', rpy='0 0 0')
    geometry = ET.SubElement(visual, 'geometry')
    # 간단한 box로 표현 (90mm x 25mm x 25mm)
    ET.SubElement(geometry, 'box', size='0.09 0.025 0.025')
    material = ET.SubElement(visual, 'material', name='camera_material')
    # 진한 회색으로 변경
    ET.SubElement(material, 'color', rgba='0.15 0.15 0.15 1.0')
    
    # Collision
    collision = ET.SubElement(link, 'collision')
    ET.SubElement(collision, 'origin', xyz='0 0 0', rpy='0 0 0')
    geometry = ET.SubElement(collision, 'geometry')
    ET.SubElement(geometry, 'box', size='0.09 0.025 0.025')
    
    return link


def create_camera_mount_joint() -> ET.Element:
    """그리퍼에서 카메라로의 고정 조인트"""
    joint = ET.Element('joint', name='camera_mount_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='gripper_link')
    ET.SubElement(joint, 'child', link='camera_link')
    # 그리퍼 중앙, 앞쪽 5cm, 위쪽 5cm 위치에 장착 (아래가 아닌 위)
    ET.SubElement(joint, 'origin', xyz='0.05 0 0.05', rpy='0 0 0')
    return joint


def create_camera_frames() -> tuple:
    """카메라 optical frames 생성"""
    frames = []
    joints = []
    
    # Depth frame
    depth_frame = ET.Element('link', name='camera_depth_frame')
    frames.append(depth_frame)
    
    depth_joint = ET.Element('joint', name='camera_depth_joint', type='fixed')
    ET.SubElement(depth_joint, 'parent', link='camera_link')
    ET.SubElement(depth_joint, 'child', link='camera_depth_frame')
    ET.SubElement(depth_joint, 'origin', xyz='0 0 0', rpy='0 0 0')
    joints.append(depth_joint)
    
    # Depth optical frame
    depth_optical = ET.Element('link', name='camera_depth_optical_frame')
    frames.append(depth_optical)
    
    depth_optical_joint = ET.Element('joint', name='camera_depth_optical_joint', type='fixed')
    ET.SubElement(depth_optical_joint, 'parent', link='camera_depth_frame')
    ET.SubElement(depth_optical_joint, 'child', link='camera_depth_optical_frame')
    # ROS optical frame convention: x=right, y=down, z=forward
    ET.SubElement(depth_optical_joint, 'origin', xyz='0 0 0', rpy='-1.5707963267948966 0 -1.5707963267948966')
    joints.append(depth_optical_joint)
    
    # Color frame
    color_frame = ET.Element('link', name='camera_color_frame')
    frames.append(color_frame)
    
    color_joint = ET.Element('joint', name='camera_color_joint', type='fixed')
    ET.SubElement(color_joint, 'parent', link='camera_depth_frame')
    ET.SubElement(color_joint, 'child', link='camera_color_frame')
    # RGB 센서는 depth와 약간 offset (실제 D405 스펙 기준)
    ET.SubElement(color_joint, 'origin', xyz='0 0.015 0', rpy='0 0 0')
    joints.append(color_joint)
    
    # Color optical frame
    color_optical = ET.Element('link', name='camera_color_optical_frame')
    frames.append(color_optical)
    
    color_optical_joint = ET.Element('joint', name='camera_color_optical_joint', type='fixed')
    ET.SubElement(color_optical_joint, 'parent', link='camera_color_frame')
    ET.SubElement(color_optical_joint, 'child', link='camera_color_optical_frame')
    ET.SubElement(color_optical_joint, 'origin', xyz='0 0 0', rpy='-1.5707963267948966 0 -1.5707963267948966')
    joints.append(color_optical_joint)
    
    return frames, joints


def add_camera_to_urdf(input_urdf: str, output_urdf: str):
    """원본 URDF에 카메라 추가"""
    print(f"\n{'='*80}")
    print("Adding D405 Camera to RoArm-M3 URDF")
    print(f"{'='*80}")
    print(f"Input:  {input_urdf}")
    print(f"Output: {output_urdf}")
    
    # Parse original URDF
    tree = ET.parse(input_urdf)
    root = tree.getroot()
    
    # Add camera link
    print("\n📦 Adding camera_link...")
    camera_link = create_camera_link()
    root.append(camera_link)
    
    # Add camera mount joint
    print("🔗 Adding camera_mount_joint...")
    camera_mount = create_camera_mount_joint()
    root.append(camera_mount)
    
    # Add camera frames
    print("🎥 Adding camera optical frames...")
    frames, joints = create_camera_frames()
    for frame in frames:
        root.append(frame)
    for joint in joints:
        root.append(joint)
    
    # Format XML nicely
    indent_xml(root)
    
    # Write to file
    tree.write(output_urdf, encoding='utf-8', xml_declaration=True)
    print(f"\n✅ Successfully created: {output_urdf}")
    
    # Summary
    print(f"\n📊 Summary:")
    print(f"  Added Links: camera_link, camera_depth_frame, camera_depth_optical_frame,")
    print(f"               camera_color_frame, camera_color_optical_frame")
    print(f"  Added Joints: camera_mount_joint (gripper_link -> camera_link)")
    print(f"                + 4 optical frame joints")
    print(f"  Camera Mount: xyz='0.05 0 0.05' on gripper_link (5cm forward, 5cm up)")
    print(f"  Camera Mass: 0.06 kg (60g)")


def indent_xml(elem, level=0):
    """XML을 보기 좋게 포맷팅"""
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for child in elem:
            indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def main():
    parser = argparse.ArgumentParser(description='Add D405 camera to RoArm-M3 URDF')
    parser.add_argument('--input', type=str,
                        help='Input URDF file (default: roarm_m3.generated.urdf)')
    parser.add_argument('--output', type=str,
                        help='Output URDF file (default: roarm_m3_with_camera_correct.urdf)')
    args = parser.parse_args()
    
    # Default paths
    project_dir = Path(__file__).resolve().parents[2]
    urdf_dir = project_dir / "assets" / "roarm_m3" / "urdf"
    
    input_urdf = args.input if args.input else str(urdf_dir / "roarm_m3.generated.urdf")
    output_urdf = args.output if args.output else str(urdf_dir / "roarm_m3_with_camera_correct.urdf")
    
    if not Path(input_urdf).exists():
        print(f"❌ Error: Input file not found: {input_urdf}")
        return
    
    add_camera_to_urdf(input_urdf, output_urdf)
    print(f"\n{'='*80}")


if __name__ == "__main__":
    main()
