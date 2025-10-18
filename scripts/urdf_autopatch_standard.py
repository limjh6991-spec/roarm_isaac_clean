#!/usr/bin/env python3
"""
URDF 자동 표준화 패처
- 모든 <visual>/<collision>을 링크별 STL로 교체
- <origin xyz="0 0 0" rpy="0 0 0"> 강제
- scale="1 1 1" 강제 (STL에 이미 베이크됨)
- 관성 자동 계산 (충돌 프리미티브 기반)

사용법:
    python urdf_autopatch_standard.py \
        --input assets/roarm_m3/urdf/roarm_m3_v3_transformed.urdf \
        --output assets/roarm_m3/urdf/roarm_m3_standard.urdf \
        --mesh-dir assets/roarm_m3/meshes
"""

import xml.etree.ElementTree as ET
import argparse
import sys
from pathlib import Path
import math

# 링크별 충돌 형상 정의 (관성 계산용)
COLLISION_PRIMITIVES = {
    "base_link": {"type": "cylinder", "radius": 0.04, "length": 0.06, "mass": 0.5},
    "link_1": {"type": "cylinder", "radius": 0.035, "length": 0.08, "mass": 0.3},
    "link_2": {"type": "box", "size": [0.16, 0.04, 0.04], "mass": 0.25},
    "link_3": {"type": "box", "size": [0.16, 0.035, 0.035], "mass": 0.2},
    "link_4": {"type": "box", "size": [0.12, 0.03, 0.03], "mass": 0.15},
    "link_5": {"type": "cylinder", "radius": 0.025, "length": 0.06, "mass": 0.1},
    "gripper_base": {"type": "box", "size": [0.06, 0.04, 0.02], "mass": 0.05},
    "gripper_left_finger": {"type": "box", "size": [0.01, 0.02, 0.04], "mass": 0.01},
    "gripper_right_finger": {"type": "box", "size": [0.01, 0.02, 0.04], "mass": 0.01},
}


def calculate_inertia_box(mass, size):
    """Box 관성 텐서 계산: Ixx = 1/12 * m * (y^2 + z^2)"""
    x, y, z = size
    return {
        "ixx": mass / 12.0 * (y**2 + z**2),
        "iyy": mass / 12.0 * (x**2 + z**2),
        "izz": mass / 12.0 * (x**2 + y**2),
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }


def calculate_inertia_cylinder(mass, radius, length):
    """Cylinder 관성 텐서 (Z축): Ixx=Iyy = 1/12*m*(3r^2+l^2), Izz = 1/2*m*r^2"""
    return {
        "ixx": mass / 12.0 * (3 * radius**2 + length**2),
        "iyy": mass / 12.0 * (3 * radius**2 + length**2),
        "izz": 0.5 * mass * radius**2,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }


def get_inertia_for_link(link_name):
    """링크 이름으로 관성 값 계산"""
    if link_name not in COLLISION_PRIMITIVES:
        # 기본값 (작은 질량)
        return {"mass": 0.01, "origin": [0, 0, 0], "inertia": calculate_inertia_box(0.01, [0.01, 0.01, 0.01])}

    prim = COLLISION_PRIMITIVES[link_name]
    mass = prim["mass"]

    if prim["type"] == "box":
        inertia = calculate_inertia_box(mass, prim["size"])
        origin = [0, 0, 0]  # box는 중심
    elif prim["type"] == "cylinder":
        inertia = calculate_inertia_cylinder(mass, prim["radius"], prim["length"])
        origin = [0, 0, prim["length"] / 2]  # cylinder는 중심
    else:
        inertia = calculate_inertia_box(mass, [0.01, 0.01, 0.01])
        origin = [0, 0, 0]

    return {"mass": mass, "origin": origin, "inertia": inertia}


def fix_origin_element(parent_elem, tag_name="origin"):
    """<origin>을 [0,0,0] [0,0,0]으로 강제"""
    origin = parent_elem.find(tag_name)
    if origin is None:
        origin = ET.SubElement(parent_elem, tag_name)
    origin.set("xyz", "0 0 0")
    origin.set("rpy", "0 0 0")


def fix_visual_collision(link_elem, link_name, mesh_dir, is_visual=True):
    """<visual> 또는 <collision>을 링크별 STL로 교체"""
    tag = "visual" if is_visual else "collision"
    subdir = "visual" if is_visual else "collision"

    # 기존 태그 모두 제거
    for elem in link_elem.findall(tag):
        link_elem.remove(elem)

    # 새 태그 생성
    new_elem = ET.SubElement(link_elem, tag)
    fix_origin_element(new_elem)

    geom = ET.SubElement(new_elem, "geometry")
    mesh_elem = ET.SubElement(geom, "mesh")

    # 절대 경로로 STL 지정 (file:// 없이, 원본 v3_transformed 방식)
    from pathlib import Path
    import os
    
    # mesh_dir를 절대 경로로 변환
    if mesh_dir.startswith("/"):
        # 이미 절대 경로
        abs_mesh_dir = Path(mesh_dir)
    else:
        # 상대 경로면 현재 작업 디렉토리 기준으로 절대 경로 생성
        abs_mesh_dir = Path(os.getcwd()) / "assets" / "roarm_m3" / mesh_dir
    
    stl_path = f"{abs_mesh_dir}/{subdir}/{link_name}.stl"
    mesh_elem.set("filename", stl_path)
    mesh_elem.set("scale", "0.001 0.001 0.001")  # STL은 mm 단위, URDF는 m 단위

    if is_visual:
        # material 유지 (기존 것 복사)
        material = ET.SubElement(new_elem, "material")
        material.set("name", "gray")  # 기본값


def fix_inertial(link_elem, link_name):
    """<inertial>을 자동 계산된 값으로 교체"""
    inertial_data = get_inertia_for_link(link_name)

    # 기존 inertial 제거
    for elem in link_elem.findall("inertial"):
        link_elem.remove(elem)

    # 새 inertial 생성
    inertial = ET.SubElement(link_elem, "inertial")

    # mass
    mass_elem = ET.SubElement(inertial, "mass")
    mass_elem.set("value", f"{inertial_data['mass']:.4f}")

    # origin
    origin_elem = ET.SubElement(inertial, "origin")
    orig = inertial_data["origin"]
    origin_elem.set("xyz", f"{orig[0]:.4f} {orig[1]:.4f} {orig[2]:.4f}")
    origin_elem.set("rpy", "0 0 0")

    # inertia
    inertia_elem = ET.SubElement(inertial, "inertia")
    inert = inertial_data["inertia"]
    inertia_elem.set("ixx", f"{inert['ixx']:.8f}")
    inertia_elem.set("iyy", f"{inert['iyy']:.8f}")
    inertia_elem.set("izz", f"{inert['izz']:.8f}")
    inertia_elem.set("ixy", f"{inert['ixy']:.8f}")
    inertia_elem.set("ixz", f"{inert['ixz']:.8f}")
    inertia_elem.set("iyz", f"{inert['iyz']:.8f}")


def patch_urdf(input_path, output_path, mesh_dir):
    """URDF 파일을 표준화"""
    tree = ET.parse(input_path)
    root = tree.getroot()
    
    # mesh_dir를 입력 URDF 기준 절대 경로로 변환
    input_dir = input_path.parent
    mesh_dir_abs = (input_dir.parent / mesh_dir).resolve()

    links_processed = 0

    for link in root.findall(".//link"):
        link_name = link.get("name")

        if link_name == "world":
            continue  # world는 건드리지 않음

        print(f"[Processing] {link_name}")

        # 1. Visual 교체
        stl_path_abs = mesh_dir_abs / "visual" / f"{link_name}.stl"
        if stl_path_abs.exists():
            fix_visual_collision(link, link_name, mesh_dir, is_visual=True)
            print(f"  ✓ Visual: {link_name}.stl")
        else:
            print(f"  ⚠ Visual STL 없음: {stl_path_abs}")

        # 2. Collision 교체
        stl_path_abs = mesh_dir_abs / "collision" / f"{link_name}.stl"
        if stl_path_abs.exists():
            fix_visual_collision(link, link_name, mesh_dir, is_visual=False)
            print(f"  ✓ Collision: {link_name}.stl")
        else:
            print(f"  ⚠ Collision STL 없음, primitive 유지")

        # 3. Inertial 자동 계산
        fix_inertial(link, link_name)
        print(f"  ✓ Inertial: mass={get_inertia_for_link(link_name)['mass']:.4f}kg")

        links_processed += 1

    # 저장
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"\n[완료] {links_processed}개 링크 처리 완료")
    print(f"출력: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="URDF 표준화 패처")
    parser.add_argument("--input", required=True, help="입력 URDF 파일")
    parser.add_argument("--output", required=True, help="출력 URDF 파일")
    parser.add_argument("--mesh-dir", default="meshes", help="메시 디렉토리 (상대경로)")
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        print(f"[ERROR] 입력 파일 없음: {input_path}")
        sys.exit(1)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    patch_urdf(input_path, output_path, args.mesh_dir)


if __name__ == "__main__":
    main()
