#!/usr/bin/env python3
"""
🤖 RoArm-M3 STL 메시 기반 URDF 자동 생성
===============================================

현재 primitive shapes(box, cylinder)를 실제 STL 메시로 대체하여
sim-to-real gap을 최소화합니다.

Author: RoArm-M3 RL Team
Date: 2025-10-30
"""

import os
from pathlib import Path

# 프로젝트 경로
PROJECT_ROOT = Path(__file__).parent.parent.parent
MESH_DIR = PROJECT_ROOT / "assets/roarm_m3/meshes/visual"
OUTPUT_DIR = PROJECT_ROOT / "assets/roarm_m3/urdf"

# URDF 템플릿
URDF_HEADER = """<?xml version="1.0" ?>
<robot name="roarm_m3_mesh">
  <!-- Materials -->
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>
  <material name="dark_gray">
    <color rgba="0.3 0.3 0.3 1.0"/>
  </material>
  <material name="silver">
    <color rgba="0.7 0.7 0.75 1.0"/>
  </material>
  <material name="gray">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <!-- World Link (Fixed) -->
  <link name="world"/>
"""

# 링크 정의 (STL 메시 기반)
LINK_CONFIGS = {
    "base_link": {
        "mass": 0.2,  # kg
        "mesh": "base_link.stl",
        "material": "dark_gray",
        "origin_xyz": "0 0 0.02",
        "origin_rpy": "0 0 0",
        "inertia": {
            "ixx": 0.0001, "iyy": 0.0001, "izz": 0.0002,
            "ixy": 0, "ixz": 0, "iyz": 0
        }
    },
    "link_1": {
        "mass": 0.1,
        "mesh": "link_1.stl",
        "material": "black",
        "origin_xyz": "0 0 0.04",
        "origin_rpy": "0 0 0",
        "inertia": {
            "ixx": 0.00008, "iyy": 0.00008, "izz": 0.00006,
            "ixy": 0, "ixz": 0, "iyz": 0
        }
    },
    "link_2": {
        "mass": 0.15,
        "mesh": "link_2.stl",
        "material": "silver",
        "origin_xyz": "0.06 0 0",
        "origin_rpy": "0 0 0",
        "inertia": {
            "ixx": 0.00015, "iyy": 0.00015, "izz": 0.00005,
            "ixy": 0, "ixz": 0, "iyz": 0
        }
    },
    "link_3": {
        "mass": 0.15,
        "mesh": "link_3.stl",
        "material": "silver",
        "origin_xyz": "0.09 0 0",
        "origin_rpy": "0 0 0",
        "inertia": {
            "ixx": 0.00018, "iyy": 0.00018, "izz": 0.00005,
            "ixy": 0, "ixz": 0, "iyz": 0
        }
    },
    "link_4": {
        "mass": 0.08,
        "mesh": "link_4.stl",
        "material": "silver",
        "origin_xyz": "0.07 0 0",
        "origin_rpy": "0 0 0",
        "inertia": {
            "ixx": 0.00008, "iyy": 0.00008, "izz": 0.00003,
            "ixy": 0, "ixz": 0, "iyz": 0
        }
    },
    "link_5": {
        "mass": 0.08,
        "mesh": "link_5.stl",
        "material": "black",
        "origin_xyz": "0.04 0 0",
        "origin_rpy": "0 0 0",
        "inertia": {
            "ixx": 0.00005, "iyy": 0.00005, "izz": 0.00003,
            "ixy": 0, "ixz": 0, "iyz": 0
        }
    },
    "gripper_base": {
        "mass": 0.05,
        "mesh": "gripper_base.stl",
        "material": "silver",
        "origin_xyz": "0.04 0 0",
        "origin_rpy": "0 0 0",
        "inertia": {
            "ixx": 0.00003, "iyy": 0.00003, "izz": 0.00002,
            "ixy": 0, "ixz": 0, "iyz": 0
        }
    },
    "gripper_left_finger": {
        "mass": 0.02,
        "mesh": "gripper_left_finger.stl",
        "material": "black",
        "origin_xyz": "0 0.015 0",
        "origin_rpy": "0 0 0",
        "inertia": {
            "ixx": 0.00001, "iyy": 0.00001, "izz": 0.00001,
            "ixy": 0, "ixz": 0, "iyz": 0
        }
    },
    "gripper_right_finger": {
        "mass": 0.02,
        "mesh": "gripper_right_finger.stl",
        "material": "black",
        "origin_xyz": "0 -0.015 0",
        "origin_rpy": "0 0 0",
        "inertia": {
            "ixx": 0.00001, "iyy": 0.00001, "izz": 0.00001,
            "ixy": 0, "ixz": 0, "iyz": 0
        }
    }
}

# 조인트 정의
JOINT_CONFIGS = [
    {
        "name": "root_joint",
        "type": "fixed",
        "parent": "world",
        "child": "base_link",
        "origin_xyz": "0 0 0",
        "origin_rpy": "0 0 0"
    },
    {
        "name": "joint_1",
        "type": "revolute",
        "parent": "base_link",
        "child": "link_1",
        "origin_xyz": "0 0 0.04",
        "origin_rpy": "0 0 0",
        "axis": "0 0 1",
        "limit": {
            "lower": -3.14,
            "upper": 3.14,
            "effort": 5.0,
            "velocity": 2.0
        },
        "dynamics": {
            "damping": 0.1,
            "friction": 0.01
        }
    },
    {
        "name": "joint_2",
        "type": "revolute",
        "parent": "link_1",
        "child": "link_2",
        "origin_xyz": "0 0 0.08",
        "origin_rpy": "0 -1.57 0",
        "axis": "0 0 1",
        "limit": {
            "lower": -1.57,
            "upper": 1.57,
            "effort": 5.0,
            "velocity": 2.0
        },
        "dynamics": {
            "damping": 0.1,
            "friction": 0.01
        }
    },
    {
        "name": "joint_3",
        "type": "revolute",
        "parent": "link_2",
        "child": "link_3",
        "origin_xyz": "0.12 0 0",
        "origin_rpy": "0 0 0",
        "axis": "0 0 1",
        "limit": {
            "lower": -1.57,
            "upper": 1.57,
            "effort": 5.0,
            "velocity": 2.0
        },
        "dynamics": {
            "damping": 0.1,
            "friction": 0.01
        }
    },
    {
        "name": "joint_4",
        "type": "revolute",
        "parent": "link_3",
        "child": "link_4",
        "origin_xyz": "0.18 0 0",
        "origin_rpy": "0 0 0",
        "axis": "0 0 1",
        "limit": {
            "lower": -1.57,
            "upper": 1.57,
            "effort": 3.0,
            "velocity": 2.0
        },
        "dynamics": {
            "damping": 0.05,
            "friction": 0.01
        }
    },
    {
        "name": "joint_5",
        "type": "revolute",
        "parent": "link_4",
        "child": "link_5",
        "origin_xyz": "0.14 0 0",
        "origin_rpy": "0 0 0",
        "axis": "0 0 1",
        "limit": {
            "lower": -3.14,
            "upper": 3.14,
            "effort": 2.0,
            "velocity": 2.0
        },
        "dynamics": {
            "damping": 0.05,
            "friction": 0.01
        }
    },
    {
        "name": "gripper_joint",
        "type": "fixed",
        "parent": "link_5",
        "child": "gripper_base",
        "origin_xyz": "0.08 0 0",
        "origin_rpy": "0 0 0"
    },
    {
        "name": "gripper_left_joint",
        "type": "prismatic",
        "parent": "gripper_base",
        "child": "gripper_left_finger",
        "origin_xyz": "0.02 0 0",
        "origin_rpy": "0 0 0",
        "axis": "0 1 0",
        "limit": {
            "lower": 0.0,
            "upper": 0.04,
            "effort": 1.0,
            "velocity": 0.1
        },
        "dynamics": {
            "damping": 0.1,
            "friction": 0.05
        }
    },
    {
        "name": "gripper_right_joint",
        "type": "prismatic",
        "parent": "gripper_base",
        "child": "gripper_right_finger",
        "origin_xyz": "0.02 0 0",
        "origin_rpy": "0 0 0",
        "axis": "0 1 0",
        "limit": {
            "lower": -0.04,
            "upper": 0.0,
            "effort": 1.0,
            "velocity": 0.1
        },
        "mimic": {
            "joint": "gripper_left_joint",
            "multiplier": -1.0
        },
        "dynamics": {
            "damping": 0.1,
            "friction": 0.05
        }
    }
]


def generate_link(name: str, config: dict) -> str:
    """링크 XML 생성"""
    mesh_path = f"../meshes/visual/{config['mesh']}"
    
    link_xml = f"""
  <!-- {name.upper()} -->
  <link name="{name}">
    <!-- Visual (STL Mesh) -->
    <visual>
      <origin xyz="{config['origin_xyz']}" rpy="{config['origin_rpy']}"/>
      <geometry>
        <mesh filename="{mesh_path}" scale="1.0 1.0 1.0"/>
      </geometry>
      <material name="{config['material']}"/>
    </visual>
    
    <!-- Collision (Simplified) -->
    <collision>
      <origin xyz="{config['origin_xyz']}" rpy="{config['origin_rpy']}"/>
      <geometry>
        <mesh filename="{mesh_path}" scale="1.0 1.0 1.0"/>
      </geometry>
    </collision>
    
    <!-- Inertial Properties -->
    <inertial>
      <mass value="{config['mass']:.4f}"/>
      <origin xyz="{config['origin_xyz']}" rpy="{config['origin_rpy']}"/>
      <inertia ixx="{config['inertia']['ixx']:.8f}" 
               iyy="{config['inertia']['iyy']:.8f}" 
               izz="{config['inertia']['izz']:.8f}" 
               ixy="{config['inertia']['ixy']:.8f}" 
               ixz="{config['inertia']['ixz']:.8f}" 
               iyz="{config['inertia']['iyz']:.8f}"/>
    </inertial>
  </link>
"""
    return link_xml


def generate_joint(config: dict) -> str:
    """조인트 XML 생성"""
    joint_xml = f"""
  <!-- {config['name'].upper()} -->
  <joint name="{config['name']}" type="{config['type']}">
    <parent link="{config['parent']}"/>
    <child link="{config['child']}"/>
    <origin xyz="{config['origin_xyz']}" rpy="{config['origin_rpy']}"/>
"""
    
    if config['type'] in ['revolute', 'prismatic']:
        joint_xml += f"""    <axis xyz="{config['axis']}"/>
    <limit lower="{config['limit']['lower']:.2f}" 
           upper="{config['limit']['upper']:.2f}" 
           effort="{config['limit']['effort']:.1f}" 
           velocity="{config['limit']['velocity']:.1f}"/>
    <dynamics damping="{config['dynamics']['damping']:.2f}" 
              friction="{config['dynamics']['friction']:.2f}"/>
"""
    
    if 'mimic' in config:
        joint_xml += f"""    <mimic joint="{config['mimic']['joint']}" 
           multiplier="{config['mimic']['multiplier']:.1f}"/>
"""
    
    joint_xml += "  </joint>\n"
    return joint_xml


def main():
    """URDF 생성"""
    print("=" * 80)
    print("🤖 RoArm-M3 STL 메시 기반 URDF 자동 생성")
    print("=" * 80)
    print()
    
    # STL 파일 확인
    print("📦 STL 메시 파일 확인...")
    missing_meshes = []
    for name, config in LINK_CONFIGS.items():
        mesh_path = MESH_DIR / config['mesh']
        if not mesh_path.exists():
            missing_meshes.append(config['mesh'])
            print(f"   ❌ {config['mesh']} - 파일 없음!")
        else:
            size_mb = mesh_path.stat().st_size / 1024 / 1024
            print(f"   ✅ {config['mesh']} ({size_mb:.2f} MB)")
    
    if missing_meshes:
        print(f"\n❌ {len(missing_meshes)}개의 메시 파일이 없습니다!")
        return
    
    print(f"\n✅ 모든 STL 메시 파일 확인 완료! ({len(LINK_CONFIGS)}개)\n")
    
    # URDF 생성
    print("📝 URDF 생성 중...")
    urdf_content = URDF_HEADER
    
    # 링크 추가
    for name, config in LINK_CONFIGS.items():
        urdf_content += generate_link(name, config)
    
    # 조인트 추가
    for joint_config in JOINT_CONFIGS:
        urdf_content += generate_joint(joint_config)
    
    urdf_content += "</robot>\n"
    
    # 파일 저장
    output_path = OUTPUT_DIR / "roarm_m3_mesh.urdf"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(urdf_content)
    
    print(f"✅ URDF 생성 완료!")
    print(f"   파일: {output_path}")
    print(f"   크기: {len(urdf_content)} bytes")
    print()
    
    # 통계
    print("📊 생성된 URDF 통계:")
    print(f"   - 링크 수: {len(LINK_CONFIGS)}")
    print(f"   - 조인트 수: {len(JOINT_CONFIGS)}")
    print(f"   - 총 질량: {sum(c['mass'] for c in LINK_CONFIGS.values()):.3f} kg")
    print()
    
    print("=" * 80)
    print("🎯 다음 단계:")
    print("=" * 80)
    print()
    print("1. URDF 검증:")
    print(f"   python scripts/debug/test_mesh_urdf.py")
    print()
    print("2. 환경 설정 업데이트:")
    print(f"   URDF_PATH = '{output_path}'")
    print()
    print("3. 새 URDF로 재학습:")
    print(f"   python scripts/rl/train_v4.2_mesh.py")
    print()


if __name__ == "__main__":
    main()
