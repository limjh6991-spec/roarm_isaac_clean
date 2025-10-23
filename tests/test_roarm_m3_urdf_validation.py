"""
Test: URDF Validation
URDF 파일의 기본 무결성 테스트
"""

import pytest
from pathlib import Path


def test_urdf_file_exists():
    """
    완전한 URDF 파일이 존재하는지 확인
    """
    urdf_path = Path("assets/roarm_m3/urdf/roarm_m3.urdf")
    assert urdf_path.exists(), f"URDF file not found: {urdf_path}"


def test_urdf_xml_valid():
    """
    URDF 파일이 유효한 XML인지 확인
    """
    import xml.etree.ElementTree as ET
    
    urdf_path = Path("assets/roarm_m3/urdf/roarm_m3.urdf")
    
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        assert root.tag == "robot", "Root element must be 'robot'"
    except ET.ParseError as e:
        pytest.fail(f"Invalid XML: {e}")


def test_urdf_has_links():
    """
    URDF에 link가 정의되어 있는지 확인
    """
    import xml.etree.ElementTree as ET
    
    urdf_path = Path("assets/roarm_m3/urdf/roarm_m3.urdf")
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    links = root.findall("link")
    assert len(links) > 0, "No links found in URDF"


def test_urdf_has_joints():
    """
    URDF에 joint가 정의되어 있는지 확인
    """
    import xml.etree.ElementTree as ET
    
    urdf_path = Path("assets/roarm_m3/urdf/roarm_m3.urdf")
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    joints = root.findall("joint")
    assert len(joints) > 0, "No joints found in URDF"


def test_primitive_shapes_exist():
    """
    URDF에서 참조하는 프리미티브 형상이 실제로 존재하는지 확인
    """
    import xml.etree.ElementTree as ET
    
    urdf_path = Path("assets/roarm_m3/urdf/roarm_m3.urdf")
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    geometry_elements = root.findall(".//geometry")
    
    for geometry_elem in geometry_elements:
        children = list(geometry_elem)
        assert len(children) > 0, "No shape found in geometry"
        shape_elem = children[0]
        assert shape_elem.tag in ["box", "cylinder", "sphere"], f"Unknown shape: {shape_elem.tag}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
