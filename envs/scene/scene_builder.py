#!/usr/bin/env python3
"""
Scene Builder Module for Isaac Sim 5.1

Isaac Sim 5.1에서 동적 물체 생성을 위한 유틸리티
- DynamicCuboidWrapper: DynamicCuboid 대체 클래스
- create_dynamic_cuboid: USD prim 기반 큐브 생성
- create_physics_material: 마찰 물리 재질 생성
"""

import numpy as np
from typing import Optional, Tuple
from pxr import UsdGeom, UsdPhysics, Gf, PhysxSchema, UsdShade


def create_dynamic_cuboid(
    stage,
    prim_path: str,
    position: Tuple[float, float, float],
    size: float,
    color: Optional[Tuple[float, float, float]] = None,
    mass: float = 0.1,
    name: Optional[str] = None
):
    """
    Isaac Sim 5.1 replacement for DynamicCuboid.
    Creates a rigid body cube using USD prims.
    
    Args:
        stage: USD stage
        prim_path: 큐브 prim 경로
        position: 초기 위치 (x, y, z)
        size: 큐브 크기 (정육면체 한 변)
        color: RGB 색상 (0-1 범위)
        mass: 질량 (kg)
        name: 이름 (옵션)
    
    Returns:
        cube_prim: 생성된 큐브 prim
    """
    # Create Xform for the cube
    cube_xform = UsdGeom.Xform.Define(stage, prim_path)
    cube_xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    
    # Create the cube geometry
    cube_path = f"{prim_path}/Cube"
    cube_geom = UsdGeom.Cube.Define(stage, cube_path)
    cube_geom.GetSizeAttr().Set(size)
    
    # Set color
    if color is not None:
        color_arr = np.array(color)
        cube_geom.GetDisplayColorAttr().Set([Gf.Vec3f(*color_arr)])
    
    # Get the root prim for physics
    cube_prim = stage.GetPrimAtPath(prim_path)
    
    # Apply RigidBody and Collision APIs
    UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    
    # Apply collision to the geometry
    cube_geom_prim = stage.GetPrimAtPath(cube_path)
    UsdPhysics.CollisionAPI.Apply(cube_geom_prim)
    
    # Add mass
    mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
    mass_api.GetMassAttr().Set(mass)
    
    return cube_prim


def create_physics_material(
    stage,
    prim_path: str = "/World/PhysicsMaterials/HighFriction",
    static_friction: float = 1.2,
    dynamic_friction: float = 1.0,
    restitution: float = 0.1
):
    """
    고마찰 Physics Material 생성
    
    Args:
        stage: USD stage
        prim_path: Material prim 경로
        static_friction: 정지 마찰 계수
        dynamic_friction: 동적 마찰 계수
        restitution: 반발 계수
    
    Returns:
        mat_prim: 생성된 Material prim
    """
    mat_prim = stage.GetPrimAtPath(prim_path)
    
    if not mat_prim or not mat_prim.IsValid():
        mat_prim = stage.DefinePrim(prim_path, "Material")
        
        material_api = UsdPhysics.MaterialAPI.Apply(mat_prim)
        material_api.CreateStaticFrictionAttr(static_friction)
        material_api.CreateDynamicFrictionAttr(dynamic_friction)
        material_api.CreateRestitutionAttr(restitution)
    
    return mat_prim


def apply_material_to_prim(stage, prim_path: str, material_path: str):
    """
    Prim에 Material 바인딩
    
    Args:
        stage: USD stage
        prim_path: 대상 prim 경로
        material_path: Material prim 경로
    """
    prim = stage.GetPrimAtPath(prim_path)
    mat_prim = stage.GetPrimAtPath(material_path)
    
    if prim and prim.IsValid() and mat_prim and mat_prim.IsValid():
        binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
        binding_api.Bind(UsdShade.Material(mat_prim))


class DynamicCuboidWrapper:
    """
    Wrapper class to provide DynamicCuboid-like interface for Isaac Sim 5.1.
    
    기존 Isaac Sim 4.x의 DynamicCuboid와 호환되는 인터페이스 제공
    """
    
    def __init__(
        self,
        prim_path: str,
        name: str,
        position: Tuple[float, float, float],
        size: float,
        color: Tuple[float, float, float],
        mass: float = 0.1
    ):
        """
        Args:
            prim_path: USD prim 경로
            name: 객체 이름
            position: 초기 위치
            size: 큐브 크기
            color: RGB 색상
            mass: 질량
        """
        self.prim_path = prim_path
        self.name = name
        self._position = np.array(position)
        self._size = size
        self._color = color
        self._mass = mass
        self._prim = None
        self._stage = None
    
    def initialize(self, stage):
        """Initialize the cuboid in the stage"""
        self._stage = stage
        self._prim = create_dynamic_cuboid(
            stage,
            self.prim_path,
            self._position,
            self._size,
            self._color,
            self._mass,
            self.name
        )
    
    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get world position and orientation"""
        if self._prim is None:
            return self._position, np.array([1, 0, 0, 0])
        
        xformable = UsdGeom.Xformable(self._prim)
        world_transform = xformable.ComputeLocalToWorldTransform(0)
        translation = world_transform.ExtractTranslation()
        
        # Extract rotation as quaternion (simplified - identity for now)
        return np.array([translation[0], translation[1], translation[2]]), np.array([1, 0, 0, 0])
    
    def set_world_pose(
        self,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None
    ):
        """Set world position"""
        if self._prim is None or position is None:
            return
        
        xformable = UsdGeom.Xformable(self._prim)
        # Clear existing ops and set new translation
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(*position))
    
    def set_linear_velocity(self, velocity: np.ndarray):
        """Set linear velocity (requires physics step)"""
        pass  # Velocity is handled by physics engine
    
    def set_angular_velocity(self, velocity: np.ndarray):
        """Set angular velocity (requires physics step)"""
        pass  # Velocity is handled by physics engine
    
    def set_default_state(
        self,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None
    ):
        """Set default state for reset"""
        if position is not None:
            self._position = np.array(position)
            self.set_world_pose(position=position)


__all__ = [
    "create_dynamic_cuboid",
    "create_physics_material",
    "apply_material_to_prim",
    "DynamicCuboidWrapper",
]
