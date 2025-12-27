"""
Scene Module

Isaac Sim 5.1 scene building utilities.
"""

from envs.scene.scene_builder import (
    create_dynamic_cuboid,
    create_physics_material,
    apply_material_to_prim,
    DynamicCuboidWrapper,
)

__all__ = [
    "create_dynamic_cuboid",
    "create_physics_material", 
    "apply_material_to_prim",
    "DynamicCuboidWrapper",
]
