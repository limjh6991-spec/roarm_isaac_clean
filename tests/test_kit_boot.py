"""
Test: Isaac Sim Headless Boot
최소 부팅 테스트 - SimulationApp이 정상적으로 초기화되는지 확인
"""

import pytest


def test_headless_boot():
    """
    SimulationApp을 headless 모드로 부팅하고 정상 종료
    """
    from isaacsim import SimulationApp
    
    # Headless 모드로 부팅
    simulation_app = SimulationApp({
        "headless": True,
        "renderer": "RayTracedLighting"
    })
    
    # 정상 부팅 확인
    assert simulation_app is not None
    
    # 정상 종료
    simulation_app.close()


def test_simulation_app_with_extensions():
    """
    SimulationApp 부팅 후 필수 확장이 로드되었는지 확인
    """
    from isaacsim import SimulationApp
    
    simulation_app = SimulationApp({"headless": True})
    
    # 필수 확장 확인
    import omni.kit.app
    app_interface = omni.kit.app.get_app()
    ext_manager = app_interface.get_extension_manager()
    
    required_extensions = [
        "isaacsim.core.api",
        "omni.isaac.core",
        "omni.physx",
    ]
    
    for ext_name in required_extensions:
        assert ext_manager.is_extension_enabled(ext_name), \
            f"Required extension not enabled: {ext_name}"
    
    simulation_app.close()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
