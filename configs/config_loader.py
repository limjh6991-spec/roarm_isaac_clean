#!/usr/bin/env python3
"""
RoArm Isaac Clean: Configuration Loader

YAML 기반 설정 관리 시스템
- 환경 변수 치환 (${PROJECT_ROOT} 등)
- 설정 파일 상속 (_extends)
- 타입 안전 Dataclass 변환
"""

import os
import yaml
from pathlib import Path
from typing import Any, Dict, Optional, TypeVar, Type
from dataclasses import dataclass, field


# 프로젝트 루트 경로
PROJECT_ROOT = Path(__file__).parent.parent.absolute()


def _substitute_env_vars(config: Dict[str, Any]) -> Dict[str, Any]:
    """
    설정 값에서 환경 변수를 치환합니다.
    
    지원 패턴:
    - ${PROJECT_ROOT}: 프로젝트 루트 경로
    - ${ENV_VAR_NAME}: 시스템 환경 변수
    """
    def _substitute(value: Any) -> Any:
        if isinstance(value, str):
            # ${PROJECT_ROOT} 치환
            value = value.replace("${PROJECT_ROOT}", str(PROJECT_ROOT))
            
            # 기타 환경 변수 치환
            import re
            pattern = r'\$\{([^}]+)\}'
            matches = re.findall(pattern, value)
            for match in matches:
                env_value = os.environ.get(match, "")
                value = value.replace(f"${{{match}}}", env_value)
            return value
        elif isinstance(value, dict):
            return {k: _substitute(v) for k, v in value.items()}
        elif isinstance(value, list):
            return [_substitute(item) for item in value]
        return value
    
    return _substitute(config)


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """
    딕셔너리를 재귀적으로 병합합니다.
    override가 base를 덮어씁니다.
    """
    result = base.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(config_path: str, config_dir: Optional[str] = None) -> Dict[str, Any]:
    """
    YAML 설정 파일을 로드합니다.
    
    Args:
        config_path: 설정 파일 경로 (절대 경로 또는 configs/ 상대 경로)
        config_dir: 설정 파일 디렉토리 (기본: PROJECT_ROOT/configs)
    
    Returns:
        병합된 설정 딕셔너리
    
    Example:
        >>> config = load_config("vision_rl.yaml")
        >>> print(config["camera"]["resolution"])
        [84, 84]
    """
    if config_dir is None:
        config_dir = PROJECT_ROOT / "configs"
    else:
        config_dir = Path(config_dir)
    
    # 절대 경로 또는 상대 경로 처리
    if os.path.isabs(config_path):
        full_path = Path(config_path)
    else:
        full_path = config_dir / config_path
    
    if not full_path.exists():
        raise FileNotFoundError(f"설정 파일을 찾을 수 없습니다: {full_path}")
    
    with open(full_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    # 상속 처리 (_extends)
    if "_extends" in config:
        base_path = config.pop("_extends")
        base_config = load_config(base_path, config_dir)
        config = _deep_merge(base_config, config)
    
    # 환경 변수 치환
    config = _substitute_env_vars(config)
    
    return config


# ═══════════════════════════════════════════════════════════════
# Typed Dataclasses for IDE Support
# ═══════════════════════════════════════════════════════════════

@dataclass
class RobotDriveConfig:
    stiffness: float = 5000.0
    damping: float = 500.0
    max_force: float = 500.0


@dataclass
class RobotConfig:
    urdf_path: str = ""
    position: tuple = (0.0, 0.0, 0.0)
    home_position: tuple = (0.0, 0.0, -1.0, 0.0, 0.0, 0.0125)
    arm_drive: RobotDriveConfig = field(default_factory=RobotDriveConfig)
    gripper_drive: RobotDriveConfig = field(default_factory=lambda: RobotDriveConfig(
        stiffness=8000.0, damping=800.0, max_force=50.0
    ))


@dataclass
class ObjectConfig:
    position: tuple = (0.3, 0.0, 0.05)
    size: tuple = (0.04, 0.04, 0.04)
    color: tuple = (0.8, 0.2, 0.2)
    mass: float = 0.1


@dataclass
class RewardConfig:
    success_reward: float = 100.0
    success_threshold: float = 0.02
    success_hold_frames: int = 10
    time_penalty: float = 0.01
    reach_bonus: float = 5.0
    attach_bonus: float = 50.0


@dataclass
class CurriculumPhaseConfig:
    cube_distance: tuple = (0.15, 0.25)
    target_distance: tuple = (0.25, 0.35)


@dataclass
class CurriculumConfig:
    enabled: bool = True
    initial_phase: int = 0
    easy: CurriculumPhaseConfig = field(default_factory=CurriculumPhaseConfig)
    normal: CurriculumPhaseConfig = field(default_factory=lambda: CurriculumPhaseConfig(
        cube_distance=(0.35, 0.50), target_distance=(0.35, 0.50)
    ))
    success_rate_window: int = 100
    success_rate_threshold: float = 0.30
    reach_milestone_threshold: int = 5


@dataclass
class CameraConfig:
    resolution: tuple = (84, 84)
    channels: int = 4
    parent_link: str = "gripper_base"
    position: tuple = (0.0, 0.0, 0.05)
    rotation: tuple = (0.0, -30.0, 0.0)


@dataclass
class EnvConfig:
    """환경 전체 설정을 담는 메인 Dataclass"""
    num_envs: int = 1
    env_spacing: float = 2.5
    episode_length_s: float = 10.0
    fps: int = 60
    
    robot: RobotConfig = field(default_factory=RobotConfig)
    cube: ObjectConfig = field(default_factory=ObjectConfig)
    target: ObjectConfig = field(default_factory=lambda: ObjectConfig(
        position=(0.0, 0.3, 0.2), color=(0.2, 0.8, 0.2)
    ))
    reward: RewardConfig = field(default_factory=RewardConfig)
    curriculum: CurriculumConfig = field(default_factory=CurriculumConfig)
    camera: Optional[CameraConfig] = None


T = TypeVar('T')


def load_typed_config(config_path: str, config_class: Type[T] = EnvConfig) -> T:
    """
    YAML 설정을 로드하고 타입 안전한 Dataclass로 변환합니다.
    
    Args:
        config_path: 설정 파일 경로
        config_class: 변환할 Dataclass 타입
    
    Returns:
        타입화된 설정 객체
    
    Example:
        >>> config = load_typed_config("base.yaml")
        >>> print(config.robot.home_position)
        (0.0, 0.0, -1.0, 0.0, 0.0, 0.0125)
    """
    raw_config = load_config(config_path)
    
    # 재귀적으로 dataclass 변환
    def _to_dataclass(data: Any, cls: Type) -> Any:
        if data is None:
            return None
        if not hasattr(cls, '__dataclass_fields__'):
            return data
        
        kwargs = {}
        for field_name, field_def in cls.__dataclass_fields__.items():
            if field_name in data:
                value = data[field_name]
                field_type = field_def.type
                
                # 중첩 dataclass 처리
                if hasattr(field_type, '__dataclass_fields__'):
                    value = _to_dataclass(value, field_type)
                elif isinstance(value, list):
                    value = tuple(value)  # list → tuple
                
                kwargs[field_name] = value
        
        return cls(**kwargs)
    
    # EnvConfig 구조로 변환
    return _to_dataclass(raw_config, config_class)


# ═══════════════════════════════════════════════════════════════
# Convenience Functions
# ═══════════════════════════════════════════════════════════════

def get_config(name: str = "base") -> Dict[str, Any]:
    """
    설정을 이름으로 로드합니다.
    
    Args:
        name: 설정 이름 (base, vision_rl 등)
    
    Example:
        >>> cfg = get_config("vision_rl")
    """
    return load_config(f"{name}.yaml")


if __name__ == "__main__":
    # 테스트
    print("=" * 60)
    print("🔧 Config Loader Test")
    print("=" * 60)
    
    # 기본 설정 로드
    base_cfg = load_config("base.yaml")
    print(f"\n📄 base.yaml:")
    print(f"  - Robot URDF: {base_cfg['robot']['urdf_path']}")
    print(f"  - Home Position: {base_cfg['robot']['home_position']}")
    print(f"  - Curriculum: {base_cfg['curriculum']['enabled']}")
    
    # Vision RL 설정 로드 (상속 테스트)
    try:
        vision_cfg = load_config("vision_rl.yaml")
        print(f"\n📄 vision_rl.yaml (extends base.yaml):")
        print(f"  - Camera Resolution: {vision_cfg['camera']['resolution']}")
        print(f"  - Algorithm: {vision_cfg['algorithm']['name']}")
        print(f"  - Episode Length: {vision_cfg['env']['episode_length_s']}s (overridden)")
    except FileNotFoundError as e:
        print(f"\n⚠️ {e}")
    
    print("\n✅ Config Loader 정상 동작!")
