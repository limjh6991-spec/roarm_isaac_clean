#!/usr/bin/env python3
"""
RoArm-M3 Pick and Place 강화학습 환경
Isaac Sim 5.0 + omni.isaac.lab 기반
"""

import numpy as np
import torch
from typing import Dict, Tuple
import sys
import os
import gymnasium as gym

# 🔥 v3.5: 프로젝트 루트를 Python path에 추가 (모듈 import용)
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# 🔍 USD/pxr 로딩 경로 진단
print("=" * 80)
print("🔍 USD/pxr 모듈 로딩 진단")
print("=" * 80)

try:
    from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics, UsdShade
    pxr_path = sys.modules['pxr'].__file__
    print(f"✅ pxr 로딩 성공")
    print(f"   경로: {pxr_path}")
    print(f"   버전: {Usd.GetVersion()}")
    
    # USD 스키마 무결성 점검
    print("\n📋 USD 스키마 무결성 점검:")
    schema_checks = {
        "UsdGeom.Xform": hasattr(UsdGeom, 'Xform'),
        "UsdGeom.Mesh": hasattr(UsdGeom, 'Mesh'),
        "UsdGeom.Cube": hasattr(UsdGeom, 'Cube'),
        "UsdGeom.Sphere": hasattr(UsdGeom, 'Sphere'),
        "Gf.Vec3f": hasattr(Gf, 'Vec3f'),
        "Gf.Matrix4d": hasattr(Gf, 'Matrix4d'),
        "Sdf.Path": hasattr(Sdf, 'Path'),
    }
    
    all_ok = True
    for schema_name, exists in schema_checks.items():
        status = "✅" if exists else "❌"
        print(f"   {status} {schema_name}")
        if not exists:
            all_ok = False
    
    if all_ok:
        print("✅ 모든 USD 스키마 정상")
    else:
        print("⚠️ 일부 USD 스키마 누락됨")
        
except ImportError as e:
    print(f"❌ pxr 로딩 실패: {e}")
    print(f"   Python 경로: {sys.executable}")
    print(f"   sys.path: {sys.path[:3]}...")
    raise

print("=" * 80)
print()

# Isaac Sim imports
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction  # 🔥 v3.7.6: 타깃 기반 제어용

# Isaac Lab imports (Isaac Sim 5.0)
try:
    from omni.isaac.lab.envs import DirectRLEnv, DirectRLEnvCfg
    from omni.isaac.lab.utils import configclass
except ImportError:
    # Fallback for older API
    print("⚠️ Isaac Lab API not found. Using basic implementation.")
    DirectRLEnv = object
    DirectRLEnvCfg = object
    configclass = lambda x: x

# 🔥 v3.5: 모듈화된 컴포넌트 임포트
from controllers.gripper import Gripper
from robot_utils.ee_pose import find_ee_prim, get_ee_position
from rewards.pick_place import GateConfig, grasp_gate, compute_hybrid_reward

# 🔥 v3.9.6: 리팩토링 - 기능별 모듈 분리
from envs.reward.reward_calculator import RewardCalculator
from envs.observation.observation_builder import ObservationBuilder
from envs.robot.robot_controller import RobotController


@configclass
class RoArmPickPlaceEnvCfg(DirectRLEnvCfg):
    """RoArm-M3 Pick and Place 환경 설정"""
    
    # Environment settings
    num_envs: int = 1
    env_spacing: float = 2.5
    episode_length_s: float = 10.0
    
    # Robot settings
    # 🤖 V4.2: STL 메시 기반 정확한 URDF (sim-to-real gap 최소화)
    # codex_mcp에서 생성된 정확한 관성 텐서 포함
    urdf_path: str = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3.generated.urdf"
    robot_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    # Object settings
    object_position: Tuple[float, float, float] = (0.3, 0.0, 0.05)
    object_size: Tuple[float, float, float] = (0.04, 0.04, 0.04)
    target_position: Tuple[float, float, float] = (0.0, 0.3, 0.2)
    
    # Reward settings (Shaped-Sparse)
    reach_reward_scale: float = 1.0      # (Deprecated)
    grasp_reward: float = 5.0            # (Deprecated)
    lift_reward_scale: float = 2.0       # (Deprecated)
    move_reward_scale: float = 2.0       # (Deprecated)
    success_reward: float = 100.0        # Success bonus
    success_threshold: float = 0.02      # 2cm (5cm → 2cm, 더 정밀한 제어!)
    success_hold_frames: int = 10        # 10프레임 연속 유지 (5 → 10)
    time_penalty: float = 0.01           # Efficiency penalty
    
    # ═══════════════════════════════════════════════════════════
    # 📚 CURRICULUM LEARNING 설정
    # ═══════════════════════════════════════════════════════════
    curriculum_enabled: bool = True      # Curriculum 활성화
    curriculum_phase: int = 0            # 현재 Phase (0: Easy, 1: Normal)
    
    # 🔥 v3.7.9 FIX: Phase 0 거리 조정 (15-25cm / 25-35cm)
    # 이전: 10-15cm는 너무 가까워서 로봇팔 움직임 제한적
    # 해결: 15-25cm로 늘려서 충분한 작업 공간 확보
    easy_cube_distance: Tuple[float, float] = (0.15, 0.25)  # 15~25cm (작업 공간 확보)
    easy_target_distance: Tuple[float, float] = (0.25, 0.35)  # 25~35cm
    
    # Phase 1: Normal Mode (먼 거리, 35~50cm → 더 어렵게!)
    normal_cube_distance: Tuple[float, float] = (0.35, 0.50)  # 35~50cm (25-35cm → 35-50cm)
    normal_target_distance: Tuple[float, float] = (0.35, 0.50)  # 35~50cm (25-35cm → 35-50cm)
    
    # ═══════════════════════════════════════════════════════════
    # 📚 자동 승급 조건 (완화됨!)
    # ═══════════════════════════════════════════════════════════
    success_rate_window: int = 100       # 최근 100 에피소드 (200 → 100)
    success_rate_threshold: float = 0.30  # 성공률 30% 이상 (60% → 30%)
    reach_milestone_threshold: int = 5    # REACH 5회 달성 (10회 → 5회)


class RoArmPickPlaceEnv(gym.Env):
    """
    RoArm-M3 Pick and Place 강화학습 환경 (Dense Reward)
    
    🤖 V4.2: STL 메시 URDF + 6 DOF (그리퍼는 revolute joint로 통합)
    
    Task: 큐브를 집어서 타겟 위치로 옮기기
    
    Observation Space (24 dim):
        - Joint positions (6 dim): joint_1 ~ joint_6 (arm + gripper)
        - End-effector position (3 dim): x, y, z
        - Cube position (3 dim): x, y, z
        - Target position (3 dim): x, y, z
        - EE → Cube vector (3 dim): dx, dy, dz
        - Cube → Target vector (3 dim): dx, dy, dz
        - Gripper state (3 dim): joint_6 position, velocity, torque
        - Gripper width (1 dim): distance between fingers
        - Is grasped (1 dim): 1.0 if grasping, 0.0 otherwise
    
    Action Space (8 dim):
        - Joint position deltas (6 dim): joint_1 ~ joint_6
        - Gripper position deltas (2 dim): left_finger, right_finger
    """
    
    metadata = {'render.modes': ['human']}
    def __init__(self, cfg: RoArmPickPlaceEnvCfg = None):
        """환경 초기화"""
        self.cfg = cfg if cfg else RoArmPickPlaceEnvCfg()
        
        # Isaac Sim World 초기화
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        
        print("=" * 60)
        print("🤖 RoArm-M3 Pick and Place Environment")
        print("🔥 ENV_VERSION = v4.2 (6 DOF STL 메시 URDF)")
        print("   - REACH: 지속적 보상 +2.0 (유지) + 5.0 (최초)")
        print("   - 그리퍼 전략: ACTION 기반 (action[5] < -0.3)")
        print("     - 닫기 시도: +0.5")
        print("     - position < 0.01: +2.0 (총 +2.5)")
        print("   - ATTACH: +50.0 (부착 성공)")
        print("   - 멀어짐 패널티: -2.0")
        print("=" * 60)
        
        # 로봇 로드
        self._load_robot()
        
        # 물체 생성
        self._create_objects()
        
        # 환경 변수 초기화
        self.current_step = 0
        self.step_count = 0  # 🔥 v3.7.1: 디버그 로깅용 전역 스텝 카운터
        self.max_steps = int(self.cfg.episode_length_s * 60)  # 60 FPS 가정
        
        # ═══════════════════════════════════════════════════════════
        # 📊 Observation/Action Space (개선: EE 기준 상대 좌표)
        # ═══════════════════════════════════════════════════════════
        # Observation: 28 dim (EE 기준 상대 좌표!)
        #   - Joint positions (8)
        #   - Cube pos relative to EE (3) ← 핵심!
        #   - Target pos relative to EE (3) ← 핵심!
        #   - Cube to Target vector (3)
        # 🤖 V4.2: 6 DOF (STL 메시 URDF)
        # Observation: 24 dim
        #   - Joint positions (6): joint_1 ~ joint_6 (마지막이 gripper)
        #   - EE position (3)
        #   - Cube position (3)
        #   - Target position (3)
        #   - EE → Cube vector (3)
        #   - Cube → Target vector (3)
        #   - Gripper state (3): position, velocity, torque
        self.observation_space_dim = 24
        # Action: 6 DOF 
        # 구조: [5 DoF arm joints] + [1 gripper joint]
        self.action_dim = 6
        self.action_space_dim = 7
        
        # Gymnasium spaces
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(24,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(6,), dtype=np.float32
        )
        
        # 이전 상태 저장 (속도 계산 & Dense Reward)
        self.prev_ee_pos = None
        self.prev_cube_pos = None
        self.prev_ee_to_cube_dist = None
        self.prev_cube_to_target_dist = None
        self.previous_reward = 0.0
        
        # 🔥 v3.7.2: 그리퍼 폭 직접 추적 (physics lag 회피)
        self.current_gripper_width = 0.0
        
        # 🔥 v3.7.8: 그리퍼 증분 제어 상태 (URDF limit 준수: 0~0.025)
        self.current_gripper_pos = 0.0125  # 중간값에서 시작 (0.0 ~ 0.025 범위)
        
        # ═══════════════════════════════════════════════════════════
        # 🎯 v3.9.0: 단계별 메트릭 추적 (False Positive 방지)
        # ═══════════════════════════════════════════════════════════
        self.first_reach = False      # EE가 큐브에 처음 근접 (5cm)
        self.first_attach = False     # 🔥 v3.9.0: 물리적 부착 달성
        self.first_lift = False       # 🔥 v3.9.0: 리프트 달성 (attach 후)
        self.success = False          # 🔥 v3.9.0: 최종 성공 (attach & lift & hold)
        
        # 🔥 v3.9.0: 프레임 카운터
        self.attach_hold_frames = 0   # Attach 유지 프레임
        self.lift_hold_frames = 0     # Lift 유지 프레임
        self.success_frames = 0       # Success 유지 프레임
        
        # 🔥 v3.9.0: 에피소드 메트릭 (분석용)
        self.episode_metrics = {
            'reach': False,           # dist < 5cm
            'attach': False,          # is_attached
            'lift': False,            # Δz > 2cm
            'hold': False,            # N 프레임 유지
            'success': False          # attach & lift & hold
        }
        
        # 마일스톤 카운터 (에피소드 통계용)
        self.episode_reach_count = 0
        self.episode_attach_count = 0  # 🔥 v3.9.0: attach 카운트
        self.episode_lift_count = 0
        self.episode_success_count = 0 # 🔥 v3.9.0: success 카운트
        
        # 🔥 v3.9.0: 초기 큐브 위치 저장 (리프트 계산용)
        self.initial_cube_z = None
        
        # ═══════════════════════════════════════════════════════════
        # 📚 CURRICULUM: 성공률 추적
        # ═══════════════════════════════════════════════════════════
        self.episode_successes = []   # 최근 에피소드 성공 여부
        
        # ═══════════════════════════════════════════════════════════
        # 🔍 v3.9.6: 최소 로깅 시스템 (성능 최적화)
        # ═══════════════════════════════════════════════════════════
        # 매 스텝 로깅 제거 → 누적 통계만 (리스트 제거!)
        self.episode_min_distance = float('inf')
        self.episode_distance_sum = 0.0
        self.episode_distance_count = 0
        
        # 단계별 보상 트리거 (카운터만, 리스트 제거)
        self.stage_7cm_count = 0
        self.stage_5cm_count = 0
        self.stage_3cm_count = 0
        
        # 그리퍼 통계 (평균만, 리스트 제거)
        self.gripper_reward_count = 0
        self.gripper_reward_total = 0.0
        
        # 보상 분해 (에피소드별 누적)
        self.episode_reward_breakdown = {
            'distance_reward': 0.0,
            'stage_bonus': 0.0,
            'gripper_bonus': 0.0,
            'synergy_bonus': 0.0,
            'milestone_bonus': 0.0,
        }
        
        # 🔥 v3.5: 모듈화된 컴포넌트 초기화 (로봇 로드 전 설정)
        self.gripper = None           # Gripper controller
        self.ee_prim_path = None      # EE prim 경로
        self.gate_config = GateConfig(cube_size=self.cfg.object_size[0])
        
        print(f"\n📊 환경 정보:")
        print(f"  - Observation dim: {self.observation_space_dim}")
        print(f"  - Action dim: {self.action_space_dim}")
        print(f"  - Episode length: {self.cfg.episode_length_s}s * 60 FPS = {self.max_steps} steps")
        print(f"  - Success threshold: {self.cfg.success_threshold}m ({int(self.cfg.success_threshold*100)}cm)")
        print(f"  🔥 v3.5: 모듈화 활성화 (Gripper/EE/Rewards)")
        print(f"  - Success hold frames: {self.cfg.success_hold_frames} (연속 유지)")
        print(f"  - Reward type: Shaped-Sparse (게이팅 + 1회성 이벤트)")
        print(f"  - Curriculum: Phase {self.cfg.curriculum_phase} ({'Easy' if self.cfg.curriculum_phase == 0 else 'Normal'})")
        if self.cfg.curriculum_phase == 0:
            print(f"    • Cube: {self.cfg.easy_cube_distance[0]*100:.0f}-{self.cfg.easy_cube_distance[1]*100:.0f}cm")
            print(f"    • Target: {self.cfg.easy_target_distance[0]*100:.0f}-{self.cfg.easy_target_distance[1]*100:.0f}cm")
        else:
            print(f"    • Cube: {self.cfg.normal_cube_distance[0]*100:.0f}-{self.cfg.normal_cube_distance[1]*100:.0f}cm")
            print(f"    • Target: {self.cfg.normal_target_distance[0]*100:.0f}-{self.cfg.normal_target_distance[1]*100:.0f}cm")
    
    def _load_robot(self):
        """로봇 URDF 로드"""
        print(f"\n🔧 로봇 로딩: {self.cfg.urdf_path}")
        
        # URDF 임포트 (Isaac Sim 5.0 방식)
        import omni.kit.commands
        from isaacsim.asset.importer.urdf import _urdf  # ✅ Isaac Sim 5.0 모듈!
        
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.fix_base = True
        import_config.import_inertia_tensor = True
        import_config.self_collision = False
        import_config.distance_scale = 1.0
        
        # URDF 임포트 (Isaac Sim 5.0 공식 방식)
        # ✅ URDFParseAndImportFile는 stage에 직접 import하며 prim_path를 반환
        print("  ⏳ URDF 파싱 중...")
        
        success, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=self.cfg.urdf_path,
            import_config=import_config,
            get_articulation_root=True,  # ✅ 공식 예제 패턴!
        )
        
        if not success:
            raise RuntimeError(f"❌ URDF 임포트 실패: {self.cfg.urdf_path}")
        
        print(f"  ✅ 로봇 임포트 성공!")
        print(f"  � Prim path: {prim_path}")
        
        # Articulation 생성 (반환된 prim_path 사용)
        self.robot = self.world.scene.add(
            Articulation(prim_path=prim_path, name="roarm_m3")
        )
        
        # World reset으로 articulation 초기화
        print(f"  ⏳ Articulation 초기화 중...")
        self.world.reset()
        
        # Joint 이름 가져오기
        self.joint_names = self.robot.dof_names
        print(f"  ✅ Joints ({len(self.joint_names) if self.joint_names else 0}): {self.joint_names[:3] if self.joint_names and len(self.joint_names) > 3 else self.joint_names}...")
        
        # 🔥 v3.5 FIX #3: Prismatic joint에 linear drive 적용
        print(f"  ⏳ Joint drive 설정 중...")
        stage = self.world.stage
        
        # 모든 joint에 대해 drive 설정
        for i, joint_name in enumerate(self.joint_names):
            joint_prim = stage.GetPrimAtPath(f"{prim_path}/{joint_name}")
            if joint_prim and joint_prim.IsValid():
                # 🔥 v3.5: Prismatic joint 구분
                is_gripper = ("gripper" in joint_name)
                
                if is_gripper:
                    # 🔥 FIX #3: Prismatic joint에는 LINEAR drive!
                    drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
                    drive_api.GetStiffnessAttr().Set(8000.0)   # 강한 힘으로 죄기
                    drive_api.GetDampingAttr().Set(800.0)      # 안정적인 제어
                    drive_api.GetMaxForceAttr().Set(50.0)      # 🔥 충분한 힘 (8→50N)
                    print(f"    ✅ {joint_name}: LINEAR drive (stiff=8000, damp=800, force=50N)")
                elif i < 6:  # 팔 joint
                    drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
                    drive_api.GetStiffnessAttr().Set(5000.0)  # 완화
                    drive_api.GetDampingAttr().Set(500.0)     # 완화
                    drive_api.GetMaxForceAttr().Set(500.0)    # 완화
                else:  # 그리퍼
                    drive_api.GetStiffnessAttr().Set(1000.0)
                    drive_api.GetDampingAttr().Set(100.0)
                    drive_api.GetMaxForceAttr().Set(100.0)
        
        print(f"  ✅ Joint drive 설정 완료! (완화된 값)")
        
        # 🔥 v3.5: Gripper controller 초기화
        print(f"  ⏳ Gripper controller 초기화 중...")
        self.gripper = Gripper(
            stage=stage,
            robot_prim_path=prim_path,
            finger_joint_names=["gripper_left_joint", "gripper_right_joint"]
        )
        # Drive는 이미 위에서 설정됨 (수동으로)
        
        # 🔥 v3.5: EE prim 탐색
        print(f"  ⏳ End-Effector prim 탐색 중...")
        self.ee_prim_path = find_ee_prim(stage, prim_path)
        if self.ee_prim_path:
            print(f"  ✅ EE prim: {self.ee_prim_path}")
        
        # 🔥 v3.7.6 FIX: Articulation controller 가져오기 (타깃 기반 제어용)
        print(f"  ⏳ Articulation controller 초기화 중...")
        self.controller = self.robot.get_articulation_controller()
        print(f"  ✅ Controller 준비 완료! (PD 타깃 기반 제어)")
    
    def _create_objects(self):
        """물체 및 타겟 생성"""
        print(f"\n📦 물체 생성 중...")
        
        # 큐브 생성 (Pick 대상)
        self.cube = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/cube",
                name="cube",
                position=np.array(self.cfg.object_position),
                size=self.cfg.object_size[0],  # 정육면체
                color=np.array([0.8, 0.2, 0.2]),  # 빨간색
            )
        )
        print(f"  ✅ 큐브 생성: {self.cfg.object_position}")
        
        # 🔥 v3.5 FIX #4: 큐브에 고마찰 Physics Material 적용
        stage = self.world.stage
        cube_prim = stage.GetPrimAtPath("/World/cube")
        
        if cube_prim and cube_prim.IsValid():
            # Physics Material 생성 (이미 있으면 재사용)
            mat_path = "/World/PhysicsMaterials/HighFriction"
            mat_prim = stage.GetPrimAtPath(mat_path)
            
            if not mat_prim or not mat_prim.IsValid():
                # 새로 생성
                mat_prim = stage.DefinePrim(mat_path, "Material")
                
                # ✅ 기본 마찰/반발 속성: UsdPhysics.MaterialAPI 사용
                material_api = UsdPhysics.MaterialAPI.Apply(mat_prim)
                material_api.CreateStaticFrictionAttr(1.2)   # Static friction
                material_api.CreateDynamicFrictionAttr(1.0)  # Dynamic friction
                material_api.CreateRestitutionAttr(0.1)      # Restitution (낮은 반발)
                
                print(f"    ✅ 고마찰 Material 생성: friction=1.2/1.0")
            
            # 큐브에 Material 바인딩 (UsdShade.MaterialBindingAPI 사용)
            binding_api = UsdShade.MaterialBindingAPI.Apply(cube_prim)
            binding_api.Bind(UsdShade.Material(mat_prim))
            print(f"    ✅ 큐브에 고마찰 Material 적용")
        
        # 타겟 마커 생성 (시각적 목표)
        self.target = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/target",
                name="target",
                position=np.array(self.cfg.target_position),
                size=self.cfg.object_size[0],
                color=np.array([0.2, 0.8, 0.2]),  # 초록색
            )
        )
        # 타겟은 정적 (kinematic)으로 설정
        self.target.set_default_state(position=np.array(self.cfg.target_position))
        
        print(f"  ✅ 타겟 생성: {self.cfg.target_position}")
        
        # ═══════════════════════════════════════════════════════════
        # 🔥 v3.9.6: 리팩토링 - 컴포넌트 초기화
        # ═══════════════════════════════════════════════════════════
        print(f"\n🔧 컴포넌트 초기화 중...")
        
        # 1. 보상 계산기
        self.reward_calc = RewardCalculator()
        print(f"  ✅ RewardCalculator 초기화")
        
        # 2. 관측 생성기
        ee_prim = self.world.stage.GetPrimAtPath(self.ee_prim_path)
        self.obs_builder = ObservationBuilder(
            robot=self.robot,
            gripper=self.gripper,
            cube=self.cube,
            target=self.target,
            ee_prim=ee_prim
        )
        print(f"  ✅ ObservationBuilder 초기화")
        
        # 3. 로봇 제어기
        self.robot_controller = RobotController(
            robot=self.robot,
            gripper=self.gripper
        )
        print(f"  ✅ RobotController 초기화")
        print(f"✅ v3.9.6 리팩토링 완료!\n")
    
    def reset(self, seed=None, options=None) -> np.ndarray:
        """환경 리셋 (Gymnasium API 호환: seed/options 파라미터 추가)"""
        if seed is not None:
            np.random.seed(seed)
        
        print(f"\n🔄 환경 리셋 (Step {self.current_step})")
        
        # World 리셋
        self.world.reset()
        
        # 🤖 V4.2: 6 DOF home positions (STL 메시 URDF)
        # RoArm-M3 실제 초기 자세: 'ㄱ'자 모양
        # ⚠️ Joint Limits 고려: Joint 2는 -1.0 ~ 2.95 범위
        # Joint 0 (Base): 0.0 (정면)
        # Joint 1 (Link1): 0.0 (지면과 90도 수직)
        # Joint 2 (Link2): -1.0 (URDF limit! -1.57은 불가능)
        # Joint 3 (Wrist): 0.0
        # Joint 4 (Wrist2): 0.0
        # Joint 5 (Gripper): 0.0125 (약간 열림)
        home_positions = np.array([0.0, 0.0, -1.0, 0.0, 0.0, 0.0125])  # 6 DOF
        print(f"  🤖 V4.2: 'ㄱ'자 Home Position (Joint2=-1.0, URDF limit 준수)")
        print(f"  🔧 설정값: {home_positions.tolist()}")
        self.robot.set_joint_positions(home_positions)
        self.robot.set_joint_velocities(np.zeros(6))  # 6 DOF
        
        # Physics 안정화 시간 (60 steps = 1초)
        for _ in range(60):
            self.world.step(render=False)
        
        # 초기 큐브 Z 위치 저장 (리프트 계산용)
        cube_pos = self.cube.get_world_pose()[0]
        self.initial_cube_z = cube_pos[2]
        print(f"  📦 초기 큐브 Z: {self.initial_cube_z:.3f}m")
        
        # 안정화 후 실제 적용된 관절 값 확인
        actual_positions = self.robot.get_joint_positions()
        print(f"  ✅ 안정화 후 실제 관절 값: {actual_positions[:6].tolist()}")
        
        # ═══════════════════════════════════════════════════════════
        # 📚 CURRICULUM: Phase에 따른 큐브/타겟 위치 설정
        # ═══════════════════════════════════════════════════════════
        if self.cfg.curriculum_enabled:
            if self.cfg.curriculum_phase == 0:  # Easy Mode
                # 큐브를 로봇 가까이 (10~15cm)
                distance = np.random.uniform(*self.cfg.easy_cube_distance)
                angle = np.random.uniform(0, 2 * np.pi)
                cube_pos = np.array([
                    distance * np.cos(angle),
                    distance * np.sin(angle),
                    0.025  # 🔧 FIX: 바닥에 안착 (큐브 높이의 절반 = 0.05/2)
                ])
                
                # 타겟도 가까이 (20~25cm)
                target_distance = np.random.uniform(*self.cfg.easy_target_distance)
                target_angle = np.random.uniform(0, 2 * np.pi)
                target_pos = np.array([
                    target_distance * np.cos(target_angle),
                    target_distance * np.sin(target_angle),
                    0.2  # 타겟 높이
                ])
                
                print(f"  📚 Phase 0 (Easy): cube={distance:.2f}m, target={target_distance:.2f}m")
            else:  # Normal Mode (Phase 1+)
                # 원래 거리 (25~35cm)
                distance = np.random.uniform(*self.cfg.normal_cube_distance)
                angle = np.random.uniform(0, 2 * np.pi)
                cube_pos = np.array([
                    distance * np.cos(angle),
                    distance * np.sin(angle),
                    0.025  # 🔧 FIX: 바닥에 안착
                ])
                
                target_distance = np.random.uniform(*self.cfg.normal_target_distance)
                target_angle = np.random.uniform(0, 2 * np.pi)
                target_pos = np.array([
                    target_distance * np.cos(target_angle),
                    target_distance * np.sin(target_angle),
                    0.2
                ])
                
                print(f"  📚 Phase {self.cfg.curriculum_phase} (Normal): cube={distance:.2f}m, target={target_distance:.2f}m")
        else:
            # Curriculum 비활성화: 기본 위치 + 랜덤
            cube_pos = np.array(self.cfg.object_position)
            cube_pos[:2] += np.random.uniform(-0.05, 0.05, size=2)
            target_pos = np.array(self.cfg.target_position)
        
        self.cube.set_world_pose(position=cube_pos)
        self.cube.set_linear_velocity(np.zeros(3))
        self.cube.set_angular_velocity(np.zeros(3))
        
        # 🔧 FIX: 큐브가 바닥에 안착할 시간 제공 (Physics 안정화)
        for _ in range(30):  # 30 프레임 동안 큐브가 떨어지고 안정화
            self.world.step(render=False)
        
        # 타겟 위치 업데이트 (Curriculum 적용)
        if self.cfg.curriculum_enabled:
            self.target.set_world_pose(position=target_pos)
        
        # 🔧 v3.3: 초기 상태 안정화를 위한 1초 대기 (60 프레임)
        # 로봇이 초기 자세에서 안정화되고, 큐브/타겟 위치 확인 가능
        for _ in range(60):
            self.world.step(render=False)
        
        # 스텝 카운터 초기화
        self.current_step = 0
        
        # 🔥 v3.7.2: 그리퍼 폭 추적 초기화
        self.current_gripper_width = 0.0
        
        # 🔥 v3.7.7: 그리퍼 증분 제어 위치 초기화
        self.current_gripper_pos = 0.02  # 중간값에서 시작 (0.0 ~ 0.04 범위)
        
        # 이전 거리 초기화 (보상 계산용)
        self.prev_ee_to_cube_dist = None
        self.prev_cube_to_target_dist = None
        
        # ═══════════════════════════════════════════════════════════
        # 🎯 v4.1.0: 마일스톤 플래그 리셋
        # ═══════════════════════════════════════════════════════════
        self.first_reach = False
        self.first_attach = False
        self.first_lift = False
        self.success = False
        
        # 🆕 V4.1: 그리퍼 전략 추적 (ACTION 기반)
        self._gripper_close_success = False
        
        self.attach_hold_frames = 0
        self.lift_hold_frames = 0
        self.success_frames = 0
        
        # 🔥 v3.9.0: 에피소드 메트릭 리셋
        self.episode_metrics = {
            'reach': False,
            'attach': False,
            'lift': False,
            'hold': False,
            'success': False
        }
        
        # 🔥 v3.5: Gripper attach 상태 리셋
        if self.gripper:
            self.gripper.reset()
        
        # 🔥 v3.9.0: 마일스톤 카운터 리셋
        self.episode_reach_count = 0
        self.episode_attach_count = 0
        self.episode_lift_count = 0
        self.episode_success_count = 0
        
        # ═══════════════════════════════════════════════════════════
        # 🔍 v3.9.6: 보상 계산기 리셋 (리팩토링)
        # ═══════════════════════════════════════════════════════════
        self.reward_calc.reset()
        
        # 🔥 v3.5: FixedJoint 상태 리셋 (이전 에피소드 attach 제거)
        if self.gripper and self.gripper.is_attached:
            try:
                self.gripper.detach(self.world.stage)
            except Exception as e:
                print(f"    ⚠️ Reset 시 FixedJoint 정리 실패: {e}")
        
        # 초기 observation 반환
        # Gymnasium API: (observation, info) 반환
        return self._get_observation(), {}
    
    def _get_observation(self) -> np.ndarray:
        """
        현재 상태 관측 (개선: EE 기준 상대 좌표!)
        
        핵심 개선:
        1. 월드 좌표 → EE 기준 상대 좌표 변환
        2. 속도 정보 추가 (EE, Cube)
        3. 디버깅 정보 추가
        """
        # ═══════════════════════════════════════════════════════════
        # 1. 기본 정보 수집 (월드 좌표)
        # ═══════════════════════════════════════════════════════════
        joint_positions = self.robot.get_joint_positions()[:6]  # 🤖 V4.2: 6 DOF
        ee_pos = self._get_ee_position()  # 월드 좌표
        cube_pos, _ = self.cube.get_world_pose()  # 월드 좌표
        target_pos, _ = self.target.get_world_pose()  # 실제 타깃 위치
        
        # ═══════════════════════════════════════════════════════════
        # 2. EE 기준 상대 좌표 변환
        # ═══════════════════════════════════════════════════════════
        cube_relative_to_ee = cube_pos - ee_pos       # EE → Cube 벡터
        target_relative_to_ee = target_pos - ee_pos   # EE → Target 벡터
        cube_to_target = target_pos - cube_pos        # Cube → Target 벡터
        
        # ═══════════════════════════════════════════════════════════
        # 3. Gripper state (joint 6)
        # ═══════════════════════════════════════════════════════════
        gripper_pos = joint_positions[5]  # Joint 6 (gripper)
        gripper_vel = self.robot.get_joint_velocities()[5]
        
        # Get gripper torque from articulation
        try:
            gripper_torque = self.robot.get_applied_joint_efforts()[5]
        except:
            gripper_torque = 0.0
        
        # ═══════════════════════════════════════════════════════════
        # 4. Grasp detection (simplified for 6 DOF)
        # ═══════════════════════════════════════════════════════════
        ee_to_cube_dist = np.linalg.norm(cube_relative_to_ee)
        is_grasped = 1.0 if ee_to_cube_dist < 0.05 and abs(gripper_pos) > 0.3 else 0.0
        
        dist_to_cube = ee_to_cube_dist
        dist_cube_to_target = np.linalg.norm(cube_to_target)
        
        # ═══════════════════════════════════════════════════════════
        # 5. Observation 벡터 구성 (24 dim)
        # ═══════════════════════════════════════════════════════════
        obs = np.concatenate([
            joint_positions,              # Joint positions (6)
            cube_relative_to_ee,          # Cube relative to EE (3)
            target_relative_to_ee,        # Target relative to EE (3)
            cube_to_target,               # Cube to Target (3)
            ee_pos,                       # EE position (3) - for debugging
            cube_pos,                     # Cube position (3) - for debugging
            [gripper_pos, gripper_vel, gripper_torque],  # Gripper state (3)
        ])
        
        # ═══════════════════════════════════════════════════════════
        # 7. 디버깅 (첫 스텝에만)
        # ═══════════════════════════════════════════════════════════
        if self.current_step == 0:
            print(f"\n🔍 관측 신호 점검:")
            print(f"  - Observation dim: {len(obs)} (expected: {self.observation_space_dim})")
            print(f"  - EE pos (world): {ee_pos}")
            print(f"  - Cube pos (world): {cube_pos}")
            print(f"  - Cube relative to EE: {cube_relative_to_ee}")
            print(f"  - Distance to cube: {dist_to_cube:.3f}m")
            print(f"  - Is grasped: {is_grasped}")
        
        return obs
    
    def _get_ee_position(self) -> np.ndarray:
        """
        🔥 v3.5: 모듈화된 EE 포즈 추출
        
        utils/ee_pose.py 사용
        """
        if self.ee_prim_path:
            ee_pos, _ = get_ee_position(self.world.stage, self.ee_prim_path)
            
            # 성공 체크
            if np.linalg.norm(ee_pos) > 0.01:  # 영벡터가 아니면 성공
                return ee_pos
        
        # Fallback: FK 방식
        from robot_utils.ee_pose import get_ee_position_fallback
        joint_positions = self.robot.get_joint_positions()
        return get_ee_position_fallback(joint_positions)
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        🤖 V4.2: 6 DOF 직접 제어 (STL 메시 URDF)
        
        Action Space (6 dim):
            - action[0:5]: 팔 조인트 (joint_1 ~ joint_5)
            - action[5]: 그리퍼 조인트 (joint_6, link5_to_gripper_link)
        
        Changes from V4.1.1:
            - 8 DOF (7 arm + 1 scalar) → 6 DOF (5 arm + 1 gripper)
        
        Returns:
            Gymnasium API 호환: (observation, reward, terminated, truncated, info)
            - gripper_scalar 제거 → action[5]로 직접 제어
            - current_gripper_width 추적 제거
        """
        
        action = np.clip(action, -1.0, 1.0)
        
        # 현재 joint positions 가져오기 (6 DOF)
        current_positions = self.robot.get_joint_positions()[:6]
        
        # Action scaling: 팔은 작게, 그리퍼는 크게
        action_scale = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.1])
        target_positions = current_positions + action * action_scale
        
        # Joint limits 적용 (그리퍼만 명시적으로 제한)
        target_positions[5] = np.clip(target_positions[5], 0.0, 0.025)
        
        # PD 제어로 action 적용
        self.controller.apply_action(
            ArticulationAction(joint_positions=target_positions)
        )
        
        # ═══════════════════════════════════════════════════════════
        # 현재 상태 가져오기
        if self.gripper is not None:  # Gripper 초기화 확인
            joint_positions = self.robot.get_joint_positions()
            ee_pos = self._get_ee_position()
            cube_pos = self.cube.get_world_pose()[0]
            gripper_width = self.gripper.measure_width(joint_positions)
            
            # Grasp 조건 체크 (v3.9.3: 그리퍼 폭 조건 대폭 완화)
            is_grasping = self.gripper.is_grasped(
                ee_pos, 
                cube_pos, 
                gripper_width,
                cube_size=0.04,      # 4cm 큐브
                dist_tol=0.05,       # 5cm 거리 허용
                z_tol=0.015,         # 1.5cm Z축 정렬
                width_margin=0.015   # ±15mm 그리퍼 폭 여유 (완화: 8mm→15mm, 2.5-5.5cm 허용)
            )
            
            # Attach/Detach 처리
            if is_grasping and not self.gripper.is_attached:
                # 조건 만족 → 부착
                self.gripper.attach(
                    self.world.stage,
                    self.ee_prim_path,          # gripper_base 경로
                    "/World/DynamicCube",       # 큐브 경로
                    timestep=self.step_count
                )
                if self.current_step % 10 == 0:
                    print(f"  🟢 GRASP! dist={np.linalg.norm(cube_pos-ee_pos):.3f}, width={gripper_width:.3f}, attached={self.gripper.is_attached}")
            elif not is_grasping and self.gripper.is_attached:
                # 조건 불만족 → 분리
                self.gripper.detach(self.world.stage)
                self.gripper.detach_called_this_step = True
            else:
                self.gripper.detach_called_this_step = False
        
        # 현재 상태 관측
        obs = self._get_observation()
        
        # Reward 계산 (action 전달)
        reward = self._calculate_reward(obs, action)
        
        # 종료 조건 확인
        done = self._check_done(obs)
        
        # 스텝 카운터 증가
        self.current_step += 1
        self.step_count += 1  # 🔥 v3.7.1: 전역 스텝 카운터
        
        # ═══════════════════════════════════════════════════════════
        # 📊 로깅: 이벤트 및 진행 상황 추적
        # ═══════════════════════════════════════════════════════════
        # 🔧 v3.2: 월드 좌표로 재계산 (실제 타깃 위치 사용)
        cube_pos, _ = self.cube.get_world_pose()
        target_pos, _ = self.target.get_world_pose()  # 🔧 v3.2: Curriculum 반영
        cube_to_target_dist = float(np.linalg.norm(target_pos - cube_pos))
        
        # 성공률 계산 (최근 에피소드 기준)
        success_rate = np.mean(self.episode_successes) if len(self.episode_successes) > 0 else 0.0
        
        # V4.2: gripper 정보 (observation에서 is_grasped 제거됨)
        gripper_pos = obs[21]
        gripper_vel = obs[22]
        gripper_torque = obs[23]
        
        info = {
            "step": self.current_step,
            "cube_position": cube_pos.tolist(),  # 🔧 v3.2: 월드 좌표 (obs[11:14]는 타깃 상대좌표)
            "distance_to_target": float(cube_to_target_dist),
            # 🔥 v3.9.0: 콜백용 단계별 달성 플래그
            "reached_near_cube": self.first_reach,
            "reached_grasp": self.first_attach,      # v3.9.0: valid_grip → first_attach
            "reached_lift": self.first_lift,         # v3.9.0: lifted → first_lift
            "reached_near_target": self.episode_metrics['hold'],
            "is_success": self.success,              # v3.9.0: success flag
            # 마일스톤 이벤트 추적
            "events": {
                "first_reach": self.first_reach,
                "valid_grip": self.first_attach,     # v3.9.0: valid_grip → first_attach
                "lifted": self.first_lift,           # v3.9.0: lifted → first_lift
                "goal_near": self.episode_metrics['hold'],
                "success": self.success,
            },
            # 🔥 v3.9.0: 마일스톤 카운터
            "milestone_counts": {
                "reach": self.episode_reach_count,
                "grip": self.episode_attach_count,   # v3.9.0: grip → attach
                "lift": self.episode_lift_count,
            },
            # 그리퍼 정보 (V4.2: position 기반)
            "gripper": {
                "position": float(gripper_pos),
                "velocity": float(gripper_vel),
                "torque": float(gripper_torque),
                "attach_frames": self.attach_hold_frames,  # v3.9.0: grip_frames → attach_hold_frames
            },
            # 종료 사유
            "done_reason": "ongoing" if not done else (
                "success" if cube_to_target_dist < self.cfg.success_threshold else
                "timeout" if self.current_step >= self.max_steps else
                "safety"
            ),
            # Curriculum 정보
            "curriculum_phase": self.cfg.curriculum_phase,
            "success_rate": float(success_rate),
        }
        
        # Gymnasium API: (observation, reward, terminated, truncated, info)
        truncated = False  # 환경이 시간 제한으로 종료되지 않음
        return obs, reward, done, truncated, info
    
    def _calculate_reward(self, obs: np.ndarray, action: np.ndarray) -> float:
        """
        V4.1.0 Reward: ACTION 기반 그리퍼 보상
        
        보상 구조:
        A. Dense Reward (매 스텝):
          - EE to Cube 접근
          - Cube to Target 접근
        
        B. Milestone Rewards (v4.1.0):
          - REACH (+7.0): 5cm 진입 + 지속 보상
          - 그리퍼 닫기 (+0.5~2.5): ACTION 기반
          - ATTACH (+50.0): 물리적 부착
          - LIFT (+15.0): 큐브 들어올림
          - SUCCESS (+50.0): 최종 성공
        
        Args:
            obs: 관측 벡터 (28차원)
            action: 액션 벡터 (7차원)
        """
        # 🤖 V4.2: 관찰에서 필요한 값 추출 (24 dim)
        # Observation 구조:
        # [0:6]   - joint_positions (6 DOF)
        # [6:9]   - cube_relative_to_ee
        # [9:12]  - target_relative_to_ee
        # [12:15] - cube_to_target
        # [15:18] - ee_pos
        # [18:21] - cube_pos
        # [21:24] - gripper_state (pos, vel, torque)
        
        # 디버그: observation 길이 확인
        if self.step_count % 100 == 1:
            print(f"[REWARD-DEBUG-PRE] step={self.step_count}, obs.shape={obs.shape}, len={len(obs)}")
        
        # Observation에서 값 추출
        cube_relative_to_ee = obs[6:9]
        target_relative_to_ee = obs[9:12]
        cube_to_target = obs[12:15]
        ee_pos = obs[15:18]
        cube_pos = obs[18:21]
        gripper_pos = obs[21]
        gripper_vel = obs[22]
        gripper_torque = obs[23]
        
        # 거리 계산
        dist_to_cube = float(np.linalg.norm(cube_relative_to_ee))
        dist_cube_to_target = float(np.linalg.norm(cube_to_target))
        
        # 디버그: gripper state 검증
        if self.step_count % 100 == 1:
            print(f"[REWARD-DEBUG] step={self.step_count}, gripper_pos={gripper_pos:.4f}, gripper_vel={gripper_vel:.4f}")
        
        # ═══════════════════════════════════════════════════════════
        # 물리적 부착 확인 (Attach 기반 성공 정의)
        # ═══════════════════════════════════════════════════════════
        is_attached = self.gripper.is_attached if self.gripper else False
        
        # 보상용 게이트 (느슨 - 탐색 유도)
        # V4.2: gripper_width 대신 gripper_pos 사용
        grasp_valid_reward = (
            dist_to_cube < 0.05 and            # 5cm 거리
            (gripper_pos < 0.015)              # 그리퍼가 어느 정도 닫힘
        )
        
        # 성공용 게이트 (엄격 - 평가 전용)
        # 물리적 부착 확인
        if is_attached and not self.gripper.detach_called_this_step:
            self.attach_hold_frames += 1
        else:
            self.attach_hold_frames = 0
        
        # 리프트 확인 (부착 후 2cm 상승)
        lift_achieved = False
        if is_attached and cube_pos[2] > self.initial_cube_z + 0.02:
            lift_achieved = True
            self.lift_hold_frames += 1
        else:
            self.lift_hold_frames = 0
        
        # ═══════════════════════════════════════════════════════════
        # 🎁 A. DENSE REWARD (Δ형 - 개선량 기반) - v3.7 DENSE-HEAVY
        # ═══════════════════════════════════════════════════════════
        reward = 0.0
        
        # Time penalty 제거 (dense reward만으로 충분)
        # reward -= 0.001  # 🔥 v3.7: Time penalty 제거
        
        # 🔧 v3.7.1 NEW: EE Orientation 보상 (그리퍼가 큐브를 향하도록)
        # 문제: 로봇이 팔꿈치를 큐브에 가까이 하고, 그리퍼는 반대 방향을 향함
        # 해결: EE의 forward vector와 큐브 방향을 정렬
        try:
            from pxr import UsdGeom, Gf
            ee_prim = self.world.stage.GetPrimAtPath(self.ee_prim_path)
            if ee_prim.IsValid():
                xformable = UsdGeom.Xformable(ee_prim)
                world_transform = xformable.ComputeLocalToWorldTransform(0)
                
                # Forward vector: -Z axis in world space (USD convention)
                forward_vector = np.array([-world_transform.GetRow(2)[0],
                                          -world_transform.GetRow(2)[1],
                                          -world_transform.GetRow(2)[2]])
                
                # Cube direction (normalized)
                if dist_to_cube > 1e-6:
                    cube_direction = cube_relative_to_ee / dist_to_cube
                    
                    # Alignment: 1.0 = perfectly aligned, -1.0 = opposite
                    orientation_alignment = np.dot(forward_vector, cube_direction)
                    
                    # Strong reward for pointing gripper at cube
                    reward += 10.0 * max(0, orientation_alignment)  # Only positive alignment
        except Exception as e:
            pass  # Fallback: skip orientation reward if error
        
        # 🔧 V4.2: EE velocity 제거 (observation에 포함 안됨)
        # 이전: ee_velocity 기반 방향성 보상
        # 현재: 제거 (심플한 reward 구조)
        
        # 2. EE → Cube 접근 보상 (개선량 기반) - 더 강화!
        if self.prev_ee_to_cube_dist is not None:
            ee_progress = self.prev_ee_to_cube_dist - dist_to_cube
            reward += 20.0 * ee_progress  # 🔥 v3.7: 10.0 → 20.0 (2배 강화)
        
        # 3. 거리 기반 추가 보상 (가까울수록 높은 보상) - 더 강화!
        # 큐브에 가까울수록 지속적으로 양의 보상
        distance_reward = max(0, 0.3 - dist_to_cube) * 10.0  # 🔥 v3.7: 5.0 → 10.0 (2배 강화)
        reward += distance_reward
        self.episode_reward_breakdown['distance_reward'] += distance_reward
        
        # � v3.9.6: 거리 추적 (누적 통계만 - 리스트 제거!)
        self.episode_distance_sum += dist_to_cube
        self.episode_distance_count += 1
        if dist_to_cube < self.episode_min_distance:
            self.episode_min_distance = dist_to_cube
        
        # 🔥 v3.9.6: 단계별 근접 보상 (카운터만 - 리스트 제거!)
        # v3.9.2 기반 + 단계별 강화 (v3.9.3 실패 교훈)
        stage_bonus = 0.0
        if dist_to_cube < 0.07:  # 7cm 이하
            stage_bonus += 10.0
            self.stage_7cm_count += 1
        if dist_to_cube < 0.05:  # 5cm 이하
            stage_bonus += 20.0
            self.stage_5cm_count += 1
        if dist_to_cube < 0.03:  # 3cm 이하
            stage_bonus += 30.0
            self.stage_3cm_count += 1
        reward += stage_bonus
        self.episode_reward_breakdown['stage_bonus'] += stage_bonus
        
        # 🔥 V4.1: Dense 그리퍼 보상 제거 (Milestone로 이동)
        # V4.0/V4.1 문제: 이 부분이 매 스텝 +3.0 지급 (100% 트리거)
        # 해결: 완전 제거, Milestone Reward에서만 지급
        gripper_bonus = 0.0
        # if gripper_width > 0.01:  # ← V3 코드 (제거됨)
        #     gripper_bonus = 3.0
        #     reward += gripper_bonus
        #     self.gripper_reward_count += 1
        #     self.gripper_reward_total += gripper_bonus
        self.episode_reward_breakdown['gripper_bonus'] += gripper_bonus
        
        # 🔥 V4.2: 거리-폭 연동 보상 제거 (gripper_width → gripper_pos)
        # 이전: gripper_width 기반 synergy bonus
        # 현재: gripper_pos 기반으로 간소화
        synergy_bonus = 0.0
        if dist_to_cube < 0.10 and gripper_pos < 0.020:  # 10cm 이내 + 그리퍼 닫는 중
            synergy_bonus += 10.0
        if dist_to_cube < 0.05 and gripper_pos < 0.015:  # 5cm 이내 + 거의 닫힘
            synergy_bonus += 40.0
        reward += synergy_bonus
        self.episode_reward_breakdown['synergy_bonus'] += synergy_bonus
        
        # 🔥 V4.2: Width Penalty 제거 (gripper_width 없음)
        # 이전: gripper_width 기반 이차 패널티
        # 현재: 제거 (심플한 reward 구조)
        
        # 6. Cube → Target 접근 보상 (부착 시만, 개선량 기반)
        if is_attached and self.prev_cube_to_target_dist is not None:
            cube_progress = self.prev_cube_to_target_dist - dist_cube_to_target
            reward += 15.0 * cube_progress  # 부착 후 목표로 이동
        
        # 거리 이력 업데이트
        self.prev_ee_to_cube_dist = dist_to_cube
        self.prev_cube_to_target_dist = dist_cube_to_target
        
        # ═══════════════════════════════════════════════════════════
        # 🎁 B. v4.1.0: Milestone Rewards (그리퍼 보상 수정)
        # ═══════════════════════════════════════════════════════════
        
        # 1️⃣ REACH (지속적): EE가 큐브 근처 유지 (5cm 이내)
        if dist_to_cube < 0.05:
            reward += 2.0  # 매 스텝 보상 (유지 동기 부여)
            if not self.first_reach:
                reward += 5.0  # 최초 달성 보너스
                self.first_reach = True
                self.episode_reach_count += 1
                self.episode_metrics['reach'] = True
                print(f"  🎯 REACH (+7.0 = 2.0 지속 + 5.0 최초)")
            
            # 🤖 V4.2: REACH 후 그리퍼 닫기 유도 (ACTION 기반)
            # action[5] < -0.3: 닫기 액션 (그리퍼 joint 음수 이동)
            if action[5] < -0.3:
                gripper_try_reward = 0.5  # 닫기 시도 보상
                reward += gripper_try_reward
                self.episode_reward_breakdown['gripper_bonus'] += gripper_try_reward
                
                # 실제로 그리퍼가 닫혔는지 확인 (position 기반)
                current_joint_positions = self.robot.get_joint_positions()
                gripper_pos = current_joint_positions[5]  # 현재 그리퍼 position
                if gripper_pos < 0.01:  # 거의 닫힘 (0.025의 40%)
                    gripper_close_reward = 2.0
                    reward += gripper_close_reward
                    self.episode_reward_breakdown['gripper_bonus'] += gripper_close_reward
                    
                    if not self._gripper_close_success:
                        print(f"  ✋ GRIPPER CLOSE SUCCESS (+2.5) ← 닫기 액션 + position < 0.01!")
                        self._gripper_close_success = True
                        self.gripper_reward_count += 1
                        self.gripper_reward_total += 2.5
        else:
            # REACH 달성 후 멀어지면 패널티 (V4: 강화)
            if self.first_reach and dist_to_cube > 0.05:
                reward -= 2.0  # 멀어짐 패널티 (V3: -1.0 → V4: -2.0)
        
        # 2️⃣ ATTACH (+50): 물리적 부착 달성 (엄격 - 진짜 그립만)
        if not self.first_attach and is_attached:
            reward += 50.0  # 부착 성공 보상
            self.first_attach = True
            self.episode_attach_count += 1
            self.episode_metrics['attach'] = True
            print(f"  🤝 ATTACH (+50.0) ← 물리적 부착!")
        
        # 🆕 V4.1: ATTACH 준비 보상 제거 (과다 보상 방지)
        # (ATTACH 성공 시 +50으로 충분)
        
        # 3️⃣ LIFT (+15): 리프트 달성 (부착 후 2cm 상승)
        if not self.first_lift and lift_achieved:
            reward += 15.0
            self.first_lift = True
            self.episode_lift_count += 1
            self.episode_metrics['lift'] = True
            print(f"  ⬆️ LIFT (+15.0) [z={cube_pos[2]:.3f}, Δz={cube_pos[2]-self.initial_cube_z:.3f}]")
        
        # 4️⃣ SUCCESS (+50): 최종 성공 (attach & lift & hold 10프레임)
        if lift_achieved and self.lift_hold_frames >= 10 and not self.success:
            reward += 50.0
            self.success = True
            self.episode_success_count += 1
            self.episode_metrics['hold'] = True
            self.episode_metrics['success'] = True
            print(f"  � SUCCESS (+50.0) ← 진짜 성공! (hold={self.lift_hold_frames})")
        
        # 🤖 V4.2: 진단 로그 (50 스텝마다)
        if self.current_step % 50 == 0 and not is_attached:
            current_joint_positions = self.robot.get_joint_positions()
            print(f"  🔍 [v4.2] dist={dist_to_cube:.3f}, "
                  f"gripper_pos={current_joint_positions[5]:.4f}, "
                  f"attached={is_attached}, "
                  f"lift_z={cube_pos[2]-self.initial_cube_z:.3f}, "
                  f"gripper_action={action[5]:.3f}")
        
        # ═══════════════════════════════════════════════════════════
        # 🎁 C. REWARD CLIPPING (1스텝 보상 제한)
        # ═══════════════════════════════════════════════════════════
        # 🔥 v3.7.6 FIX: 클리핑 완화 (-2.0→-10.0) - 점진적 향상 신호 보존
        # VecNormalize에서 이미 ±10 클립 들어가므로 여기서 더 줄이면 학습 신호 손실
        if reward < 90.0:  # 큰 이벤트 보상 제외
            reward = np.clip(reward, -10.0, 10.0)  # -2.0→-10.0 완화
        
        # Previous reward 저장 (관측에 포함)
        self.previous_reward = reward
        
        return reward
    
    def _check_done(self, obs: np.ndarray) -> bool:
        """에피소드 종료 조건 + 성공률 추적
        
        ✅ SUCCESS 조건 강화:
        - threshold 이내 + N프레임 연속 유지 필수!
        """
        # 🔧 BUG FIX: 월드 좌표로 정확하게 재계산
        cube_pos, _ = self.cube.get_world_pose()
        target_pos = np.array(self.cfg.target_position)
        cube_to_target_dist = float(np.linalg.norm(target_pos - cube_pos))
        
        # 관측 벡터의 큐브 위치도 참고 (디버깅용)
        cube_pos_obs = obs[11:14]
        
        # ═══════════════════════════════════════════════════════════
        # ✅ SUCCESS 조건: threshold 이내 + N프레임 연속 유지
        # ═══════════════════════════════════════════════════════════
        if cube_to_target_dist < self.cfg.success_threshold:
            # 아직 N프레임 유지 안 됨 → 계속 진행
            if self.success_frames < self.cfg.success_hold_frames:
                # 진행 상황 로그 (매 프레임마다는 아니고 5프레임마다)
                if self.success_frames % 5 == 0 and self.success_frames > 0:
                    print(f"  ⏳ Holding... {self.success_frames}/{self.cfg.success_hold_frames} frames (dist: {cube_to_target_dist:.3f}m)")
                return False  # 아직 종료하지 않음!
            else:
                # N프레임 유지 완료 → SUCCESS!
                print(f"  ✅ SUCCESS CONFIRMED! Distance: {cube_to_target_dist:.3f}m (held {self.success_frames} frames)")
                self._record_success(True)
                return True
        
        # 최대 스텝 도달
        if self.current_step >= self.max_steps:
            print(f"  ⏱️ TimeLimit reached: {self.current_step}/{self.max_steps} steps")
            self._record_success(False)
            self._print_episode_stats()  # 🔍 v3.9.5: 에피소드 종료 시 통계 출력
            return True
        
        # 물체가 테이블 밖으로 떨어짐 (Z < 0)
        if cube_pos[2] < -0.1:
            print(f"  ❌ Cube fell off table (Z: {cube_pos[2]:.3f}m)")
            self._record_success(False)
            return True
            self._record_success(False)
            return True
        
        return False
    
    def _record_success(self, success: bool):
        """
        성공률 추적 및 자동 승급 체크
        """
        # 성공 여부 기록
        self.episode_successes.append(1.0 if success else 0.0)
        
        # 윈도우 크기 제한
        if len(self.episode_successes) > self.cfg.success_rate_window:
            self.episode_successes.pop(0)
        
        # 성공률 계산 (최소 50 에피소드 이상)
        if len(self.episode_successes) >= 50:
            success_rate = np.mean(self.episode_successes)
    
    def _print_episode_stats(self):
        """
        🔍 v3.9.5: 에피소드 종료 시 상세 통계 출력
        """
        print(f"\n" + "="*60)
        print(f"📊 에피소드 종료 통계 (Step {self.step_count})")
        print(f"="*60)
        
        # 1. 거리 분포
        if hasattr(self, 'episode_distances') and self.episode_distances:
            print(f"\n1️⃣ 거리 분석:")
            print(f"   최소 거리: {self.episode_min_distance*100:.1f}cm")
            print(f"   평균 거리: {np.mean(self.episode_distances)*100:.1f}cm")
            print(f"   최대 거리: {np.max(self.episode_distances)*100:.1f}cm")
            print(f"   표준편차: {np.std(self.episode_distances)*100:.1f}cm")
            
            # 히스토그램 (텍스트 기반)
            bins = [0, 0.05, 0.07, 0.10, 0.15, 0.20, 1.0]  # 5cm, 7cm, 10cm, 15cm, 20cm
            hist, _ = np.histogram(self.episode_distances, bins=bins)
            print(f"   분포:")
            print(f"     < 5cm:  {hist[0]:4d}회 ({hist[0]/len(self.episode_distances)*100:5.1f}%)")
            print(f"     5-7cm:  {hist[1]:4d}회 ({hist[1]/len(self.episode_distances)*100:5.1f}%)")
            print(f"     7-10cm: {hist[2]:4d}회 ({hist[2]/len(self.episode_distances)*100:5.1f}%)")
            print(f"     10-15cm: {hist[3]:4d}회 ({hist[3]/len(self.episode_distances)*100:5.1f}%)")
            print(f"     15-20cm: {hist[4]:4d}회 ({hist[4]/len(self.episode_distances)*100:5.1f}%)")
            print(f"     > 20cm:  {hist[5]:4d}회 ({hist[5]/len(self.episode_distances)*100:5.1f}%)")
        
        # 2. 단계별 보상 트리거
        if hasattr(self, 'stage_7cm_triggered'):
            print(f"\n2️⃣ 단계별 보상 트리거:")
            print(f"   < 7cm:  {self.stage_7cm_triggered}회")
            if hasattr(self, 'stage_7cm_steps') and self.stage_7cm_steps:
                print(f"           첫 트리거: Step {self.stage_7cm_steps[0]}")
            print(f"   < 5cm:  {self.stage_5cm_triggered}회")
            if hasattr(self, 'stage_5cm_steps') and self.stage_5cm_steps:
                print(f"           첫 트리거: Step {self.stage_5cm_steps[0]}")
            print(f"   < 3cm:  {self.stage_3cm_triggered}회")
            if hasattr(self, 'stage_3cm_steps') and self.stage_3cm_steps:
                print(f"           첫 트리거: Step {self.stage_3cm_steps[0]}")
        
        # 3. 그리퍼 보상 통계
        if hasattr(self, 'gripper_reward_count'):
            print(f"\n3️⃣ 그리퍼 보상 통계:")
            print(f"   트리거 횟수: {self.gripper_reward_count}회 ({self.gripper_reward_count/self.step_count*100:.1f}%)")
            if self.gripper_reward_count > 0:
                avg_gripper = self.gripper_reward_total / self.gripper_reward_count
                print(f"   평균 보상: {avg_gripper:.2f}")
                print(f"   총 보상: {self.gripper_reward_total:.1f}")
        
        # 4. 액션 분포
        if hasattr(self, 'action_stats') and self.action_stats['gripper_actions']:
            gripper_actions = self.action_stats['gripper_actions']
            print(f"\n4️⃣ 그리퍼 액션 분포:")
            print(f"   평균: {np.mean(gripper_actions):.3f}")
            print(f"   표준편차: {np.std(gripper_actions):.3f}")
            print(f"   최소: {np.min(gripper_actions):.3f}")
            print(f"   최대: {np.max(gripper_actions):.3f}")
            
            # 액션 히스토그램
            positive = np.sum(np.array(gripper_actions) > 0.1)
            negative = np.sum(np.array(gripper_actions) < -0.1)
            neutral = len(gripper_actions) - positive - negative
            print(f"   열기(>0.1):  {positive}회 ({positive/len(gripper_actions)*100:.1f}%)")
            print(f"   닫기(<-0.1): {negative}회 ({negative/len(gripper_actions)*100:.1f}%)")
            print(f"   중립:        {neutral}회 ({neutral/len(gripper_actions)*100:.1f}%)")
        
        # 5. 보상 분해
        print(f"\n5️⃣ 보상 분해:")
        total = sum(self.episode_reward_breakdown.values())
        for key, value in self.episode_reward_breakdown.items():
            if abs(value) > 0.01:  # 0이 아닌 항목만
                percentage = (value / total * 100) if total != 0 else 0
                print(f"   {key:20s}: {value:8.1f} ({percentage:5.1f}%)")
        print(f"   {'TOTAL':20s}: {total:8.1f}")
        
        # 6. 마일스톤 달성
        print(f"\n6️⃣ 마일스톤 달성:")
        print(f"   REACH:   {'✅' if self.episode_metrics['reach'] else '❌'}")
        print(f"   ATTACH:  {'✅' if self.episode_metrics['attach'] else '❌'}")
        print(f"   LIFT:    {'✅' if self.episode_metrics['lift'] else '❌'}")
        print(f"   SUCCESS: {'✅' if self.episode_metrics['success'] else '❌'}")
        
        # 7. 환경 파라미터
        print(f"\n7️⃣ 환경 파라미터:")
        print(f"   dist_tol: {self.gate_config.grip_dist_tol*100:.1f}cm")
        print(f"   width_margin: {self.gate_config.grip_width_margin*100:.1f}cm")
        print(f"   ideal_width: {self.gate_config.cube_size*100:.1f}cm")
        
        print(f"="*60 + "\n")
    
    def _check_curriculum_upgrade(self):
        """
        Curriculum 승급 체크 (별도 함수로 분리)
        """
        # 성공률 계산 (최소 50 에피소드 이상)
        if len(self.episode_successes) >= 50:
            success_rate = np.mean(self.episode_successes)
            
            # Curriculum 승급 체크 (Phase 0 → Phase 1)
            if (self.cfg.curriculum_enabled and 
                self.cfg.curriculum_phase == 0 and
                success_rate >= self.cfg.success_rate_threshold):
                
                print("\n" + "=" * 60)
                print(f"🎓 CURRICULUM UPGRADE! Phase 0 → Phase 1")
                print(f"   Success Rate: {success_rate:.1%} (≥{self.cfg.success_rate_threshold:.0%})")
                print(f"   Window: {len(self.episode_successes)} episodes")
                print("=" * 60 + "\n")
                
                # Phase 1으로 승급
                self.cfg.curriculum_phase = 1
                self.episode_successes.clear()  # 성공률 리셋
    
    def render(self):
        """렌더링 (Isaac Sim에서 자동 처리)"""
        pass
    
    def close(self):
        """환경 종료"""
        print("\n🛑 환경 종료")
        self.world.stop()
    
if __name__ == "__main__":
    print("🚀 RoArm-M3 Pick and Place 환경 테스트\n")
    
    # 환경 생성
    cfg = RoArmPickPlaceEnvCfg()
    cfg.episode_length_s = 5.0  # 짧게 테스트
    
    env = RoArmPickPlaceEnv(cfg)
    
    # 몇 에피소드 테스트
    for episode in range(2):
        print(f"\n{'='*60}")
        print(f"Episode {episode + 1}")
        print(f"{'='*60}")
        
        obs = env.reset()
        print(f"Initial observation shape: {obs.shape}")
        
        done = False
        total_reward = 0
        step = 0
        
        while not done and step < 100:
            # 랜덤 액션
            action = np.random.uniform(-0.5, 0.5, size=8)
            
            obs, reward, done, info = env.step(action)
            total_reward += reward
            step += 1
            
            if step % 20 == 0:
                print(f"  Step {step}: Reward={reward:.2f}, Distance={info['distance_to_target']:.3f}m")
        
        print(f"\n  📊 Episode {episode + 1} 결과:")
        print(f"    - Total steps: {step}")
        print(f"    - Total reward: {total_reward:.2f}")
        print(f"    - Final distance: {info['distance_to_target']:.3f}m")
    
    env.close()
    print("\n✅ 테스트 완료!")
