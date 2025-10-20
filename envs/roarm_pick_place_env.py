#!/usr/bin/env python3
"""
RoArm-M3 Pick and Place 강화학습 환경
Isaac Sim 5.0 + omni.isaac.lab 기반
"""

import numpy as np
import torch
from typing import Dict, Tuple

# Isaac Sim imports
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Gf

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


@configclass
class RoArmPickPlaceEnvCfg(DirectRLEnvCfg):
    """RoArm-M3 Pick and Place 환경 설정"""
    
    # Environment settings
    num_envs: int = 1
    env_spacing: float = 2.5
    episode_length_s: float = 10.0
    
    # Robot settings
    urdf_path: str = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_multiprim.urdf"
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
    
    # Phase 0: Easy Mode (중간 거리, 15~20cm → 더 도전적!)
    easy_cube_distance: Tuple[float, float] = (0.15, 0.20)  # 15~20cm (10-15cm → 15-20cm)
    easy_target_distance: Tuple[float, float] = (0.25, 0.30)  # 25~30cm (20-25cm → 25-30cm)
    
    # Phase 1: Normal Mode (먼 거리, 35~50cm → 더 어렵게!)
    normal_cube_distance: Tuple[float, float] = (0.35, 0.50)  # 35~50cm (25-35cm → 35-50cm)
    normal_target_distance: Tuple[float, float] = (0.35, 0.50)  # 35~50cm (25-35cm → 35-50cm)
    
    # ═══════════════════════════════════════════════════════════
    # 📚 자동 승급 조건 (완화됨!)
    # ═══════════════════════════════════════════════════════════
    success_rate_window: int = 100       # 최근 100 에피소드 (200 → 100)
    success_rate_threshold: float = 0.30  # 성공률 30% 이상 (60% → 30%)
    reach_milestone_threshold: int = 5    # REACH 5회 달성 (10회 → 5회)


class RoArmPickPlaceEnv:
    """
    RoArm-M3 Pick and Place 강화학습 환경 (Dense Reward)
    
    Task: 큐브를 집어서 타겟 위치로 옮기기
    
    Observation Space (25 dim):
        - Joint positions (6 dim): joint_1 ~ joint_6
        - Gripper state (2 dim): left_finger, right_finger positions
        - End-effector position (3 dim): x, y, z
        - Cube position (3 dim): x, y, z
        - Target position (3 dim): x, y, z
        - EE → Cube vector (3 dim): dx, dy, dz
        - Cube → Target vector (3 dim): dx, dy, dz
        - Gripper width (1 dim): distance between fingers
        - Is grasped (1 dim): 1.0 if grasping, 0.0 otherwise
    
    Action Space (8 dim):
        - Joint position deltas (6 dim): joint_1 ~ joint_6
        - Gripper position deltas (2 dim): left_finger, right_finger
    """
    
    def __init__(self, cfg: RoArmPickPlaceEnvCfg = None):
        """환경 초기화"""
        self.cfg = cfg if cfg else RoArmPickPlaceEnvCfg()
        
        # Isaac Sim World 초기화
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        
        print("=" * 60)
        print("🤖 RoArm-M3 Pick and Place Environment")
        print("=" * 60)
        
        # 로봇 로드
        self._load_robot()
        
        # 물체 생성
        self._create_objects()
        
        # 환경 변수 초기화
        self.current_step = 0
        self.max_steps = int(self.cfg.episode_length_s * 60)  # 60 FPS 가정
        
        # ═══════════════════════════════════════════════════════════
        # 📊 Observation/Action Space (개선: EE 기준 상대 좌표)
        # ═══════════════════════════════════════════════════════════
        # Observation: 28 dim (EE 기준 상대 좌표!)
        #   - Joint positions (8)
        #   - Cube pos relative to EE (3) ← 핵심!
        #   - Target pos relative to EE (3) ← 핵심!
        #   - Cube to Target vector (3)
        #   - EE velocity (3)
        #   - Cube velocity (3)
        #   - Gripper width (1)
        #   - Is grasped (1)
        #   - Distance to cube (1)
        #   - Distance cube to target (1)
        #   - Previous reward (1)
        self.observation_space_dim = 28
        self.action_space_dim = 8
        
        # 이전 상태 저장 (속도 계산 & Dense Reward)
        self.prev_ee_pos = None
        self.prev_cube_pos = None
        self.prev_ee_to_cube_dist = None
        self.prev_cube_to_target_dist = None
        self.previous_reward = 0.0
        
        # ═══════════════════════════════════════════════════════════
        # 🎯 SHAPED-SPARSE: 1회성 이벤트 플래그
        # ═══════════════════════════════════════════════════════════
        self.first_reach = False      # EE가 큐브에 처음 근접 (5cm)
        self.valid_grip = False       # 유효한 그립 달성 (3프레임)
        self.lifted = False           # 큐브를 들어올림 (5cm)
        self.goal_near = False        # 큐브가 목표에 근접 (8cm)
        
        # 프레임 히스테리시스 카운터
        self.grip_frames = 0          # 연속 그립 프레임
        self.success_frames = 0       # 연속 성공 프레임
        
        # 마일스톤 카운터 (에피소드 통계용)
        self.episode_reach_count = 0
        self.episode_grip_count = 0
        self.episode_lift_count = 0
        
        # ═══════════════════════════════════════════════════════════
        # 📚 CURRICULUM: 성공률 추적
        # ═══════════════════════════════════════════════════════════
        self.episode_successes = []   # 최근 에피소드 성공 여부
        
        print(f"\n📊 환경 정보:")
        print(f"  - Observation dim: {self.observation_space_dim}")
        print(f"  - Action dim: {self.action_space_dim}")
        print(f"  - Episode length: {self.cfg.episode_length_s}s * 60 FPS = {self.max_steps} steps")
        print(f"  - Success threshold: {self.cfg.success_threshold}m ({int(self.cfg.success_threshold*100)}cm)")
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
        
        # ✅ Joint drive 설정 (USD API 사용)
        print(f"  ⏳ Joint drive 설정 중...")
        from pxr import UsdPhysics, PhysxSchema
        stage = self.world.stage
        
        # 모든 joint에 대해 drive 설정
        for i, joint_name in enumerate(self.joint_names):
            joint_prim = stage.GetPrimAtPath(f"{prim_path}/{joint_name}")
            if joint_prim and joint_prim.IsValid():
                # Drive API 적용 (완화된 값으로 부드러운 움직임)
                drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
                if i < 6:  # 팔 joint (완화: 10000 → 5000)
                    drive_api.GetStiffnessAttr().Set(5000.0)  # 완화
                    drive_api.GetDampingAttr().Set(500.0)     # 완화
                    drive_api.GetMaxForceAttr().Set(500.0)    # 완화
                else:  # 그리퍼
                    drive_api.GetStiffnessAttr().Set(1000.0)
                    drive_api.GetDampingAttr().Set(100.0)
                    drive_api.GetMaxForceAttr().Set(100.0)
        
        print(f"  ✅ Joint drive 설정 완료! (완화된 값)")
    
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
    
    def reset(self) -> np.ndarray:
        """환경 리셋"""
        print(f"\n🔄 환경 리셋 (Step {self.current_step})")
        
        # World 리셋
        self.world.reset()
        
        # 로봇 초기 자세 (Home position) - 약간 위로 올린 자세
        # 중력에 의해 떨어지지 않도록 안정적인 자세
        home_positions = np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])  # 8 DOF
        self.robot.set_joint_positions(home_positions)
        self.robot.set_joint_velocities(np.zeros(8))
        
        # Physics 스텝 실행하여 안정화
        for _ in range(10):
            self.world.step(render=False)
        
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
                    0.05  # 테이블 높이
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
                    0.05
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
        
        # 타겟 위치 업데이트 (Curriculum 적용)
        if self.cfg.curriculum_enabled:
            self.target.set_world_pose(position=target_pos)
        
        # 스텝 카운터 초기화
        self.current_step = 0
        
        # 이전 거리 초기화 (보상 계산용)
        self.prev_ee_to_cube_dist = None
        self.prev_cube_to_target_dist = None
        
        # ═══════════════════════════════════════════════════════════
        # 🎯 SHAPED-SPARSE: 1회성 플래그 리셋
        # ═══════════════════════════════════════════════════════════
        self.first_reach = False
        self.valid_grip = False
        self.lifted = False
        self.goal_near = False
        self.grip_frames = 0
        self.success_frames = 0
        
        # 마일스톤 카운터 리셋
        self.episode_reach_count = 0
        self.episode_grip_count = 0
        self.episode_lift_count = 0
        
        # 초기 observation 반환
        return self._get_observation()
    
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
        joint_positions = self.robot.get_joint_positions()[:8]
        ee_pos = self._get_ee_position()  # 월드 좌표
        cube_pos, _ = self.cube.get_world_pose()  # 월드 좌표
        target_pos = np.array(self.cfg.target_position)  # 월드 좌표 (고정)
        
        # ═══════════════════════════════════════════════════════════
        # 2. EE 기준 상대 좌표 변환 (핵심!)
        # ═══════════════════════════════════════════════════════════
        # 정책이 학습하기 쉽도록 "EE에서 본" 큐브/타겟 위치 제공
        cube_relative_to_ee = cube_pos - ee_pos       # EE → Cube 벡터
        target_relative_to_ee = target_pos - ee_pos   # EE → Target 벡터
        cube_to_target = target_pos - cube_pos        # Cube → Target 벡터
        
        # ═══════════════════════════════════════════════════════════
        # 3. 속도 계산 (시간적 정보)
        # ═══════════════════════════════════════════════════════════
        if self.prev_ee_pos is not None:
            ee_velocity = (ee_pos - self.prev_ee_pos) * 60.0  # 60 FPS 가정
            cube_velocity = (cube_pos - self.prev_cube_pos) * 60.0
        else:
            ee_velocity = np.zeros(3)
            cube_velocity = np.zeros(3)
        
        self.prev_ee_pos = ee_pos.copy()
        self.prev_cube_pos = cube_pos.copy()
        
        # ═══════════════════════════════════════════════════════════
        # 4. 그리퍼 상태
        # ═══════════════════════════════════════════════════════════
        gripper_width = joint_positions[6] + joint_positions[7]
        
        # Is grasped: EE 근처 + 그리퍼 닫힘 (임계치 완화)
        ee_to_cube_dist = np.linalg.norm(cube_relative_to_ee)
        is_grasped = 1.0 if (ee_to_cube_dist < 0.08 and gripper_width < 0.025) else 0.0
        
        # ═══════════════════════════════════════════════════════════
        # 5. 거리 정보
        # ═══════════════════════════════════════════════════════════
        dist_to_cube = ee_to_cube_dist
        dist_cube_to_target = np.linalg.norm(cube_to_target)
        
        # ═══════════════════════════════════════════════════════════
        # 6. Observation 벡터 구성 (28 dim)
        # ═══════════════════════════════════════════════════════════
        obs = np.concatenate([
            joint_positions[:6],          # Joint positions (6)
            joint_positions[6:8],         # Gripper state (2)
            cube_relative_to_ee,          # Cube relative to EE (3) ← 핵심!
            target_relative_to_ee,        # Target relative to EE (3) ← 핵심!
            cube_to_target,               # Cube to Target (3)
            ee_velocity,                  # EE velocity (3)
            cube_velocity,                # Cube velocity (3)
            [gripper_width],              # Gripper width (1)
            [is_grasped],                 # Is grasped (1)
            [dist_to_cube],               # Distance to cube (1)
            [dist_cube_to_target],        # Distance cube to target (1)
            [self.previous_reward],       # Previous reward (1)
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
        """End-effector 위치 계산 (간단한 forward kinematics)"""
        # 실제로는 robot.get_link_world_pose()를 사용해야 하지만
        # 여기서는 간단히 joint 값 기반으로 근사
        joint_positions = self.robot.get_joint_positions()
        
        # 베이스에서 시작
        z_base = 0.06  # base_link height
        z_link1 = 0.08  # link_1 height
        
        # Joint 2-4는 수평 암 (link_2, link_3, link_4)
        link2_length = 0.16
        link3_length = 0.15
        
        # 간단히 X축 방향으로 투영 (실제는 더 복잡)
        x_reach = link2_length * np.cos(joint_positions[1]) + \
                  link3_length * np.cos(joint_positions[1] + joint_positions[2])
        
        z_reach = z_base + z_link1 + \
                  link2_length * np.sin(joint_positions[1]) + \
                  link3_length * np.sin(joint_positions[1] + joint_positions[2])
        
        # Joint 1은 Z축 회전
        y_offset = x_reach * np.sin(joint_positions[0])
        x_offset = x_reach * np.cos(joint_positions[0])
        
        return np.array([x_offset, y_offset, z_reach])
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """환경 스텝 실행"""
        # Action: joint position deltas (8 dim)
        action = np.clip(action, -1.0, 1.0)  # [-1, 1] 범위로 제한
        
        # 현재 joint positions 가져오기
        current_positions = self.robot.get_joint_positions()
        
        # Position delta 스케일링 (작은 움직임)
        max_deltas = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.005, 0.005])
        position_deltas = action * max_deltas
        
        # 새로운 목표 위치 계산
        target_positions = current_positions + position_deltas
        
        # Joint limits 적용
        target_positions = np.clip(
            target_positions,
            [-3.14, -1.57, -1.57, -3.14, -3.14, -3.14, 0.0, 0.0],  # lower limits
            [3.14, 1.57, 1.57, 3.14, 3.14, 3.14, 0.04, 0.04]  # upper limits
        )
        
        # 로봇에 position 명령 전송 (직접 설정)
        self.robot.set_joint_positions(target_positions)
        
        # Physics 시뮬레이션 스텝 (학습 시 render=False 권장)
        self.world.step(render=False)
        
        # 현재 상태 관측
        obs = self._get_observation()
        
        # Reward 계산
        reward = self._calculate_reward(obs)
        
        # 종료 조건 확인
        done = self._check_done(obs)
        
        # 스텝 카운터 증가
        self.current_step += 1
        
        # ═══════════════════════════════════════════════════════════
        # 📊 로깅: 이벤트 및 진행 상황 추적
        # ═══════════════════════════════════════════════════════════
        # 🔧 BUG FIX: 올바른 인덱스 사용 (14:17 = cube_to_target 벡터)
        # 더 신뢰도 높은 방법: 월드 좌표로 재계산
        cube_pos, _ = self.cube.get_world_pose()
        target_pos = np.array(self.cfg.target_position)
        cube_to_target_dist = float(np.linalg.norm(target_pos - cube_pos))
        
        # 성공률 계산 (최근 에피소드 기준)
        success_rate = np.mean(self.episode_successes) if len(self.episode_successes) > 0 else 0.0
        
        # 그리퍼 정보 추가
        gripper_width = obs[23]
        is_grasped = obs[24]
        
        info = {
            "step": self.current_step,
            "cube_position": obs[11:14].tolist(),
            "distance_to_target": float(cube_to_target_dist),
            # 마일스톤 이벤트 추적
            "events": {
                "first_reach": self.first_reach,
                "valid_grip": self.valid_grip,
                "lifted": self.lifted,
                "goal_near": self.goal_near,
                "success": cube_to_target_dist < self.cfg.success_threshold,
            },
            # 마일스톤 카운터
            "milestone_counts": {
                "reach": self.episode_reach_count,
                "grip": self.episode_grip_count,
                "lift": self.episode_lift_count,
            },
            # 그리퍼 정보
            "gripper": {
                "width": float(gripper_width),
                "is_grasped": float(is_grasped),
                "grip_frames": self.grip_frames,
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
        
        return obs, reward, done, info
    
    def _calculate_reward(self, obs: np.ndarray) -> float:
        """
        🎯 개선된 HYBRID REWARD: Shaped-Sparse + Dense
        
        개선 사항:
        1. Dense Reward 추가: 매 스텝 거리 기반 피드백
        2. Shaped-Sparse: 마일스톤 이벤트 보상 유지
        3. 게이팅 강화: grasp_valid 체크
        
        보상 구조:
        A. Dense Reward (매 스텝):
          - EE → Cube 접근: -distance * 3.0
          - Cube → Target 접근: -distance * 2.0 (grasp_valid 시)
          - 진전 보너스: +0.5 (거리 줄어들 때)
        
        B. Shaped-Sparse (1회성):
          - 근접 (+5): EE가 큐브 5cm 이내
          - 그립 (+10): 유효 그립 3프레임
          - 리프트 (+15): 큐브 5cm 이상
          - 목표 근접 (+20): 큐브가 목표 8cm 이내
          - Success (+100): 목표 5cm 이내
        
        C. Penalty:
          - Time penalty: -0.01
        """
        # 관찰에서 필요한 값 추출 (개선된 28 dim 관측)
        cube_relative_to_ee = obs[8:11]    # EE 기준 상대 좌표
        target_relative_to_ee = obs[11:14]  # EE 기준 상대 좌표
        cube_to_target = obs[14:17]
        gripper_width = obs[23]
        is_grasped = obs[24]
        dist_to_cube = obs[25]
        dist_cube_to_target = obs[26]
        
        # 월드 좌표 재구성 (디버깅용)
        ee_pos = self._get_ee_position()
        cube_pos = ee_pos + cube_relative_to_ee
        
        # ═══════════════════════════════════════════════════════════
        # 🔒 GATING: grasp_valid 체크 (임계치 완화)
        # ═══════════════════════════════════════════════════════════
        grasp_valid = (
            dist_to_cube < 0.08 and         # EE가 8cm 이내
            gripper_width < 0.025 and       # 그리퍼가 닫힘 (0.02 → 0.025)
            cube_pos[2] > 0.03              # 큐브가 바닥 위
        )
        
        if grasp_valid:
            self.grip_frames += 1
        else:
            self.grip_frames = 0
        
        # ═══════════════════════════════════════════════════════════
        # 🎁 A. DENSE REWARD (Δ형 - 개선량 기반)
        # ═══════════════════════════════════════════════════════════
        reward = 0.0
        
        # Time penalty 완화 (기존 0.01 → 0.001)
        reward -= 0.001
        
        # 1. EE → Cube 접근 보상 (개선량 기반)
        if self.prev_ee_to_cube_dist is not None:
            ee_progress = self.prev_ee_to_cube_dist - dist_to_cube
            reward += 5.0 * ee_progress  # 가까워지면 +, 멀어지면 -
        
        # 2. Cube → Target 접근 보상 (grasp_valid 시만, 개선량 기반)
        if grasp_valid and self.prev_cube_to_target_dist is not None:
            cube_progress = self.prev_cube_to_target_dist - dist_cube_to_target
            reward += 4.0 * cube_progress  # 가까워지면 +, 멀어지면 -
        
        # 거리 이력 업데이트
        self.prev_ee_to_cube_dist = dist_to_cube
        self.prev_cube_to_target_dist = dist_cube_to_target
        
        # ═══════════════════════════════════════════════════════════
        # 🎁 B. SHAPED-SPARSE REWARDS (1회성 이벤트, 상향 조정)
        # ═══════════════════════════════════════════════════════════
        
        # 1️⃣ 근접 보상 (+10): EE가 큐브에 처음 근접
        if not self.first_reach and dist_to_cube < 0.05:
            reward += 10.0
            self.first_reach = True
            self.episode_reach_count += 1
            print(f"  🎯 Milestone: REACH! (+10.0)")
        
        # 2️⃣ 그립 보상 (+40): 유효 그립 3프레임 유지 [게이팅]
        if not self.valid_grip and grasp_valid and self.grip_frames >= 3:
            reward += 40.0
            self.valid_grip = True
            self.episode_grip_count += 1
            print(f"  ✊ Milestone: GRIP! (+40.0)")
        
        # 3️⃣ 리프트 보상 (+50): 큐브 5cm 이상 들어올림 [게이팅]
        if not self.lifted and grasp_valid and cube_pos[2] > 0.05:
            reward += 50.0
            self.lifted = True
            self.episode_lift_count += 1
            print(f"  ⬆️ Milestone: LIFT! (+50.0)")
        
        # 4️⃣ 목표 근접 보상 (제거): LIFT와 SUCCESS 사이 간격이 크지 않아 제거
        # if not self.goal_near and grasp_valid and dist_cube_to_target < 0.08:
        #     reward += 20.0
        #     self.goal_near = True
        #     print(f"  🎯 Milestone: GOAL NEAR! (+20.0)")
        
        # 5️⃣ Success 보상 (+100): 목표 threshold 이내 N프레임 연속 유지
        if dist_cube_to_target < self.cfg.success_threshold:
            self.success_frames += 1
            if self.success_frames >= self.cfg.success_hold_frames:
                reward += 100.0
                print(f"  🏆 Milestone: SUCCESS! (+100.0) [{self.success_frames} frames]")
        else:
            self.success_frames = 0
        
        # ═══════════════════════════════════════════════════════════
        # 🎁 C. REWARD CLIPPING (1스텝 보상 제한)
        # ═══════════════════════════════════════════════════════════
        # 스파이크 보상 제외하고 Dense 보상만 클램핑
        if reward < 90.0:  # 큰 이벤트 보상 제외
            reward = np.clip(reward, -2.0, 2.0)
        
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
            print(f"  ⏱️ Timeout (Max steps: {self.max_steps})")
            self._record_success(False)
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


# 간단한 테스트
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
