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
    success_threshold: float = 0.05      # 5cm
    time_penalty: float = 0.01           # Efficiency penalty
    
    # ═══════════════════════════════════════════════════════════
    # 📚 CURRICULUM LEARNING 설정
    # ═══════════════════════════════════════════════════════════
    curriculum_enabled: bool = True      # Curriculum 활성화
    curriculum_phase: int = 0            # 현재 Phase (0: Easy, 1: Normal)
    
    # Phase 0: Easy Mode (가까운 거리)
    easy_cube_distance: Tuple[float, float] = (0.10, 0.15)  # 10~15cm
    easy_target_distance: Tuple[float, float] = (0.20, 0.25)  # 20~25cm
    
    # Phase 1: Normal Mode (원래 거리)
    normal_cube_distance: Tuple[float, float] = (0.25, 0.35)  # 25~35cm
    normal_target_distance: Tuple[float, float] = (0.25, 0.35)  # 25~35cm
    
    # 자동 승급 조건
    success_rate_window: int = 200       # 최근 200 에피소드
    success_rate_threshold: float = 0.60  # 성공률 60% 이상


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
        
        # Observation/Action space 정의 (Dense Reward: 더 많은 정보)
        self.observation_space_dim = 25  # joint(8) + ee(3) + cube(3) + target(3) + ee2cube(3) + cube2target(3) + gripper_width(1) + is_grasped(1)
        self.action_space_dim = 8
        
        # 이전 상태 저장 (보상 계산용)
        self.prev_ee_to_cube_dist = None
        self.prev_cube_to_target_dist = None
        
        # ═══════════════════════════════════════════════════════════
        # 🎯 SHAPED-SPARSE: 1회성 이벤트 플래그
        # ═══════════════════════════════════════════════════════════
        self.first_reach = False      # EE가 큐브에 처음 근접 (5cm)
        self.valid_grip = False       # 유효한 그립 달성 (3프레임)
        self.lifted = False           # 큐브를 들어올림 (5cm)
        self.goal_near = False        # 큐브가 목표에 근접 (8cm)
        
        # m 프레임 히스테리시스 카운터
        self.grip_frames = 0          # 연속 그립 프레임
        self.success_frames = 0       # 연속 성공 프레임
        
        # ═══════════════════════════════════════════════════════════
        # 📚 CURRICULUM: 성공률 추적
        # ═══════════════════════════════════════════════════════════
        self.episode_successes = []   # 최근 에피소드 성공 여부
        
        print(f"\n📊 환경 정보:")
        print(f"  - Observation dim: {self.observation_space_dim}")
        print(f"  - Action dim: {self.action_space_dim}")
        print(f"  - Max steps: {self.max_steps}")
        print(f"  - Success threshold: {self.cfg.success_threshold}m")
        print(f"  - Reward type: Shaped-Sparse (게이팅 + 1회성 이벤트)")
        print(f"  - Curriculum: Phase {self.cfg.curriculum_phase} ({'Easy' if self.cfg.curriculum_phase == 0 else 'Normal'})")
    
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
        
        # 초기 observation 반환
        return self._get_observation()
    
    def _get_observation(self) -> np.ndarray:
        """현재 상태 관측 (Dense Reward: 더 많은 정보)"""
        # Joint positions (6 revolute + 2 prismatic)
        joint_positions = self.robot.get_joint_positions()[:8]
        
        # End-effector position
        ee_pos = self._get_ee_position()
        
        # Cube position
        cube_pos, _ = self.cube.get_world_pose()
        
        # Target position
        target_pos = np.array(self.cfg.target_position)
        
        # EE → Cube vector
        ee_to_cube = cube_pos - ee_pos
        
        # Cube → Target vector
        cube_to_target = target_pos - cube_pos
        
        # Gripper width (distance between fingers)
        gripper_width = joint_positions[6] + joint_positions[7]
        
        # Is grasped (간단한 휴리스틱: EE가 큐브 가까이 + 그리퍼 닫힘)
        ee_to_cube_dist = np.linalg.norm(ee_to_cube)
        is_grasped = 1.0 if (ee_to_cube_dist < 0.08 and gripper_width < 0.02) else 0.0
        
        # Observation 벡터 생성 (25 dim)
        obs = np.concatenate([
            joint_positions[:6],      # Joint positions (6)
            joint_positions[6:8],     # Gripper state (2)
            ee_pos,                   # EE position (3)
            cube_pos,                 # Cube position (3)
            target_pos,               # Target position (3)
            ee_to_cube,               # EE → Cube vector (3)
            cube_to_target,           # Cube → Target vector (3)
            [gripper_width],          # Gripper width (1)
            [is_grasped],             # Is grasped (1)
        ])
        
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
        
        # Physics 시뮬레이션 스텝
        self.world.step(render=True)
        
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
        cube_to_target_dist = np.linalg.norm(obs[20:23])
        
        # 성공률 계산 (최근 에피소드 기준)
        success_rate = np.mean(self.episode_successes) if len(self.episode_successes) > 0 else 0.0
        
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
        🎯 SHAPED-SPARSE REWARD: 게이팅 + 1회성 마일스톤 보상
        
        전문가 최종 권장 사항 적용:
        1. 게이팅 시스템: grasp_valid 체크로 보상 폭발 차단
        2. 1회성 이벤트: 각 마일스톤은 한 번만 보상
        3. m 프레임 히스테리시스: 노이즈 필터링
        
        보상 구조:
        - 근접 (+5): EE가 큐브 5cm 이내 진입 (1회)
        - 그립 (+10): 유효 그립 3프레임 유지 (1회, 게이팅)
        - 리프트 (+15): 큐브 5cm 이상 들어올림 (1회, 게이팅)
        - 목표 근접 (+20): 큐브가 목표 8cm 이내 (1회, 게이팅)
        - Success (+100): 목표 5cm 이내 5프레임 유지 (1회)
        - Time penalty (-0.01): 효율성 유도
        """
        # 관찰에서 필요한 값 추출
        ee_pos = obs[8:11]
        cube_pos = obs[11:14]
        target_pos = obs[14:17]
        ee_to_cube_vec = obs[17:20]
        cube_to_target_vec = obs[20:23]
        gripper_width = obs[23]
        is_grasped = obs[24]
        
        # 거리 계산
        ee_to_cube_dist = np.linalg.norm(ee_to_cube_vec)
        cube_to_target_dist = np.linalg.norm(cube_to_target_vec)
        
        # ═══════════════════════════════════════════════════════════
        # 🔒 GATING: grasp_valid 체크
        # ═══════════════════════════════════════════════════════════
        # 유효 그립 조건: EE 근접 + 그리퍼 닫힘 + 큐브 높이
        grasp_valid = (
            ee_to_cube_dist < 0.08 and      # EE가 8cm 이내
            gripper_width < 0.02 and        # 그리퍼가 닫힘
            cube_pos[2] > 0.03              # 큐브가 바닥 위
        )
        
        # 히스테리시스: 연속 프레임 카운트
        if grasp_valid:
            self.grip_frames += 1
        else:
            self.grip_frames = 0
        
        # ═══════════════════════════════════════════════════════════
        # 🎁 SHAPED-SPARSE REWARDS (1회성 이벤트)
        # ═══════════════════════════════════════════════════════════
        reward = 0.0
        
        # 1️⃣ 근접 보상 (+5): EE가 큐브에 처음 근접
        if not self.first_reach and ee_to_cube_dist < 0.05:
            reward += 5.0
            self.first_reach = True
            print(f"  🎯 Milestone: REACH! (+5.0)")
        
        # 2️⃣ 그립 보상 (+10): 유효 그립 3프레임 유지 [게이팅]
        if not self.valid_grip and grasp_valid and self.grip_frames >= 3:
            reward += 10.0
            self.valid_grip = True
            print(f"  ✊ Milestone: GRIP! (+10.0)")
        
        # 3️⃣ 리프트 보상 (+15): 큐브 5cm 이상 들어올림 [게이팅]
        if not self.lifted and grasp_valid and cube_pos[2] > 0.05:
            reward += 15.0
            self.lifted = True
            print(f"  ⬆️ Milestone: LIFT! (+15.0)")
        
        # 4️⃣ 목표 근접 보상 (+20): 큐브가 목표 8cm 이내 [게이팅]
        if not self.goal_near and grasp_valid and cube_to_target_dist < 0.08:
            reward += 20.0
            self.goal_near = True
            print(f"  🎯 Milestone: GOAL NEAR! (+20.0)")
        
        # 5️⃣ Success 보상 (+100): 목표 5cm 이내 5프레임 유지
        if cube_to_target_dist < self.cfg.success_threshold:
            self.success_frames += 1
            if self.success_frames >= 5:
                reward += self.cfg.success_reward  # +100
                print(f"  🏆 Milestone: SUCCESS! (+100.0)")
        else:
            self.success_frames = 0
        
        # Time Penalty: 효율성 유도 (매 스텝 -0.01)
        time_penalty = -self.cfg.time_penalty
        reward += time_penalty
        
        return reward
    
    def _check_done(self, obs: np.ndarray) -> bool:
        """에피소드 종료 조건 + 성공률 추적"""
        # 큐브 위치
        cube_pos = obs[11:14]
        
        # Cube → Target 거리
        cube_to_target_vec = obs[20:23]
        cube_to_target_dist = np.linalg.norm(cube_to_target_vec)
        
        # 타겟 도달
        if cube_to_target_dist < self.cfg.success_threshold:
            print(f"  ✅ SUCCESS! Distance: {cube_to_target_dist:.3f}m")
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
