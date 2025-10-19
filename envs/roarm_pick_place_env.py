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
    
    # Reward settings
    distance_reward_scale: float = 10.0
    success_reward: float = 100.0
    success_threshold: float = 0.05  # 5cm


class RoArmPickPlaceEnv:
    """
    RoArm-M3 Pick and Place 강화학습 환경
    
    Task: 큐브를 집어서 타겟 위치로 옮기기
    
    Observation Space (15 dim):
        - Joint positions (6 dim): joint_1 ~ joint_6
        - Gripper state (2 dim): left_finger, right_finger positions
        - End-effector position (3 dim): x, y, z
        - Object position (3 dim): x, y, z
        - Object-target distance (1 dim)
    
    Action Space (8 dim):
        - Joint velocities (6 dim): joint_1 ~ joint_6
        - Gripper velocities (2 dim): left_finger, right_finger
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
        
        # Observation/Action space 정의
        self.observation_space_dim = 15
        self.action_space_dim = 8
        
        print(f"\n📊 환경 정보:")
        print(f"  - Observation dim: {self.observation_space_dim}")
        print(f"  - Action dim: {self.action_space_dim}")
        print(f"  - Max steps: {self.max_steps}")
        print(f"  - Success threshold: {self.cfg.success_threshold}m")
    
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
        
        # Joint 이름 가져오기
        self.joint_names = self.robot.dof_names
        print(f"  ✅ Joints: {self.joint_names}")
    
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
        
        # 로봇 초기 자세 (Home position)
        home_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 8 DOF
        self.robot.set_joint_positions(home_positions)
        self.robot.set_joint_velocities(np.zeros(8))
        
        # 큐브 위치 랜덤화 (약간의 변화)
        cube_pos = np.array(self.cfg.object_position)
        cube_pos[:2] += np.random.uniform(-0.05, 0.05, size=2)  # X, Y ±5cm
        self.cube.set_world_pose(position=cube_pos)
        self.cube.set_linear_velocity(np.zeros(3))
        self.cube.set_angular_velocity(np.zeros(3))
        
        # 스텝 카운터 초기화
        self.current_step = 0
        
        # 초기 observation 반환
        return self._get_observation()
    
    def _get_observation(self) -> np.ndarray:
        """현재 상태 관측"""
        # Joint positions (6 revolute + 2 prismatic)
        joint_positions = self.robot.get_joint_positions()[:8]
        
        # End-effector position (gripper_base 링크)
        # 간단히 forward kinematics로 계산 (여기서는 근사)
        ee_pos = self._get_ee_position()
        
        # Cube position
        cube_pos, _ = self.cube.get_world_pose()
        
        # Object-target distance
        target_pos = np.array(self.cfg.target_position)
        distance = np.linalg.norm(cube_pos - target_pos)
        
        # Observation 벡터 생성 (15 dim)
        obs = np.concatenate([
            joint_positions[:6],      # Joint positions (6)
            joint_positions[6:8],     # Gripper state (2)
            ee_pos,                   # EE position (3)
            cube_pos,                 # Object position (3)
            [distance],               # Distance (1)
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
        # Action: joint velocities (8 dim)
        action = np.clip(action, -1.0, 1.0)  # [-1, 1] 범위로 제한
        
        # 속도 스케일링
        max_velocities = np.array([2.0, 1.5, 2.0, 2.5, 3.0, 2.0, 0.05, 0.05])
        joint_velocities = action * max_velocities
        
        # 로봇에 속도 명령 전송
        self.robot.set_joint_velocities(joint_velocities)
        
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
        
        # Info 딕셔너리
        info = {
            "step": self.current_step,
            "cube_position": obs[9:12],
            "distance_to_target": obs[14],
        }
        
        return obs, reward, done, info
    
    def _calculate_reward(self, obs: np.ndarray) -> float:
        """보상 함수"""
        # Object-target distance
        distance = obs[14]
        
        # Distance-based reward (거리가 가까울수록 높은 보상)
        distance_reward = -distance * self.cfg.distance_reward_scale
        
        # Success bonus (타겟에 도달하면)
        success_bonus = 0.0
        if distance < self.cfg.success_threshold:
            success_bonus = self.cfg.success_reward
        
        total_reward = distance_reward + success_bonus
        
        return total_reward
    
    def _check_done(self, obs: np.ndarray) -> bool:
        """에피소드 종료 조건"""
        # 타겟 도달
        distance = obs[14]
        if distance < self.cfg.success_threshold:
            print(f"  ✅ SUCCESS! Distance: {distance:.3f}m")
            return True
        
        # 최대 스텝 도달
        if self.current_step >= self.max_steps:
            print(f"  ⏱️ Timeout (Max steps: {self.max_steps})")
            return True
        
        # 물체가 테이블 밖으로 떨어짐 (Z < 0)
        cube_z = obs[11]
        if cube_z < -0.1:
            print(f"  ❌ Cube fell off table (Z: {cube_z:.3f}m)")
            return True
        
        return False
    
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
