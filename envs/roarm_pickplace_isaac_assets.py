#!/usr/bin/env python3
"""
RoArm M3 Pick and Place 환경 - Isaac Assets 활용
- YCB Dataset 물체 (캔, 박스, 병 등)
- 현실적인 테이블/배경
- Curriculum Learning 지원
"""

import numpy as np
from typing import Optional, Tuple, Dict, List
import random

# Isaac Sim 5.1 Imports
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation  # ✅ 5.1 API
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import RigidPrim, XFormPrim  # ✅ 5.1 API
from isaacsim.core.utils.prims import get_prim_at_path
import omni.kit.commands


class RoArmPickPlaceIsaacEnv:
    """
    RoArm M3 Pick and Place 강화학습 환경 (Isaac Assets 활용)
    
    Features:
    - YCB Dataset 물체 (다양한 크기/무게)
    - 현실적인 테이블/배경
    - Curriculum Learning (단계별 난이도 증가)
    - Domain Randomization (물체/위치/조명)
    """
    
    # ========== 환경 설정 ==========
    ISAAC_ASSETS_PATH = "/home/roarm_m3/isaacsim_assets/Assets/Isaac/5.0/Isaac"
    
    # YCB 물체 목록 (쉬운 것부터 어려운 순서)
    YCB_OBJECTS = {
        # Level 1: 크고 잡기 쉬운 물체
        "easy": [
            "003_cracker_box.usd",      # 크래커 박스 (16.0 x 7.7 x 21.0 cm)
            "004_sugar_box.usd",         # 설탕 박스 (9.2 x 4.5 x 17.8 cm)
        ],
        # Level 2: 중간 크기
        "medium": [
            "005_tomato_soup_can.usd",   # 토마토 수프 캔 (6.7 x 6.7 x 10.2 cm)
            "006_mustard_bottle.usd",    # 머스타드 병 (6.0 x 9.6 x 19.5 cm)
        ],
        # Level 3: 작고 어려운 물체
        "hard": [
            # 추가 물체는 필요시 확장
        ]
    }
    
    # 테이블 경로
    TABLE_USD = f"{ISAAC_ASSETS_PATH}/Props/Mounts/table.usd"
    
    # 배경 환경 (선택적)
    WAREHOUSE_USD = f"{ISAAC_ASSETS_PATH}/Environments/Simple_Warehouse/warehouse.usd"
    
    def __init__(
        self,
        urdf_path: str = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_multiprim.urdf",
        curriculum_level: str = "easy",  # "easy", "medium", "hard", "mixed"
        use_warehouse: bool = False,      # 배경 환경 사용 여부
        dt: float = 1.0 / 60.0,
        max_episode_length: int = 600,
        render: bool = True,
    ):
        """
        Args:
            urdf_path: RoArm M3 URDF 경로
            curriculum_level: 난이도 ("easy", "medium", "hard", "mixed")
            use_warehouse: 창고 배경 사용 (False면 간단한 ground plane)
            dt: 시뮬레이션 타임스텝
            max_episode_length: 최대 에피소드 길이
            render: GUI 렌더링 여부
        """
        self.urdf_path = urdf_path
        self.curriculum_level = curriculum_level
        self.use_warehouse = use_warehouse
        self.dt = dt
        self.max_episode_length = max_episode_length
        self.render_enabled = render
        
        # 상태 변수
        self.step_count = 0
        self.episode_count = 0
        
        # Isaac Sim World 초기화
        self._setup_scene()
        
        # Observation/Action Space 크기
        self.observation_dim = 15  # joint_pos(6) + gripper(2) + ee_pos(3) + obj_pos(3) + distance(1)
        self.action_dim = 8        # joint_vel(6) + gripper_vel(2)
        
        print(f"✅ RoArm Pick&Place 환경 초기화 완료")
        print(f"   - Curriculum Level: {curriculum_level}")
        print(f"   - 사용 물체: {len(self._get_object_pool())}개")
        print(f"   - 배경: {'Warehouse' if use_warehouse else 'Simple Ground'}")
    
    def _setup_scene(self):
        """Isaac Sim 씬 설정"""
        # World 생성
        self.world = World(stage_units_in_meters=1.0, physics_dt=self.dt, rendering_dt=self.dt)
        
        # 1. 배경 환경 추가
        if self.use_warehouse:
            print("   📦 Warehouse 배경 로딩 중...")
            add_reference_to_stage(
                usd_path=self.WAREHOUSE_USD,
                prim_path="/World/Warehouse"
            )
        else:
            # 간단한 Ground Plane
            self.world.scene.add_default_ground_plane()
        
        # 2. 테이블 추가 (로봇 앞쪽)
        print("   🪑 테이블 로딩 중...")
        add_reference_to_stage(
            usd_path=self.TABLE_USD,
            prim_path="/World/Table"
        )
        # 테이블 위치 조정 (로봇 기준 앞쪽 40cm, 높이 0.7m)
        table_prim = get_prim_at_path("/World/Table")
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path="/World/Table",
            new_transform_matrix=np.array([
                [1, 0, 0, 0.4],   # X: 앞쪽 40cm
                [0, 1, 0, 0],     # Y: 중앙
                [0, 0, 1, 0.7],   # Z: 높이 70cm
                [0, 0, 0, 1]
            ])
        )
        
        # 3. 로봇 Import (URDF)
        print("   🤖 RoArm M3 로딩 중...")
        from isaacsim.asset.importer.urdf import _urdf  # ✅ Isaac Sim 5.1 API
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.fix_base = True
        import_config.self_collision = False
        import_config.distance_scale = 1.0
        
        success, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=self.urdf_path,
            import_config=import_config,
            dest_path="/World/RoArm_M3",
        )
        
        if not success:
            raise RuntimeError(f"URDF import 실패: {self.urdf_path}")
        
        self.robot = self.world.scene.add(SingleArticulation(prim_path="/World/RoArm_M3", name="roarm_m3"))
        
        # 4. 물체 풀 생성 (에피소드마다 랜덤 선택)
        self.object_pool = self._create_object_pool()
        self.current_object = None  # 현재 에피소드의 물체
        
        # 5. 타겟 영역 (시각적 마커)
        self._create_target_zone()
        
        # World 리셋
        self.world.reset()
        
        print("   ✅ Scene 설정 완료!")
    
    def _get_object_pool(self) -> List[str]:
        """Curriculum level에 따른 물체 풀 반환"""
        if self.curriculum_level == "easy":
            return self.YCB_OBJECTS["easy"]
        elif self.curriculum_level == "medium":
            return self.YCB_OBJECTS["easy"] + self.YCB_OBJECTS["medium"]
        elif self.curriculum_level == "hard":
            return (self.YCB_OBJECTS["easy"] + 
                    self.YCB_OBJECTS["medium"] + 
                    self.YCB_OBJECTS["hard"])
        else:  # mixed
            all_objects = []
            for level_objects in self.YCB_OBJECTS.values():
                all_objects.extend(level_objects)
            return all_objects
    
    def _create_object_pool(self) -> List[str]:
        """YCB 물체 풀 생성"""
        object_list = self._get_object_pool()
        ycb_base_path = f"{self.ISAAC_ASSETS_PATH}/Props/YCB/Axis_Aligned_Physics"
        
        pool = []
        for obj_name in object_list:
            obj_path = f"{ycb_base_path}/{obj_name}"
            pool.append(obj_path)
        
        return pool
    
    def _spawn_random_object(self):
        """랜덤 물체 생성"""
        # 기존 물체 삭제
        if self.current_object is not None:
            omni.kit.commands.execute(
                "DeletePrims",
                paths=["/World/PickObject"]
            )
        
        # 랜덤 선택
        obj_usd = random.choice(self.object_pool)
        
        # 물체 생성 (테이블 위 랜덤 위치)
        add_reference_to_stage(
            usd_path=obj_usd,
            prim_path="/World/PickObject"
        )
        
        # 랜덤 위치 (테이블 위, 로봇 작업 범위 내)
        x = random.uniform(0.25, 0.55)  # 테이블 위 25~55cm
        y = random.uniform(-0.15, 0.15)  # 좌우 ±15cm
        z = 1.0  # 테이블 위 충분히 높게 (낙하)
        
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path="/World/PickObject",
            new_transform_matrix=np.array([
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1]
            ])
        )
        
        # RigidPrim으로 등록
        self.current_object = self.world.scene.add(
            RigidPrim(
                prim_path="/World/PickObject",
                name="pick_object"
            )
        )
    
    def _create_target_zone(self):
        """타겟 영역 생성 (시각적 마커)"""
        # 초록색 투명 실린더 (목표 위치)
        from pxr import UsdGeom, Gf
        
        stage = self.world.stage
        target_path = "/World/TargetZone"
        
        # Cylinder 생성
        cylinder_geom = UsdGeom.Cylinder.Define(stage, target_path)
        cylinder_geom.CreateRadiusAttr(0.05)   # 반지름 5cm
        cylinder_geom.CreateHeightAttr(0.01)   # 높이 1cm (얇은 디스크)
        cylinder_geom.CreateAxisAttr("Z")
        
        # 위치: 테이블 위 오른쪽
        xform = UsdGeom.Xformable(cylinder_geom)
        xform.AddTranslateOp().Set(Gf.Vec3d(0.4, 0.3, 0.71))  # 테이블 위 71cm
        
        # 초록색 Material
        # 색상 적용 (Isaac Sim 5.1 방식)
        cylinder_geom.GetDisplayColorAttr().Set([(0.0, 1.0, 0.0)])  # 초록색
        
        self.target_position = np.array([0.4, 0.3, 0.71])
    
    def reset(self) -> np.ndarray:
        """환경 리셋"""
        self.step_count = 0
        self.episode_count += 1
        
        # World 리셋
        self.world.reset()
        
        # 로봇 초기 자세 (모든 관절 0)
        zero_action = np.zeros(self.robot.num_dof)
        self.robot.set_joint_positions(zero_action)
        self.robot.set_joint_velocities(np.zeros(self.robot.num_dof))
        
        # 새 물체 생성
        self._spawn_random_object()
        
        # 물리 안정화 (1초)
        for _ in range(60):
            self.world.step(render=self.render_enabled)
        
        return self._get_observation()
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """환경 스텝"""
        self.step_count += 1
        
        # Action 적용 (속도 제어, -1~1 → 실제 속도)
        max_joint_velocity = 1.0  # rad/s
        joint_velocities = action[:6] * max_joint_velocity
        gripper_velocities = action[6:8] * 0.5  # 그리퍼는 느리게
        
        full_action = np.concatenate([joint_velocities, gripper_velocities])
        self.robot.set_joint_velocities(full_action)
        
        # 물리 시뮬레이션
        self.world.step(render=self.render_enabled)
        
        # Observation
        obs = self._get_observation()
        
        # Reward 계산
        reward = self._calculate_reward()
        
        # 종료 조건
        done, info = self._check_done()
        
        return obs, reward, done, info
    
    def _get_observation(self) -> np.ndarray:
        """관측 벡터 생성 (15차원)"""
        # 1. Joint positions (6)
        joint_positions = self.robot.get_joint_positions()[:6]
        
        # 2. Gripper state (2)
        gripper_positions = self.robot.get_joint_positions()[6:8]
        
        # 3. End-effector position (3) - 근사
        ee_pos = self._get_ee_position()
        
        # 4. Object position (3)
        obj_pos, _ = self.current_object.get_world_pose()
        obj_pos = np.array(obj_pos)
        
        # 5. Distance to target (1)
        distance = np.linalg.norm(obj_pos - self.target_position)
        
        obs = np.concatenate([
            joint_positions,
            gripper_positions,
            ee_pos,
            obj_pos,
            [distance]
        ])
        
        return obs.astype(np.float32)
    
    def _get_ee_position(self) -> np.ndarray:
        """End-effector 위치 근사 (Forward Kinematics 간단 버전)"""
        # 실제로는 FK 계산 필요, 여기서는 근사
        # 로봇 베이스 위치 + 관절 기반 추정
        base_pos, _ = self.robot.get_world_pose()
        
        # 간단한 근사: link_5 위치 사용
        # 실제 구현 시 정확한 FK 필요
        joint_pos = self.robot.get_joint_positions()[:5]
        
        # 임시 근사 (개선 필요)
        ee_approx = np.array(base_pos) + np.array([0.3, 0.0, 0.4])
        
        return ee_approx
    
    def _calculate_reward(self) -> float:
        """보상 함수"""
        obj_pos, _ = self.current_object.get_world_pose()
        obj_pos = np.array(obj_pos)
        
        # 물체-타겟 거리
        distance = np.linalg.norm(obj_pos - self.target_position)
        
        # Distance-based reward (밀집 보상)
        reward = -distance * 10.0
        
        # Success bonus
        if distance < 0.05:  # 5cm 이내
            reward += 100.0
        
        # Object 떨어짐 패널티
        if obj_pos[2] < 0.5:  # 테이블 아래로 떨어짐
            reward -= 50.0
        
        return reward
    
    def _check_done(self) -> Tuple[bool, Dict]:
        """종료 조건 확인"""
        obj_pos, _ = self.current_object.get_world_pose()
        obj_pos = np.array(obj_pos)
        
        distance = np.linalg.norm(obj_pos - self.target_position)
        
        # Success
        if distance < 0.05:
            return True, {"success": True, "reason": "task_complete"}
        
        # Timeout
        if self.step_count >= self.max_episode_length:
            return True, {"success": False, "reason": "timeout"}
        
        # Object 떨어짐
        if obj_pos[2] < 0.5:
            return True, {"success": False, "reason": "object_dropped"}
        
        return False, {}
    
    def close(self):
        """환경 종료"""
        if hasattr(self, 'world'):
            self.world.clear()


# ========== 테스트 코드 ==========
if __name__ == "__main__":
    print("=" * 60)
    print("🧪 RoArm Pick&Place (Isaac Assets) 환경 테스트")
    print("=" * 60)
    
    # 환경 생성
    env = RoArmPickPlaceIsaacEnv(
        curriculum_level="easy",
        use_warehouse=False,  # 간단한 환경으로 시작
        render=True
    )
    
    # 3 에피소드 테스트
    for ep in range(3):
        print(f"\n📍 Episode {ep + 1}")
        obs = env.reset()
        
        total_reward = 0.0
        for step in range(100):  # 짧게 테스트
            # 랜덤 액션
            action = np.random.uniform(-1, 1, env.action_dim)
            
            obs, reward, done, info = env.step(action)
            total_reward += reward
            
            if done:
                print(f"   ✅ Episode 종료: {info}")
                break
        
        print(f"   Total Reward: {total_reward:.2f}")
    
    env.close()
    print("\n✅ 테스트 완료!")
