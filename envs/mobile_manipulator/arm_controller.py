"""
Robot Arm Controller
RoArm-M3 로봇팔 제어 (모바일 베이스에 마운트)

Usage:
    from envs.mobile_manipulator import ArmController
    
    arm = ArmController(world, parent_prim="/World/Jetbot/chassis")
    arm.load()
    arm.set_joint_positions([0, 0.5, -0.8, 0.3, 0, 0.06])
"""

import numpy as np
from typing import Optional, List
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, Usd, PhysxSchema


class ArmController:
    """
    RoArm-M3 로봇팔 컨트롤러
    
    모바일 베이스에 마운트되어 동작
    """
    
    # RoArm USD 경로 (프로젝트 에셋)
    ROARM_USD_PATH = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/usd/roarm_m3_with_camera_correct.usd"
    
    def __init__(self, world, prim_path: str = "/World/RoArm", 
                 parent_prim: Optional[str] = None,
                 mount_offset: tuple = (0.0, 0.0, 0.05)):
        """
        Args:
            world: Isaac Sim World 객체
            prim_path: RoArm을 로드할 USD prim 경로
            parent_prim: 부모 prim 경로 (모바일 베이스 chassis)
            mount_offset: 부모로부터의 위치 오프셋 (x, y, z)
        """
        self.world = world
        self.prim_path = prim_path
        self.parent_prim = parent_prim
        self.mount_offset = mount_offset
        
        self._robot = None
        self._dc = None
        self._controller = None
        
        # Joint configuration
        self.num_joints = 6
        self.joint_names = [
            'base_link_to_link1',
            'link1_to_link2', 
            'link2_to_link3',
            'link3_to_link4',
            'link4_to_link5',
            'link5_to_gripper_link'
        ]
        
        # Home position
        self.home_position = np.array([0.0, -0.3, 0.8, 0.5, 0.0, 0.06], dtype=np.float32)
        
        # Joint limits
        self.joint_limits = np.array([
            [-1.57, 1.57],  # base
            [-1.57, 1.57],  # shoulder
            [-1.57, 1.57],  # elbow
            [-1.57, 1.57],  # wrist1
            [-1.57, 1.57],  # wrist2
            [0.0, 0.06]     # gripper
        ])
        
        # Controller gains
        self.kp = np.array([10000.0, 50000.0, 40000.0, 20000.0, 15000.0, 8000.0], dtype=np.float32)
        self.kd = np.array([1000.0, 5000.0, 4000.0, 2000.0, 1500.0, 800.0], dtype=np.float32)
    
    def load(self) -> bool:
        """
        RoArm USD를 씬에 로드
        
        Returns:
            bool: 로드 성공 여부
        """
        try:
            from isaacsim.core.utils.stage import add_reference_to_stage
            from isaacsim.core.prims import SingleArticulation
            
            stage = omni.usd.get_context().get_stage()
            
            # Load RoArm USD
            add_reference_to_stage(
                usd_path=self.ROARM_USD_PATH,
                prim_path=self.prim_path
            )
            
            # Set position (mount offset from parent or absolute)
            if self.parent_prim:
                # TODO: Create proper parent-child relationship
                # For now, just set position relative to expected parent location
                pass
            
            # Apply mount offset
            roarm_prim = stage.GetPrimAtPath(self.prim_path)
            if roarm_prim.IsValid():
                xformable = UsdGeom.Xformable(roarm_prim)
                xformable.ClearXformOpOrder()
                translate_op = xformable.AddTranslateOp()
                translate_op.Set(Gf.Vec3d(*self.mount_offset))
            
            # Disable USD drives (use position control instead)
            self._disable_usd_drives(stage)
            
            # Disable gravity for links
            self._disable_gravity(stage)
            
            self.world.reset()
            
            # Create articulation
            self._robot = SingleArticulation(
                prim_path=self.prim_path,
                name="roarm"
            )
            self.world.scene.add(self._robot)
            self.world.reset()
            
            # Set controller gains
            try:
                self._controller = self._robot.get_articulation_controller()
                self._controller.set_gains(self.kp, self.kd)
            except Exception as e:
                print(f"⚠️ Controller setup: {e}")
            
            # Initialize dynamic control
            try:
                from omni.isaac.dynamic_control import _dynamic_control
                self._dc = _dynamic_control.acquire_dynamic_control_interface()
            except:
                self._dc = None
            
            # Set home position
            self.go_home()
            
            print(f"✅ RoArm loaded at {self.prim_path}")
            print(f"   DOF: {self._robot.num_dof}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to load RoArm: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _disable_usd_drives(self, stage):
        """USD joint drives 비활성화"""
        for joint_name in self.joint_names:
            joint_path = f"{self.prim_path}/joints/{joint_name}"
            joint_prim = stage.GetPrimAtPath(joint_path)
            if joint_prim.IsValid():
                drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
                if drive_api:
                    drive_api.GetStiffnessAttr().Set(0.0)
                    drive_api.GetDampingAttr().Set(0.0)
    
    def _disable_gravity(self, stage):
        """로봇 링크 중력 비활성화"""
        link_names = ['base_link', 'link1', 'link2', 'link3', 'link4',
                      'link5', 'gripper_link', 'camera_link']
        for link_name in link_names:
            link_path = f"{self.prim_path}/{link_name}"
            link_prim = stage.GetPrimAtPath(link_path)
            if link_prim.IsValid():
                physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(link_prim)
                physx_api.CreateDisableGravityAttr(True)
    
    def set_joint_positions(self, positions: np.ndarray):
        """
        관절 위치 설정
        
        Args:
            positions: 6개 관절 위치 배열
        """
        if self._robot is None:
            return
        
        positions = np.array(positions, dtype=np.float32)
        
        # Clip to joint limits
        for i in range(min(len(positions), self.num_joints)):
            positions[i] = np.clip(positions[i], 
                                   self.joint_limits[i][0], 
                                   self.joint_limits[i][1])
        
        self._robot.set_joint_positions(positions)
        self._robot.set_joint_velocities(np.zeros(self.num_joints))
    
    def get_joint_positions(self) -> np.ndarray:
        """현재 관절 위치 반환"""
        if self._robot is None:
            return self.home_position.copy()
        return self._robot.get_joint_positions()
    
    def go_home(self):
        """홈 포지션으로 이동"""
        self.set_joint_positions(self.home_position)
        
        # Stabilize
        for _ in range(50):
            self.set_joint_positions(self.home_position)
            self.world.step(render=False)
    
    def get_end_effector_pos(self) -> np.ndarray:
        """
        그리퍼 월드 위치 반환
        
        Returns:
            np.ndarray: [x, y, z] 위치
        """
        if self._dc is not None:
            try:
                rigid_body = self._dc.get_rigid_body(f"{self.prim_path}/gripper_link")
                if rigid_body:
                    pose = self._dc.get_rigid_body_pose(rigid_body)
                    return np.array([pose.p.x, pose.p.y, pose.p.z])
            except:
                pass
        
        # Fallback
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(f"{self.prim_path}/gripper_link")
            if prim.IsValid():
                xformable = UsdGeom.Xformable(prim)
                world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                translation = world_transform.ExtractTranslation()
                return np.array([translation[0], translation[1], translation[2]])
        except:
            pass
        
        return np.array([0.15, 0.0, 0.2])
    
    def set_gripper(self, width: float):
        """
        그리퍼 너비 설정
        
        Args:
            width: 그리퍼 열림 정도 (0.0 = 닫힘, 0.06 = 완전 열림)
        """
        if self._robot is None:
            return
        
        positions = self.get_joint_positions()
        positions[5] = np.clip(width, 0.0, 0.06)
        self.set_joint_positions(positions)
    
    def open_gripper(self):
        """그리퍼 열기"""
        self.set_gripper(0.06)
    
    def close_gripper(self):
        """그리퍼 닫기"""
        self.set_gripper(0.0)
    
    @property
    def robot(self):
        """Articulation 객체 반환"""
        return self._robot
