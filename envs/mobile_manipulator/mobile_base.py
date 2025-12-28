"""
Mobile Base Controller
Isaac Sim Jetbot 기반 모바일 베이스 제어

Usage:
    from envs.mobile_manipulator import MobileBase
    
    base = MobileBase(world, prim_path="/World/Jetbot")
    base.set_velocity(linear=0.5, angular=0.1)
"""

import numpy as np
from typing import Tuple, Optional
import omni.usd
from pxr import UsdGeom, Gf, Usd


class MobileBase:
    """
    Jetbot 모바일 베이스 컨트롤러
    
    Isaac Sim 내장 Jetbot USD를 사용한 차동구동 제어
    """
    
    # Clearpath Husky A200 USD with mesh (야외/농업용 4륜 로봇, 99x67cm)
    JETBOT_USD_PATH = "/home/roarm_m3/roarm_isaac_clean/assets/husky/usd/husky_mesh.usd"
    
    def __init__(self, world, prim_path: str = "/World/Jetbot"):
        """
        Args:
            world: Isaac Sim World 객체
            prim_path: Jetbot을 로드할 USD prim 경로
        """
        self.world = world
        self.prim_path = prim_path
        self._robot = None
        self._dc = None
        
        # Wheel joint names (Jetbot 기준)
        self.left_wheel_joint = "left_wheel_joint"
        self.right_wheel_joint = "right_wheel_joint"
        
        # Wheel parameters
        self.wheel_radius = 0.0325  # meters (Jetbot 휠 반지름)
        self.wheel_base = 0.1125   # meters (휠 간 거리)
        
        # Velocity limits
        self.max_linear_vel = 0.5   # m/s
        self.max_angular_vel = 2.0  # rad/s
    
    def load(self) -> bool:
        """
        Jetbot USD를 씬에 로드
        
        Returns:
            bool: 로드 성공 여부
        """
        try:
            from isaacsim.core.utils.stage import add_reference_to_stage
            from isaacsim.core.prims import SingleArticulation
            
            # Use local USD file directly
            jetbot_path = self.JETBOT_USD_PATH
            
            # Load Jetbot
            add_reference_to_stage(
                usd_path=jetbot_path,
                prim_path=self.prim_path
            )
            
            self.world.reset()
            
            # Create articulation
            self._robot = SingleArticulation(
                prim_path=self.prim_path,
                name="jetbot"
            )
            self.world.scene.add(self._robot)
            self.world.reset()
            
            # Initialize dynamic control
            try:
                from omni.isaac.dynamic_control import _dynamic_control
                self._dc = _dynamic_control.acquire_dynamic_control_interface()
            except:
                self._dc = None
            
            print(f"✅ Jetbot loaded at {self.prim_path}")
            print(f"   DOF: {self._robot.num_dof}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to load Jetbot: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def set_wheel_velocities(self, left_vel: float, right_vel: float):
        """
        휠 속도 직접 설정 (rad/s)
        
        Args:
            left_vel: 왼쪽 휠 각속도
            right_vel: 오른쪽 휠 각속도
        """
        if self._robot is None:
            return
        
        # Jetbot은 velocity control 사용
        velocities = np.array([left_vel, right_vel], dtype=np.float32)
        self._robot.set_joint_velocities(velocities)
    
    def set_velocity(self, linear: float, angular: float):
        """
        차동구동 속도 명령 (linear, angular → 휠 속도 변환)
        
        Args:
            linear: 선속도 (m/s)
            angular: 각속도 (rad/s)
        """
        # Clamp to limits
        linear = np.clip(linear, -self.max_linear_vel, self.max_linear_vel)
        angular = np.clip(angular, -self.max_angular_vel, self.max_angular_vel)
        
        # Differential drive kinematics
        # v_left = (linear - angular * wheel_base / 2) / wheel_radius
        # v_right = (linear + angular * wheel_base / 2) / wheel_radius
        left_vel = (linear - angular * self.wheel_base / 2) / self.wheel_radius
        right_vel = (linear + angular * self.wheel_base / 2) / self.wheel_radius
        
        self.set_wheel_velocities(left_vel, right_vel)
    
    def stop(self):
        """모바일 베이스 정지"""
        self.set_wheel_velocities(0.0, 0.0)
    
    def get_position(self) -> np.ndarray:
        """
        베이스 월드 위치 반환
        
        Returns:
            np.ndarray: [x, y, z] 위치
        """
        if self._dc is not None:
            try:
                rigid_body = self._dc.get_rigid_body(f"{self.prim_path}/chassis")
                if rigid_body:
                    pose = self._dc.get_rigid_body_pose(rigid_body)
                    return np.array([pose.p.x, pose.p.y, pose.p.z])
            except:
                pass
        
        # Fallback: USD API
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(f"{self.prim_path}/chassis")
            if prim.IsValid():
                xformable = UsdGeom.Xformable(prim)
                world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                translation = world_transform.ExtractTranslation()
                return np.array([translation[0], translation[1], translation[2]])
        except:
            pass
        
        return np.array([0.0, 0.0, 0.0])
    
    def get_orientation(self) -> float:
        """
        베이스 방향 (yaw) 반환
        
        Returns:
            float: yaw 각도 (radians)
        """
        # TODO: Implement proper orientation extraction
        return 0.0
    
    @property
    def robot(self):
        """Articulation 객체 반환"""
        return self._robot
