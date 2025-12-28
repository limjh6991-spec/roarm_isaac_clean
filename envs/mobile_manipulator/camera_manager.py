"""
Camera Manager
Hand-Eye 카메라 관리 및 이미지 처리

Usage:
    from envs.mobile_manipulator import CameraManager
    
    camera = CameraManager(stage, parent_prim="/World/RoArm/gripper_link")
    camera.setup()
    image = camera.get_image()
    red_ratio = camera.detect_red()
"""

import numpy as np
from typing import Optional, Tuple
import omni.usd
from pxr import UsdGeom, Gf


class CameraManager:
    """
    Hand-Eye 카메라 매니저
    
    그리퍼에 마운트된 카메라로 큐브 감지
    """
    
    def __init__(self, stage, 
                 parent_prim: str = "/World/RoArm/gripper_link",
                 camera_name: str = "hand_camera",
                 resolution: tuple = (84, 84)):
        """
        Args:
            stage: USD stage
            parent_prim: 카메라를 마운트할 부모 prim 경로
            camera_name: 카메라 이름
            resolution: 이미지 해상도 (width, height)
        """
        self.stage = stage
        self.parent_prim = parent_prim
        self.camera_name = camera_name
        self.resolution = resolution
        
        self.camera_prim_path = f"{parent_prim}/{camera_name}"
        self._camera_prim = None
        self._render_product = None
        self._rgb_annot = None
        
        # Camera settings
        self.focal_length = 18.0
        self.horizontal_aperture = 20.955
        self.vertical_aperture = 20.955
        self.clipping_range = (0.01, 10.0)
        
        # Camera mount offset and rotation
        self.mount_offset = (0.0, 0.0, 0.06)  # 6cm above gripper
        self.mount_rotation = (-90.0, 0.0, 180.0)  # Look down
    
    def setup(self, offset: tuple = None, rotation: tuple = None) -> bool:
        """
        카메라 설정
        
        Args:
            offset: 위치 오프셋 (x, y, z) - None이면 기본값 사용
            rotation: 회전 (rx, ry, rz) degrees - None이면 기본값 사용
            
        Returns:
            bool: 설정 성공 여부
        """
        try:
            import omni.replicator.core as rep
            
            if offset:
                self.mount_offset = offset
            if rotation:
                self.mount_rotation = rotation
            
            # Remove existing camera if any
            existing = self.stage.GetPrimAtPath(self.camera_prim_path)
            if existing.IsValid():
                self.stage.RemovePrim(self.camera_prim_path)
            
            # Create camera prim
            self._camera_prim = self.stage.DefinePrim(self.camera_prim_path, "Camera")
            camera = UsdGeom.Camera(self._camera_prim)
            
            # Set camera properties
            camera.GetFocalLengthAttr().Set(self.focal_length)
            camera.GetHorizontalApertureAttr().Set(self.horizontal_aperture)
            camera.GetVerticalApertureAttr().Set(self.vertical_aperture)
            camera.GetClippingRangeAttr().Set(Gf.Vec2f(*self.clipping_range))
            
            # Apply transforms
            xformable = UsdGeom.Xformable(self._camera_prim)
            xformable.ClearXformOpOrder()
            
            # Translate
            translate_op = xformable.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(*self.mount_offset))
            
            # Rotate
            rotate_op = xformable.AddRotateXYZOp()
            rotate_op.Set(Gf.Vec3f(*self.mount_rotation))
            
            # Create render product
            self._render_product = rep.create.render_product(
                self.camera_prim_path, 
                self.resolution
            )
            
            # Create RGB annotator
            self._rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
            self._rgb_annot.attach([self._render_product])
            
            print(f"✅ Camera setup at {self.camera_prim_path}")
            print(f"   Resolution: {self.resolution}")
            print(f"   Offset: {self.mount_offset}, Rotation: {self.mount_rotation}")
            return True
            
        except Exception as e:
            print(f"❌ Camera setup failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def get_image(self) -> Optional[np.ndarray]:
        """
        현재 카메라 이미지 반환
        
        Returns:
            np.ndarray: RGB 이미지 (H, W, 3) 또는 None
        """
        try:
            if self._rgb_annot is not None:
                rgb_data = self._rgb_annot.get_data()
                if rgb_data is not None and len(rgb_data.shape) == 3:
                    return rgb_data[:, :, :3]
        except:
            pass
        return None
    
    def get_normalized_image(self) -> np.ndarray:
        """
        Isaac Lab 스타일 정규화된 이미지 반환
        - /255 정규화
        - 평균 빼기
        - CHW 포맷
        
        Returns:
            np.ndarray: 정규화된 이미지 (C, H, W)
        """
        image = self.get_image()
        if image is None:
            return np.zeros((3, self.resolution[1], self.resolution[0]), dtype=np.float32)
        
        # Normalize
        image_norm = image.astype(np.float32) / 255.0
        image_norm -= np.mean(image_norm, axis=(0, 1), keepdims=True)
        
        # HWC -> CHW
        image_chw = np.transpose(image_norm, (2, 0, 1))
        return image_chw
    
    def detect_red(self) -> Tuple[float, Optional[Tuple[float, float]]]:
        """
        이미지에서 빨간색 객체 감지
        
        Returns:
            Tuple[float, Optional[Tuple]]: (빨간 픽셀 비율 %, 중심 좌표 또는 None)
        """
        image = self.get_image()
        if image is None:
            return 0.0, None
        
        # Red detection (R > 150, G < 100, B < 100)
        red_mask = (image[:, :, 0] > 150) & (image[:, :, 1] < 100) & (image[:, :, 2] < 100)
        red_ratio = np.sum(red_mask) / (image.shape[0] * image.shape[1]) * 100
        
        # Find center
        if np.sum(red_mask) > 0:
            y_coords, x_coords = np.where(red_mask)
            center = (float(np.mean(x_coords)), float(np.mean(y_coords)))
        else:
            center = None
        
        return red_ratio, center
    
    def is_cube_visible(self, threshold: float = 0.5) -> bool:
        """
        큐브가 보이는지 확인
        
        Args:
            threshold: 빨간 픽셀 비율 임계값 (%)
            
        Returns:
            bool: 큐브 가시 여부
        """
        red_ratio, _ = self.detect_red()
        return red_ratio >= threshold
    
    def save_image(self, filepath: str) -> bool:
        """
        현재 이미지를 파일로 저장
        
        Args:
            filepath: 저장할 파일 경로
            
        Returns:
            bool: 저장 성공 여부
        """
        try:
            from PIL import Image
            
            image = self.get_image()
            if image is not None:
                img = Image.fromarray(image.astype(np.uint8))
                img.save(filepath)
                return True
        except:
            pass
        return False
    
    def update_rotation(self, rotation: tuple):
        """
        카메라 회전 업데이트
        
        Args:
            rotation: 새 회전 (rx, ry, rz) degrees
        """
        if self._camera_prim is None:
            return
        
        self.mount_rotation = rotation
        
        xformable = UsdGeom.Xformable(self._camera_prim)
        xformable.ClearXformOpOrder()
        
        translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*self.mount_offset))
        
        rotate_op = xformable.AddRotateXYZOp()
        rotate_op.Set(Gf.Vec3f(*self.mount_rotation))
