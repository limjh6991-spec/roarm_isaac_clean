# Intel RealSense D405 고품질 Mesh 통합

## 📋 개요

기존의 간단한 box 형태 카메라 mesh를 Intel 공식 D405 고해상도 STL mesh로 교체했습니다.

## 🎯 변경 이유

**문제점**: "카메라 디테일이 너무 떨어짐" - 시각적 품질 저하

**해결책**: Intel RealSense 공식 저장소에서 D405 고품질 mesh (191KB STL) 다운로드 및 통합

## 📦 다운로드된 파일

### 1. Intel 공식 저장소
- **저장소**: `IntelRealSense/realsense-ros` (GitHub)
- **브랜치**: `ros2-master`
- **패키지**: `realsense2_description`

### 2. 다운로드 파일
```
/tmp/realsense-ros/realsense2_description/
├── urdf/
│   ├── _d405.urdf.xacro          # 7.2KB - 상세 URDF 정의
│   └── test_d405_camera.urdf.xacro
└── meshes/
    └── d405.stl                  # 191KB - 고해상도 3D Mesh
```

### 3. 프로젝트 통합 위치
```
~/roarm_isaac_clean/assets/roarm_m3/
├── urdf/
│   ├── _d405.urdf.xacro          # Intel 공식 URDF (참고용)
│   └── roarm_m3_with_camera_correct.urdf  # 통합 완료 ✅
└── meshes/
    └── d405.stl                  # Intel 공식 Mesh (191KB) ✅
```

## 🔧 URDF 변경 사항

### Before (간단한 Box)
```xml
<link name="camera_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <mass value="0.06" />
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.09 0.025 0.025" />
    </geometry>
    <material name="camera_material">
      <color rgba="0.15 0.15 0.15 1.0" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.09 0.025 0.025" />
    </geometry>
  </collision>
</link>
```

### After (Intel 공식 Mesh)
```xml
<link name="camera_link">
  <inertial>
    <!-- Intel RealSense D405 공식 스펙 (72g) -->
    <origin xyz="0 0 0" rpy="0 0 0" />
    <mass value="0.072" />
    <inertia ixx="0.003881243" ixy="0.0" ixz="0.0" 
             iyy="0.000498940" iyz="0.0" izz="0.003879257" />
  </inertial>
  <visual>
    <!-- Intel 공식 D405 Mesh (191KB, 고해상도) -->
    <origin xyz="0.0037 -0.009 0" rpy="1.5708 0 1.5708" />
    <geometry>
      <mesh filename="../meshes/d405.stl" scale="0.001 0.001 0.001" />
    </geometry>
    <material name="camera_material">
      <color rgba="0.2 0.2 0.2 1.0" />
    </material>
  </visual>
  <collision>
    <!-- D405 실제 치수: 42mm x 42mm x 23mm -->
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.023 0.042 0.042" />
    </geometry>
  </collision>
</link>
```

## 📊 주요 개선 사항

| 항목 | Before | After | 개선 |
|------|--------|-------|------|
| **Mesh 타입** | Box (간단) | STL (고해상도) | ✅ 시각적 품질 향상 |
| **Mesh 크기** | - | 191KB | ✅ 상세한 기하학 |
| **질량** | 60g | 72g (공식) | ✅ 정확한 물리 |
| **관성** | 간단 | 공식 Inertia | ✅ 정확한 동역학 |
| **치수** | 90x25x25mm | 42x42x23mm | ✅ 실제 크기 |

## 🎨 시각적 개선

### 1. 고해상도 Mesh
- **파일 크기**: 191KB STL (기존 box보다 훨씬 상세)
- **특징**: 렌즈, 센서, 마운트 홀 등 실제 D405 형상 재현

### 2. 정확한 배치
```xml
<!-- Mesh 회전 및 위치 조정 -->
<origin xyz="0.0037 -0.009 0" rpy="1.5708 0 1.5708" />
```
- 카메라 광학 중심 기준으로 정렬
- Intel 공식 URDF 기준 적용

### 3. 색상 조정
```xml
<color rgba="0.2 0.2 0.2 1.0" />
```
- 0.15 → 0.2 (약간 밝게 조정)
- 실제 D405의 어두운 회색 표현

## 🔬 물리 파라미터 개선

### 1. 정확한 질량
- **Before**: 60g (추정치)
- **After**: 72g (Intel 공식 스펙)

### 2. 정확한 관성 텐서
Intel 공식 `_d405.urdf.xacro`에서 가져온 값:
```
ixx = 0.003881243
iyy = 0.000498940
izz = 0.003879257
```

### 3. 실제 치수 Collision
- **Before**: 90mm × 25mm × 25mm (부정확)
- **After**: 23mm × 42mm × 42mm (D405 실제 크기)

## 📝 참고 자료

### Intel 공식 문서
- **GitHub PR**: #2953 "Added urdf & mesh files for D405 model"
- **저장소**: https://github.com/IntelRealSense/realsense-ros
- **패키지**: `realsense2_description`

### D405 스펙
```
크기: 42mm × 42mm × 23mm
질량: 72g
렌즈: Dual IR + RGB
베이스라인: 18mm (Infra1 ↔ Infra2)
```

## 🚀 사용 방법

### 1. USD 변환 (자동 완료)
```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/tools/convert_urdf.py \
  ~/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_with_camera_correct.urdf \
  ~/roarm_isaac_clean/assets/roarm_m3/usd/roarm_m3_with_camera.usd \
  --merge-joints --fix-base
```

### 2. IsaacLab 테스트
```bash
cd ~/IsaacLab
./isaaclab.sh --python ~/roarm_isaac_clean/scripts/test/test_roarm_with_camera_isaaclab.py
```

### 3. 시각적 확인
- Isaac Sim GUI에서 카메라 디테일 확인
- 렌즈, 센서 배치 등 실제 D405 형상 확인

## ⚠️ 주의사항

### 1. Mesh 경로
```xml
<mesh filename="../meshes/d405.stl" scale="0.001 0.001 0.001" />
```
- 상대 경로 사용 (URDF 위치 기준)
- Scale: 0.001 (mm → m 변환)

### 2. Mesh 회전
```xml
<origin xyz="0.0037 -0.009 0" rpy="1.5708 0 1.5708" />
```
- RPY: 90° X축, 0° Y축, 90° Z축
- 카메라 광학 축이 X축 방향

### 3. Collision 유지
- Visual: 고해상도 STL Mesh
- Collision: 간단한 Box (물리 성능)

## 📈 통합 전후 비교

| 특성 | Before | After |
|------|--------|-------|
| **시각적 품질** | ⭐⭐ (Simple Box) | ⭐⭐⭐⭐⭐ (High-Res STL) |
| **물리 정확도** | ⭐⭐⭐ (추정치) | ⭐⭐⭐⭐⭐ (공식 스펙) |
| **실제 크기** | ❌ 부정확 | ✅ 정확 |
| **파일 출처** | 사용자 제작 | Intel 공식 |

## 🎉 결론

Intel 공식 D405 고품질 Mesh 통합으로:
1. ✅ **시각적 품질 대폭 향상** (191KB 고해상도 STL)
2. ✅ **정확한 물리 파라미터** (72g, 정확한 관성)
3. ✅ **실제 크기 반영** (42mm × 42mm × 23mm)
4. ✅ **공식 소스 사용** (Intel 공식 저장소)

---

**작성일**: 2025-11-02  
**작성자**: AI Assistant  
**버전**: v1.0
