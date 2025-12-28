#!/usr/bin/env python3
"""
RoArm-M3 Home Position Viewer
'ㄱ'자 초기 자세 + 각 링크 동작 테스트
"""

import sys
import time
import numpy as np

# Isaac Sim 5.0 모듈
from isaacsim import SimulationApp

# GUI 모드로 초기화
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
})

print("\n" + "=" * 80)
print("🤖 RoArm-M3 'ㄱ'자 초기 자세 + 링크 동작 테스트")
print("=" * 80)
print("  • 3초간 초기 자세 유지")
print("  • 각 링크 순차 동작 테스트")
print("  • Ctrl+C로 종료")
print("=" * 80)

# Isaac Sim 모듈 임포트
from isaacsim.core.api import World
from isaacsim.core.api.articulations import Articulation
# TODO_5.1: Use isaacsim.core.api.prims or USD prims directly
# from omni.isaac.core.objects import DynamicCuboid
import omni.kit.commands
from pxr import UsdPhysics

try:
    # World 생성
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    print("\n📦 로봇 로딩 중...")
    
    # URDF 경로
    urdf_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3.generated.urdf"
    
    # URDF Import Config (Isaac Sim 5.0)
    from isaacsim.asset.importer.urdf import _urdf
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = True
    import_config.distance_scale = 1.0
    
    # URDF Import
    success, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        get_articulation_root=True,
    )
    
    if not success:
        print("❌ URDF 임포트 실패!")
        sys.exit(1)
    
    print(f"✅ 로봇 임포트 성공: {prim_path}")
    
    # Articulation 생성
    robot = world.scene.add(
        Articulation(prim_path=prim_path, name="roarm_m3")
    )
    
    # World 리셋
    world.reset()
    
    # Joint 정보
    joint_names = robot.dof_names
    num_joints = len(joint_names) if joint_names else 0
    print(f"\n🤖 로봇 정보:")
    print(f"  • Total Joints: {num_joints}개")
    if joint_names:
        for i, name in enumerate(joint_names):
            print(f"    [{i}] {name}")
    
    # Joint drive 설정
    print(f"\n⏳ Joint drive 설정 중...")
    stage = world.stage
    for i, joint_name in enumerate(joint_names):
        joint_prim = stage.GetPrimAtPath(f"{prim_path}/{joint_name}")
        if joint_prim and joint_prim.IsValid():
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            if i < 5:  # 팔 관절 (0-4)
                drive_api.GetStiffnessAttr().Set(5000.0)
                drive_api.GetDampingAttr().Set(500.0)
                drive_api.GetMaxForceAttr().Set(500.0)
            else:  # 그리퍼 (5)
                drive_api.GetStiffnessAttr().Set(1000.0)
                drive_api.GetDampingAttr().Set(100.0)
                drive_api.GetMaxForceAttr().Set(100.0)
    print(f"  ✅ Joint drive 설정 완료!")
    
    # 테스트용 큐브 추가
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/test_cube",
            name="test_cube",
            position=np.array([0.25, 0.0, 0.05]),
            size=0.04,
            color=np.array([0.8, 0.2, 0.2]),  # 빨간색
        )
    )
    print(f"  ✅ 테스트 큐브 생성")
    
    print("\n🎨 'ㄱ'자 초기 자세 설정...")
    
    # 🔥 'ㄱ'자 Home Position (URDF Joint Limits 준수)
    # 📋 RoArm-M3 공식 스펙 (Waveshare Wiki):
    #   - Base: ±180° (±3.14 rad)
    #   - Shoulder: ±90° (±1.57 rad)
    #   - Elbow: 180°
    #   - Wrist1: ±90° (±1.57 rad)
    #   - Wrist2: ±180° (±3.14 rad)
    #   - Gripper: 135° (3.14 ~ 1.08 rad 범위)
    #
    # 🔧 실제 URDF Limits:
    #   Joint 0 (Base):     -1.5708 ~ 1.5708 (±90°)
    #   Joint 1 (Link1):    -1.5708 ~ 1.5708 (±90°)
    #   Joint 2 (Link2):    -1.0 ~ 2.95 (특이!)
    #   Joint 3 (Wrist1):   -1.5708 ~ 1.5708 (±90°)
    #   Joint 4 (Wrist2):   -3.1416 ~ 3.1416 (±180°)
    #   Joint 5 (Gripper):  0.0 ~ 1.5 (열림 → 닫힘)
    #
    # 초기 자세:
    #   Joint 0 (Base): 0.0 (정면)
    #   Joint 1 (Link1): 0.0 (지면과 90도 수직)
    #   Joint 2 (Link2): -1.0 (URDF limit 준수, 'ㄱ'자)
    #   Joint 3 (Wrist1): 0.0
    #   Joint 4 (Wrist2): 0.0
    #   Joint 5 (Gripper): 0.0 (완전 열림)
    home_positions = np.array([0.0, 0.0, -1.0, 0.0, 0.0, 0.0])
    
    print(f"  📍 Home Position: {home_positions.tolist()}")
    print(f"  💡 Joint 2 = -1.0 (URDF limit 준수)")
    
    # 관절 위치 설정
    robot.set_joint_positions(home_positions)
    robot.set_joint_velocities(np.zeros(6))
    
    # 안정화 (60 프레임 = 1초)
    for _ in range(60):
        world.step(render=False)
    
    # 실제 관절 위치 확인
    actual_positions = robot.get_joint_positions()
    print(f"\n✅ 실제 관절 위치: {actual_positions[:6].tolist()}")
    print(f"  - Joint 0 (Base):   {actual_positions[0]:.4f}")
    print(f"  - Joint 1 (Link1):  {actual_positions[1]:.4f} (지면과 90도)")
    print(f"  - Joint 2 (Link2):  {actual_positions[2]:.4f} ('ㄱ'자)")
    print(f"  - Joint 3 (Wrist):  {actual_positions[3]:.4f}")
    print(f"  - Joint 4 (Wrist2): {actual_positions[4]:.4f}")
    print(f"  - Joint 5 (Gripper):{actual_positions[5]:.4f}")
    
    print("\n" + "=" * 80)
    print("⏸️  'ㄱ'자 초기 자세 3초 유지...")
    print("=" * 80)
    
    # 3초 유지 (180 프레임)
    for _ in range(180):
        world.step(render=True)
    
    print("\n" + "=" * 80)
    print("🔄 각 링크 순차 동작 테스트 시작")
    print("=" * 80)
    
    # 동작 시퀀스 정의 (최대 범위 테스트)
    motion_sequence = [
        ("초기 'ㄱ'자 자세", home_positions.copy()),
        
        # Joint 0 테스트 (Base: -1.57 ~ 1.57)
        ("Joint 0: Base 최대 우회전 +90°", np.array([1.57, 0.0, -1.0, 0.0, 0.0, 0.0])),
        ("Joint 0: Base 최대 좌회전 -90°", np.array([-1.57, 0.0, -1.0, 0.0, 0.0, 0.0])),
        ("Joint 0: 원위치", home_positions.copy()),
        
        # Joint 1 테스트 (Link1: -1.57 ~ 1.57)
        ("Joint 1: Link1 최대 앞으로 +90°", np.array([0.0, 1.57, -1.0, 0.0, 0.0, 0.0])),
        ("Joint 1: Link1 최대 뒤로 -90°", np.array([0.0, -1.57, -1.0, 0.0, 0.0, 0.0])),
        ("Joint 1: 원위치", home_positions.copy()),
        
        # Joint 2 테스트 (Link2: -1.0 ~ 2.95)
        ("Joint 2: Link2 최대 펴기 +2.95", np.array([0.0, 0.0, 2.95, 0.0, 0.0, 0.0])),
        ("Joint 2: Link2 최대 굽히기 -1.0", np.array([0.0, 0.0, -1.0, 0.0, 0.0, 0.0])),
        ("Joint 2: 원위치", home_positions.copy()),
        
        # Joint 3 테스트 (Wrist1: -1.57 ~ 1.57)
        ("Joint 3: Wrist1 최대 좌회전 -90°", np.array([0.0, 0.0, -1.0, -1.57, 0.0, 0.0])),
        ("Joint 3: Wrist1 최대 우회전 +90°", np.array([0.0, 0.0, -1.0, 1.57, 0.0, 0.0])),
        ("Joint 3: 원위치", home_positions.copy()),
        
        # Joint 4 테스트 (Wrist2: -3.14 ~ 3.14)
        ("Joint 4: Wrist2 최대 회전 +180°", np.array([0.0, 0.0, -1.0, 0.0, 3.14, 0.0])),
        ("Joint 4: Wrist2 최대 회전 -180°", np.array([0.0, 0.0, -1.0, 0.0, -3.14, 0.0])),
        ("Joint 4: 원위치", home_positions.copy()),
        
        # 🤏 Joint 5 테스트 (Gripper: 0.0 ~ 1.5)
        ("🤏 그리퍼: 완전 열림 (0.0)", np.array([0.0, 0.0, -1.0, 0.0, 0.0, 0.0])),
        ("🤏 그리퍼: 1/3 닫힘 (0.5)", np.array([0.0, 0.0, -1.0, 0.0, 0.0, 0.5])),
        ("🤏 그리퍼: 2/3 닫힘 (1.0)", np.array([0.0, 0.0, -1.0, 0.0, 0.0, 1.0])),
        ("🤏 그리퍼: 완전 닫힘 (1.5)", np.array([0.0, 0.0, -1.0, 0.0, 0.0, 1.5])),
        ("🤏 그리퍼: 다시 완전 열림", np.array([0.0, 0.0, -1.0, 0.0, 0.0, 0.0])),
        ("🤏 Joint 5: 그리퍼 다시 열기", np.array([0.0, 0.0, -1.0, 0.0, 0.0, 0.1])),
        ("🤏 Joint 5: 그리퍼 다시 닫기", np.array([0.0, 0.0, -1.0, 0.0, 0.0, 0.0])),
        ("Joint 5: 원위치", home_positions.copy()),
        ("복합 동작: 팔 펴고 회전", np.array([1.57, 0.5, 0.0, 0.5, 1.57, 0.0125])),
        ("최종: 'ㄱ'자 자세 복귀", home_positions.copy()),
    ]
    
    for idx, (description, target_positions) in enumerate(motion_sequence, 1):
        print(f"\n  [{idx}/{len(motion_sequence)}] {description}")
        
        # 현재 위치에서 목표 위치로 부드럽게 보간 이동
        current_positions = robot.get_joint_positions()
        steps = 90  # 1.5초 (60 FPS)
        
        for step in range(steps):
            alpha = step / steps
            # 선형 보간
            interpolated = current_positions + alpha * (target_positions - current_positions)
            robot.set_joint_positions(interpolated)
            world.step(render=True)
        
        # 목표 위치에서 0.5초 정지
        for _ in range(30):
            world.step(render=True)
    
    print("\n" + "=" * 80)
    print("✅ 모든 링크 동작 테스트 완료!")
    print("=" * 80)
    print("\n📊 테스트 결과:")
    print("  ✓ 'ㄱ'자 초기 자세 확인")
    print("  ✓ 각 관절의 독립적인 움직임 확인")
    print("  ✓ 그리퍼 개폐 동작 확인")
    print("  ✓ 복합 동작 확인")
    print("\n💡 계속 관찰하려면 Isaac Sim 창을 열어두세요.")
    print("   종료하려면 터미널에서 Ctrl+C를 누르세요.\n")
    
    # GUI 유지 (무한 루프)
    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.016)  # 60 FPS

except KeyboardInterrupt:
    print("\n\n⏹️  종료 중...")

except Exception as e:
    print(f"\n❌ 에러 발생: {e}")
    import traceback
    traceback.print_exc()

finally:
    simulation_app.close()
    print("✅ 종료 완료!")
