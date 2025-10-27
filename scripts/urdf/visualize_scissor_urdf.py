#!/home/roarm_m3/isaacsim/python.sh
"""
RoArm-M3 Scissor Gripper URDF Visualization Script
가위형 그리퍼 구조 확인용
"""

import time
import numpy as np
from isaacsim import SimulationApp

# Isaac Sim 초기화
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import omni.kit.commands

def main():
    """Scissor Gripper URDF 시각화"""
    
    # World 생성
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    print("\n" + "="*60)
    print("RoArm-M3 Scissor Gripper URDF Visualization")
    print("="*60)
    
    # URDF 파일 경로
    urdf_path = "/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_rl_urdf_scissor.urdf"
    
    print(f"\n📄 URDF 파일: {urdf_path}")
    print("🔧 Import 설정: merge_fixed_joints=False, fix_base=True")
    
    # URDF Import 설정
    from isaacsim.asset.importer.urdf import _urdf
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = True
    import_config.distance_scale = 1.0
    
    # URDF 임포트
    print("\n⏳ URDF 임포트 중...")
    success, robot_prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        get_articulation_root=True,
    )
    
    if not success:
        print("❌ URDF 임포트 실패!")
        simulation_app.close()
        return
    
    print(f"✅ URDF 임포트 성공: {robot_prim_path}")
    
    # World 초기화
    world.reset()
    
    # Articulation 생성
    robot = world.scene.add(
        Articulation(
            prim_path=robot_prim_path,
            name="roarm_scissor"
        )
    )
    
    # 초기화
    robot.initialize()
    
    # Joint 정보 출력
    dof_names = robot.dof_names
    num_dof = robot.num_dof
    
    print(f"\n🤖 Robot DOF: {num_dof}")
    print("📋 Joint 목록:")
    for i, name in enumerate(dof_names):
        print(f"  [{i}] {name}")
    
    # 큐브 추가 (참고용)
    from omni.isaac.core.objects import DynamicCuboid
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/Cube",
            name="test_cube",
            position=np.array([0.25, 0.0, 0.03]),
            size=0.03,  # float으로 변경
            color=np.array([1.0, 0.2, 0.2])
        )
    )
    
    print("\n📦 테스트 큐브 배치: [0.25, 0.0, 0.03]")
    
    # 테스트 시퀀스 정의 (7 DOF: 6개 팔 관절 + 1개 좌측 그리퍼, 우측은 mimic)
    test_sequence = [
        ("🏠 초기 자세", [0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0]),
        ("✋ 그리퍼 완전히 열기", [0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.52]),
        ("➡️  큐브로 접근", [0.0, -0.2, 0.2, -0.1, 0.0, 0.0, 0.52]),
        ("✊ 그리퍼 닫기 (파지!)", [0.0, -0.2, 0.2, -0.1, 0.0, 0.0, 0.0]),
        ("⬆️  들어올리기 (Lift)", [0.0, -0.7, 0.7, -0.3, 0.0, 0.0, 0.0]),
        ("🔄 회전 (좌)", [-0.5, -0.7, 0.7, -0.3, 0.0, 0.0, 0.0]),
        ("🔄 회전 (우)", [0.5, -0.7, 0.7, -0.3, 0.0, 0.0, 0.0]),
        ("🎯 목표 위치로", [0.0, -0.4, 0.4, -0.2, 0.0, 0.0, 0.0]),
        ("✋ 그리퍼 열기 (Release)", [0.0, -0.4, 0.4, -0.2, 0.0, 0.0, 0.52]),
        ("↩️  복귀", [0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.52]),
        ("🔄 손목 회전", [0.0, -0.5, 0.5, 0.0, 0.0, 1.57, 0.52]),
        ("🏠 홈 포지션", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ]
    
    print("\n" + "="*60)
    print("🎬 테스트 시퀀스 시작 (12단계)")
    print("="*60)
    
    # 메인 루프
    step = 0
    max_steps = len(test_sequence)
    frames_per_step = 180  # 각 동작당 3초 (60 FPS * 3)
    frame_count = 0
    
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.is_playing():
            if frame_count == 0:
                # 새로운 동작 시작
                if step < max_steps:
                    action_name, target_positions = test_sequence[step]
                    print(f"\n[{step+1}/{max_steps}] {action_name}")
                    
                    # Joint positions 설정
                    robot.set_joint_positions(np.array(target_positions))
                    
                    # 현재 그리퍼 상태 출력 (6번 조인트 = 좌측 그리퍼)
                    current_pos = robot.get_joint_positions()
                    if num_dof >= 6:
                        left_gripper = current_pos[5]  # gripper_left_hinge
                        right_gripper = current_pos[6] if num_dof >= 7 else "mimic"  # gripper_right_hinge
                        print(f"   🤏 그리퍼: L={left_gripper:.4f}, R={right_gripper if isinstance(right_gripper, str) else f'{right_gripper:.4f}'}")
                    
                    # 큐브 위치 출력
                    cube_pos = cube.get_world_pose()[0]
                    print(f"   📦 큐브: [{cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f}]")
                    
                    step += 1
                else:
                    # 모든 시퀀스 완료 - 무한 루프로 유지
                    print("\n" + "="*60)
                    print("✅ 모든 테스트 완료!")
                    print("🔄 시뮬레이션 계속 실행 중 (Ctrl+C로 종료)")
                    print("="*60)
                    step = 0  # 처음부터 반복
            
            frame_count = (frame_count + 1) % frames_per_step
        
        time.sleep(0.016)  # ~60 FPS
    
    simulation_app.close()

if __name__ == "__main__":
    main()
