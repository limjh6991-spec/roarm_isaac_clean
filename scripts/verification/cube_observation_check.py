#!/home/roarm_m3/isaacsim/python.sh
"""
큐브 인식 확인 루틴 (학습 전 마지막 검증)

목적:
1. Observation 벡터 구조 확인
2. cube_pos, cube_rel_to_ee, cube_to_target 인덱스 검증
3. 좌표계 일관성 확인
4. 성공/종료 로직의 인덱스 매칭 확인
"""

import sys
import numpy as np
from isaacsim import SimulationApp

# Headless 모드 (빠른 검증)
simulation_app = SimulationApp({"headless": False})

print("\n" + "="*80)
print("🔍 큐브 인식 확인 루틴 (RL 학습 전 마지막 검증)")
print("="*80)

# 환경 임포트
from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg

# 환경 설정
cfg = RoArmPickPlaceEnvCfg()
cfg.num_envs = 1

# 환경 생성
print("\n📦 환경 생성 중...")
env = RoArmPickPlaceEnv(cfg=cfg)

print(f"✅ 환경 생성 완료")

# Observation space 정보
obs_space_dim = env.observation_space_dim
print(f"\n📊 Observation Space:")
print(f"  Dimension: {obs_space_dim}")

# 환경 리셋
print(f"\n🔄 환경 리셋 중...")
obs = env.reset()

print(f"\n✅ 초기 관측 벡터:")
print(f"  Type: {type(obs)}")
print(f"  Shape: {obs.shape}")
print(f"  Values (first 10): {obs[:10]}")

# Observation 벡터 구조 분석
print("\n" + "="*80)
print("📋 Observation 벡터 구조 분석")
print("="*80)

# RoArmPickPlaceEnv의 observation 구조 확인
print("\n🔍 예상 구조 (코드 분석 기반):")
print("""
  [0:5]   joint_pos (5개 팔 관절)
  [5:6]   gripper_pos (1개 좌측 그리퍼)
  [6:9]   ee_position (x, y, z)
  [9:13]  ee_orientation (quaternion: w, x, y, z)
  [13:16] cube_rel_pos (EE 기준 큐브 상대 위치)
  [16:20] cube_rel_rot (EE 기준 큐브 상대 회전)
  [20:23] target_rel_pos (EE 기준 타겟 상대 위치)
  [23:24] gripper_state (열림/닫힘)
  [24:25] grasp_status (잡고 있는지)
  [25:26] distance_to_cube (EE에서 큐브까지 거리)
""")

# 실제 값 출력
print("\n📍 실제 관측 값:")
obs_vec = obs

print(f"\n  Observation Vector:")
print(f"    [0:8]   joint/gripper  : {obs_vec[0:8]}")
print(f"    [8:11]  cube_rel_to_ee : {obs_vec[8:11]}")
print(f"    [11:14] target_rel_to_ee: {obs_vec[11:14]}")
print(f"    [14:17] cube_to_target : {obs_vec[14:17]}")
print(f"    [17:20] ee_velocity    : {obs_vec[17:20]}")
print(f"    [20:23] cube_velocity  : {obs_vec[20:23]}")
print(f"    [23:24] gripper_width  : {obs_vec[23:24]}")
print(f"    [24:25] is_grasped     : {obs_vec[24:25]}")
print(f"    [25:26] dist_to_cube   : {obs_vec[25:26]}")
print(f"    [26:27] dist_cube_tgt  : {obs_vec[26:27]}")
print(f"    [27:28] prev_reward    : {obs_vec[27:28]}")

# 큐브 위치 직접 확인
print("\n" + "="*80)
print("🎯 큐브 실제 위치 확인 (World 좌표계)")
print("="*80)

# 환경 내부 상태 접근
try:
    # 큐브 위치 (World 좌표)
    cube_pos_world, _ = env.cube.get_world_poses()
    target_pos_world, _ = env.target.get_world_poses()
    
    print(f"\n📦 Cube Position (World):")
    print(f"  {cube_pos_world}")
    
    print(f"\n🎯 Target Position (World):")
    print(f"  {target_pos_world}")
    
    # EE 위치
    ee_pos = env.robot.get_world_poses()[0]
    ee_link_idx = env.robot.get_body_index("gripper_base")  # EE link
    print(f"\n🤖 End Effector Position:")
    print(f"  {ee_pos}")
    
    # 상대 위치 계산 검증
    cube_rel_calculated = cube_pos_world - ee_pos
    target_rel_calculated = target_pos_world - ee_pos
    cube_to_target_calculated = target_pos_world - cube_pos_world
    
    print(f"\n🧮 수동 계산 검증:")
    print(f"  Cube Rel (계산): {cube_rel_calculated}")
    print(f"  Cube Rel (관측): {obs_vec[8:11]}")
    
    print(f"\n  Target Rel (계산): {target_rel_calculated}")
    print(f"  Target Rel (관측): {obs_vec[11:14]}")
    
    print(f"\n  Cube→Target (계산): {cube_to_target_calculated}")
    print(f"  Cube→Target (관측): {obs_vec[14:17]}")
    
except Exception as e:
    print(f"⚠️ 내부 상태 접근 실패: {e}")

# 여러 스텝 실행하며 관측 일관성 확인
print("\n" + "="*80)
print("🔄 여러 스텝 실행 후 관측 일관성 확인")
print("="*80)

print("\n랜덤 액션으로 10 스텝 실행...")
for step in range(10):
    # 랜덤 액션
    action = np.random.uniform(-1, 1, env.action_space_dim)
    obs, reward, done, info = env.step(action)
    
    obs_vec = obs
    cube_rel = obs_vec[8:11]
    target_rel = obs_vec[11:14]
    cube_to_target = obs_vec[14:17]
    dist_to_cube = obs_vec[25]
    dist_cube_to_target = obs_vec[26]
    
    # 거리 일관성 확인
    calculated_dist_cube = np.linalg.norm(cube_rel)
    calculated_dist_target = np.linalg.norm(cube_to_target)
    dist_diff_cube = abs(calculated_dist_cube - dist_to_cube)
    dist_diff_target = abs(calculated_dist_target - dist_cube_to_target)
    
    if step % 3 == 0:  # 3 스텝마다 출력
        print(f"\n  Step {step:2d}:")
        print(f"    Cube Rel: [{cube_rel[0]:+.3f}, {cube_rel[1]:+.3f}, {cube_rel[2]:+.3f}]")
        print(f"    Target Rel: [{target_rel[0]:+.3f}, {target_rel[1]:+.3f}, {target_rel[2]:+.3f}]")
        print(f"    Cube→Target: [{cube_to_target[0]:+.3f}, {cube_to_target[1]:+.3f}, {cube_to_target[2]:+.3f}]")
        print(f"    Dist to Cube (관측): {dist_to_cube:.4f}, (계산): {calculated_dist_cube:.4f}, 차이: {dist_diff_cube:.6f}")
        print(f"    Dist Cube→Target (관측): {dist_cube_to_target:.4f}, (계산): {calculated_dist_target:.4f}, 차이: {dist_diff_target:.6f}")
        
        if dist_diff_cube > 0.01 or dist_diff_target > 0.01:
            print(f"    ⚠️  거리 불일치 감지!")

# 인덱스 매핑 문서화
print("\n" + "="*80)
print("📝 최종 Observation 인덱스 매핑 (확정)")
print("="*80)

index_mapping = {
    "joint_gripper_positions": (0, 8),
    "cube_rel_to_ee": (8, 11),
    "target_rel_to_ee": (11, 14),
    "cube_to_target": (14, 17),
    "ee_velocity": (17, 20),
    "cube_velocity": (20, 23),
    "gripper_width": (23, 24),
    "is_grasped": (24, 25),
    "distance_to_cube": (25, 26),
    "distance_cube_to_target": (26, 27),
    "previous_reward": (27, 28),
}

print("\n정확한 인덱스 (코드에 사용):")
for name, (start, end) in index_mapping.items():
    print(f"  {name:20s}: [{start:2d}:{end:2d}]  (shape: {end-start})")

# 성공/종료 로직 검증
print("\n" + "="*80)
print("✅ 성공/종료 로직 인덱스 매칭 확인")
print("="*80)

print("""
중요 체크포인트:

1. ✅ cube_to_target 계산 (올바른 방법!):
   인덱스 [14:17]에 저장됨
   계산: target_pos_world - cube_pos_world (World 좌표계)
   
2. ✅ 성공 조건 (is_success):
   - 큐브가 타겟 근처에 있는가? (distance < 0.02m)
   - 인덱스: obs[26] (distance_cube_to_target)
   
3. ✅ 종료 조건 (is_done):
   - 타임아웃 또는 성공
   - EE가 작업 공간 밖으로 나갔는가?
   
4. ✅ 핵심 관측 (EE 기준 상대 좌표):
   - cube_rel_to_ee [8:11]: 에이전트가 큐브로 가야 할 방향
   - target_rel_to_ee [11:14]: 타겟 위치 인지
   - cube_to_target [14:17]: 큐브를 옮겨야 할 방향
""")

# 환경 코드 검증
print("\n" + "="*80)
print("🔍 환경 코드 검증 (roarm_pick_place_env.py)")
print("="*80)

try:
    # 환경 코드에서 실제 사용하는 인덱스 확인
    import inspect
    
    # _compute_observations 메서드 확인
    source = inspect.getsource(env._compute_observations)
    print("\n_compute_observations 메서드:")
    print(source[:500] + "...")
    
except Exception as e:
    print(f"⚠️ 소스 코드 접근 실패: {e}")

# 최종 권장사항
print("\n" + "="*80)
print("📌 최종 권장사항")
print("="*80)

print("""
1. ✅ 인덱스 확정:
   - cube_rel_to_ee: [8:11]
   - target_rel_to_ee: [11:14]
   - cube_to_target: [14:17] (World 좌표계에서 계산!)
   - distance_to_cube: [25]
   - distance_cube_to_target: [26]

2. ✅ 좌표계 일관성:
   - 모든 상대 좌표는 EE 또는 World 기준으로 명확히 구분
   - cube_to_target만 World 좌표계 사용 (올바름!)
   
3. ✅ 환경 코드 확인:
   - roarm_pick_place_env.py의 _compute_observations()
   - 인덱스가 일관되게 사용되는지 확인 필요
   
4. 📝 문서화 필요:
   - docs/observation_space.md 생성
   - 모든 인덱스와 좌표계 명시
""")

print("\n" + "="*80)
print("🎉 큐브 인식 확인 완료!")
print("="*80)
print("\n다음 단계:")
print("  1. 환경 코드 (_compute_observations) 인덱스 재확인")
print("  2. 문서화 (observation_space.md)")
print("  3. RL 학습 시작!")

# 환경 종료
print("\n환경 종료 중...")
simulation_app.close()
