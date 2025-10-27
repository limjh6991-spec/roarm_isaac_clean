#!/usr/bin/env python3
"""
v3.9.0 GRIP 품질 검증 스크립트
False Positive 탐지 및 단계별 지표 분석
"""

import numpy as np
import argparse
from pathlib import Path

def validate_grip_quality(model_path: str, vecnorm_path: str, n_episodes: int = 50):
    """GRIP 품질 검증 (False Positive 탐지)"""
    
    print("=" * 80)
    print("🔬 v3.9.0 GRIP Quality Validation")
    print("=" * 80)
    print(f"Model: {model_path}")
    print(f"VecNormalize: {vecnorm_path}")
    print(f"Episodes: {n_episodes}")
    print()
    
    # Isaac Sim 환경 초기화
    from omni.isaac.kit import SimulationApp
    simulation_app = SimulationApp({"headless": False})  # GUI로 확인
    
    # 환경 및 모델 로드
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
    import sys
    sys.path.append('/home/roarm_m3/roarm_isaac_clean')
    from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg
    
    # 환경 생성
    cfg = RoArmPickPlaceEnvCfg()
    env = DummyVecEnv([lambda: RoArmPickPlaceEnv(cfg)])
    env = VecNormalize.load(vecnorm_path, env)
    env.training = False
    env.norm_reward = False
    
    # 모델 로드
    model = PPO.load(model_path)
    
    # 메트릭 수집
    metrics = {
        'reach': [],
        'attach': [],
        'lift': [],
        'success': []
    }
    
    print("\n" + "=" * 80)
    print("📊 Running Validation Episodes...")
    print("=" * 80 + "\n")
    
    for ep in range(n_episodes):
        obs = env.reset()
        done = False
        
        ep_reach = False
        ep_attach = False
        ep_lift = False
        ep_success = False
        
        step_count = 0
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            step_count += 1
            
            # 실제 환경에서 메트릭 가져오기
            actual_env = env.venv.envs[0]
            
            # 단계별 체크
            if actual_env.first_reach:
                ep_reach = True
            if actual_env.first_attach:
                ep_attach = True
            if actual_env.first_lift:
                ep_lift = True
            if actual_env.success:
                ep_success = True
        
        metrics['reach'].append(ep_reach)
        metrics['attach'].append(ep_attach)
        metrics['lift'].append(ep_lift)
        metrics['success'].append(ep_success)
        
        # 진행 상황 출력
        print(f"Episode {ep+1:3d}/{n_episodes}: "
              f"Reach={ep_reach} | "
              f"Attach={ep_attach} | "
              f"Lift={ep_lift} | "
              f"Success={ep_success} | "
              f"Steps={step_count}")
    
    # 분석
    reach_rate = np.mean(metrics['reach'])
    attach_rate = np.mean(metrics['attach'])
    lift_rate = np.mean(metrics['lift'])
    success_rate = np.mean(metrics['success'])
    
    # False Positive 계산
    false_positives = sum(metrics['attach']) - sum(metrics['success'])
    fp_rate = false_positives / sum(metrics['attach']) if sum(metrics['attach']) > 0 else 0
    
    # 단계별 전환율
    reach_to_attach = attach_rate / reach_rate if reach_rate > 0 else 0
    attach_to_lift = lift_rate / attach_rate if attach_rate > 0 else 0
    lift_to_success = success_rate / lift_rate if lift_rate > 0 else 0
    
    # 결과 출력
    print("\n" + "=" * 80)
    print("📊 Validation Results")
    print("=" * 80)
    print(f"""
╔════════════════════════════════════════════════════════════════╗
║  GRIP Quality Validation ({n_episodes} episodes)                  ║
╠════════════════════════════════════════════════════════════════╣
║  📍 Reach:    {reach_rate:6.1%}  (EE < 5cm from cube)             ║
║  🤝 Attach:   {attach_rate:6.1%}  ← Physical attachment            ║
║  ⬆️  Lift:     {lift_rate:6.1%}  ← Cube raised 2cm               ║
║  🏆 Success:  {success_rate:6.1%}  ← Attach & Lift & Hold         ║
╠════════════════════════════════════════════════════════════════╣
║  Conversion Rates:                                             ║
║  ├─ Reach → Attach:  {reach_to_attach:6.1%}                         ║
║  ├─ Attach → Lift:   {attach_to_lift:6.1%}                          ║
║  └─ Lift → Success:  {lift_to_success:6.1%}                         ║
╠════════════════════════════════════════════════════════════════╣
║  ⚠️  False Positive Rate: {fp_rate:6.1%}                           ║
║  (Attached but didn't lift successfully)                       ║
╠════════════════════════════════════════════════════════════════╣
║  Quality Assessment:                                           ║
║  {f'✅ GOOD: FP < 10%' if fp_rate < 0.10 else '⚠️  FAIR: FP < 20%' if fp_rate < 0.20 else '❌ BAD: FP > 20% - need stricter conditions'}       ║
╚════════════════════════════════════════════════════════════════╝
    """)
    
    # 추가 분석
    print("\n📈 Detailed Breakdown:")
    print(f"  Episodes with Reach:   {sum(metrics['reach'])}/{n_episodes}")
    print(f"  Episodes with Attach:  {sum(metrics['attach'])}/{n_episodes}")
    print(f"  Episodes with Lift:    {sum(metrics['lift'])}/{n_episodes}")
    print(f"  Episodes with Success: {sum(metrics['success'])}/{n_episodes}")
    print(f"  False Positives:       {false_positives} episodes")
    
    # 권장사항
    print("\n💡 Recommendations:")
    if fp_rate < 0.05:
        print("  ✅ Excellent! Grip definition is robust.")
    elif fp_rate < 0.10:
        print("  ✅ Good. Grip definition is reliable.")
    elif fp_rate < 0.20:
        print("  ⚠️  Fair. Consider tightening attach or lift conditions.")
    else:
        print("  ❌ Poor. Attach doesn't guarantee lift - conditions too loose!")
        print("     Suggestion: Increase lift threshold or attach duration.")
    
    simulation_app.close()
    return metrics

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="v3.9.0 GRIP Quality Validation")
    parser.add_argument("--model", type=str, required=True, help="Path to model.zip")
    parser.add_argument("--vecnorm", type=str, required=True, help="Path to vecnormalize.pkl")
    parser.add_argument("--episodes", type=int, default=50, help="Number of episodes to test")
    
    args = parser.parse_args()
    
    validate_grip_quality(args.model, args.vecnorm, args.episodes)
