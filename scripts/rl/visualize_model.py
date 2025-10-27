#!/home/roarm_m3/isaacsim/python.sh
"""
학습된 모델 시각화 및 동영상 녹화
현재 동작을 보여주는 데모
"""

import sys
from pathlib import Path

from isaacsim import SimulationApp

# GUI 모드로 실행 (녹화 위해)
simulation_app = SimulationApp({
    "headless": False,  # GUI 표시
    "width": 1920,
    "height": 1080,
})

print("✅ Isaac Sim 초기화 완료 (GUI 모드)\n")

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import gymnasium as gym
from gymnasium import spaces
import time
import importlib

# 환경 임포트
project_root = Path(__file__).parent.parent.parent

# 🔥 FORCE RELOAD: 모듈 캐시 무시하고 강제 재로드
print("🔄 Forcing module reload...")
if 'envs.roarm_pick_place_env' in sys.modules:
    del sys.modules['envs.roarm_pick_place_env']
if 'robot_utils' in sys.modules:
    del sys.modules['robot_utils']
if 'envs' in sys.modules:
    del sys.modules['envs']
print("✅ Module cache cleared")
sys.path.insert(0, str(project_root))

from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg

# Movie Capture 임포트
try:
    import omni.kit.viewport.utility as viewport_utils
    from omni.kit.viewport.utility import get_active_viewport
    MOVIE_CAPTURE_AVAILABLE = True
except ImportError:
    print("⚠️ Movie capture not available, will use manual observation")
    MOVIE_CAPTURE_AVAILABLE = False


class GymWrapper(gym.Env):
    """환경 래퍼"""
    
    def __init__(self):
        super().__init__()
        
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0
        cfg.curriculum_enabled = True
        cfg.curriculum_phase = 0
        
        self.env = RoArmPickPlaceEnv(cfg)
        self._max_episode_steps = self.env.max_steps
        
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.env.observation_space_dim,),
            dtype=np.float32
        )
        
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.env.action_space_dim,),
            dtype=np.float32
        )
        
        self.episode_count = 0
        self.total_steps = 0
        self._elapsed_steps = 0
    
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
            torch.manual_seed(seed)
        
        obs = self.env.reset()
        self.episode_count += 1
        self._elapsed_steps = 0
        
        return obs, {}
    
    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        self.total_steps += 1
        self._elapsed_steps += 1
        
        terminated = done and self._elapsed_steps < self._max_episode_steps
        truncated = self._elapsed_steps >= self._max_episode_steps
        
        if truncated:
            info["TimeLimit.truncated"] = True
        
        return obs, reward, terminated, truncated, info


def demo_model(model_path: str, vecnorm_path: str, num_episodes: int = 3):
    """학습된 모델 데모"""
    print("=" * 80)
    print("🎬 학습된 모델 데모 (동영상 녹화)")
    print("=" * 80)
    print(f"  모델: {model_path}")
    print(f"  에피소드 수: {num_episodes}")
    print()
    
    # 환경 생성
    def make_env():
        return GymWrapper()
    
    print("🔧 환경 생성 중...")
    env = DummyVecEnv([make_env])
    
    # VecNormalize 로드
    print(f"📂 VecNormalize 로딩: {vecnorm_path}")
    env = VecNormalize.load(vecnorm_path, env)
    env.training = False  # 테스트 모드
    env.norm_reward = False  # 보상 정규화 비활성화
    
    # 모델 로드
    print(f"📂 모델 로딩: {model_path}")
    model = PPO.load(model_path, env=env)
    print(f"  Device: {model.device}")
    print()
    
    # 데모 실행
    print("🎬 데모 시작...\n")
    
    for ep in range(num_episodes):
        print(f"\n{'='*80}")
        print(f"📺 에피소드 {ep+1}/{num_episodes}")
        print(f"{'='*80}\n")
        
        obs = env.reset()
        done = False
        step_count = 0
        episode_reward = 0
        
        # 🔧 초기 상태 관찰을 위한 1초 대기
        print("⏸️  초기 상태 확인 중 (1초 대기)...")
        for _ in range(60):  # 60 프레임 = 1초
            simulation_app.update()
            time.sleep(1.0 / 60.0)
        print("▶️  액션 시작!\n")
        
        # 마일스톤 추적
        reached_milestones = {
            "REACH": False,
            "GRIP": False,
            "LIFT": False,
            "PLACE": False
        }
        
        while not done:
            # 액션 예측
            action, _states = model.predict(obs, deterministic=True)
            
            # 스텝 실행
            obs, reward, done, info = env.step(action)
            
            step_count += 1
            episode_reward += reward[0]
            
            # Info에서 마일스톤 확인
            if len(info) > 0:
                env_info = info[0]
                
                # 마일스톤 달성 확인
                if not reached_milestones["REACH"] and env_info.get("reached_near_cube", False):
                    print(f"  ✅ Step {step_count:3d}: REACH! (EE 큐브 근처)")
                    reached_milestones["REACH"] = True
                
                if not reached_milestones["GRIP"] and env_info.get("reached_grasp", False):
                    print(f"  ✅ Step {step_count:3d}: GRIP! (큐브 잡음)")
                    reached_milestones["GRIP"] = True
                
                if not reached_milestones["LIFT"] and env_info.get("reached_lift", False):
                    print(f"  ✅ Step {step_count:3d}: LIFT! (큐브 들어올림)")
                    reached_milestones["LIFT"] = True
                
                if not reached_milestones["PLACE"] and env_info.get("reached_near_target", False):
                    print(f"  ✅ Step {step_count:3d}: PLACE! (타겟 근처)")
                    reached_milestones["PLACE"] = True
                
                # 성공 확인
                if env_info.get("is_success", False):
                    print(f"\n  🎉 SUCCESS! Step {step_count}")
            
            # 주기적 진행 상황 출력 (50 스텝마다)
            if step_count % 50 == 0:
                print(f"  📊 Step {step_count:3d}: Reward={episode_reward:.2f}")
            
            # 렌더링 (GUI에 표시) - 중요!
            simulation_app.update()  # 화면 업데이트
            time.sleep(0.05)  # 시각화 속도 조절 (0.05초 = 20 FPS, 관찰하기 좋은 속도)
        
        # 에피소드 종료
        print(f"\n📊 에피소드 {ep+1} 완료:")
        print(f"  - 스텝 수: {step_count}")
        print(f"  - 총 보상: {episode_reward:.2f}")
        print(f"  - 달성 마일스톤:")
        for milestone, achieved in reached_milestones.items():
            status = "✅" if achieved else "❌"
            print(f"    {status} {milestone}")
        print()
    
    print("\n" + "=" * 80)
    print("✅ 데모 완료!")
    print("=" * 80)
    print("\n💡 관찰 사항:")
    print("  - 로봇이 큐브에 접근하는지 확인")
    print("  - 그리퍼가 열리고 닫히는지 확인")
    print("  - GRIP이 달성되지 않는다면 그리퍼 제어 문제일 수 있음")
    print()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="모델 데모 및 동영상 녹화")
    parser.add_argument(
        "--model",
        type=str,
        default="logs/quick_test/quick_test_model.zip",
        help="모델 경로"
    )
    parser.add_argument(
        "--vecnorm",
        type=str,
        default="logs/quick_test/vecnormalize.pkl",
        help="VecNormalize 경로"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=3,
        help="데모 에피소드 수"
    )
    
    args = parser.parse_args()
    
    # 파일 존재 확인
    if not Path(args.model).exists():
        print(f"❌ 모델 파일을 찾을 수 없습니다: {args.model}")
        print("\n사용 가능한 모델:")
        for model_file in Path("logs").rglob("*.zip"):
            print(f"  - {model_file}")
        simulation_app.close()
        return
    
    if not Path(args.vecnorm).exists():
        print(f"❌ VecNormalize 파일을 찾을 수 없습니다: {args.vecnorm}")
        simulation_app.close()
        return
    
    try:
        demo_model(args.model, args.vecnorm, args.episodes)
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🔚 Isaac Sim 종료 중...")
        input("엔터를 눌러 종료하세요...")
        simulation_app.close()


if __name__ == "__main__":
    main()
