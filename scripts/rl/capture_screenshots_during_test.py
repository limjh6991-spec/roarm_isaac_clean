#!/usr/bin/env python3
"""
학습된 모델 테스트하면서 스크린샷 캡처
동영상 생성용 이미지 시퀀스 저장
"""

import sys
from pathlib import Path
import numpy as np
import time
from PIL import Image

# stdout buffering 해결
sys.stdout = sys.stderr

from isaacsim import SimulationApp

# Isaac Sim 초기화 (GUI 모드)
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
})

print("✅ Isaac Sim 초기화 완료 (Screenshot Capture Mode)\n")

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import gymnasium as gym
from gymnasium import spaces

# 스크린샷 캡처
import omni.kit.app
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_buffer

# 환경 임포트
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from envs.roarm_pick_place_env import RoArmPickPlaceEnv, RoArmPickPlaceEnvCfg


class GymWrapper(gym.Env):
    """Isaac Sim 환경을 Gymnasium 형식으로 래핑"""
    
    def __init__(self):
        super().__init__()
        
        print("🔧 환경 생성 중...")
        cfg = RoArmPickPlaceEnvCfg()
        cfg.episode_length_s = 10.0
        
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
        
        print("✅ 환경 생성 완료\n")
    
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


def capture_screenshot(output_dir: Path, frame_number: int):
    """현재 뷰포트 스크린샷 캡처"""
    # 뷰포트 업데이트
    simulation_app.update()
    
    try:
        # 활성 뷰포트 가져오기
        viewport = get_active_viewport()
        
        if viewport is None:
            print("⚠️ 활성 뷰포트를 찾을 수 없습니다")
            return False
        
        # 버퍼로 캡처
        buffer = capture_viewport_to_buffer(viewport)
        
        if buffer is None:
            print("⚠️ 스크린샷 캡처 실패")
            return False
        
        # PIL Image로 변환
        image = Image.frombytes("RGBA", (buffer.shape[1], buffer.shape[0]), buffer.tobytes())
        
        # RGB로 변환 (알파 채널 제거)
        image = image.convert("RGB")
        
        # 저장
        output_path = output_dir / f"frame_{frame_number:06d}.png"
        image.save(output_path)
        
        return True
        
    except Exception as e:
        print(f"⚠️ 스크린샷 캡처 오류: {e}")
        return False


def test_and_capture(model_path: str, vecnorm_path: str, num_episodes: int = 3, 
                     output_dir: str = "logs/screenshots", capture_every: int = 2):
    """학습된 모델 테스트 및 스크린샷 캡처"""
    print("=" * 80)
    print("📸 학습된 모델 테스트 및 스크린샷 캡처")
    print("=" * 80)
    print(f"  모델 경로: {model_path}")
    print(f"  VecNormalize: {vecnorm_path}")
    print(f"  에피소드 수: {num_episodes}")
    print(f"  출력 디렉토리: {output_dir}")
    print(f"  캡처 주기: 매 {capture_every} 프레임\n")
    
    # 출력 디렉토리 생성
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 환경 생성
    def make_env():
        env = GymWrapper()
        return env
    
    env = DummyVecEnv([make_env])
    
    # VecNormalize 로드
    print("🔧 VecNormalize 로드 중...")
    env = VecNormalize.load(vecnorm_path, env)
    env.training = False
    env.norm_reward = False
    print("✅ VecNormalize 로드 완료\n")
    
    # 모델 로드
    print("🤖 PPO 모델 로드 중...")
    model = PPO.load(model_path, env=env)
    print(f"✅ 모델 로드 완료 (device: {model.device})\n")
    
    # 통계
    episode_rewards = []
    episode_lengths = []
    success_count = 0
    frame_number = 0
    total_captured = 0
    
    # 에피소드 실행
    for episode in range(num_episodes):
        obs = env.reset()
        episode_reward = 0
        episode_length = 0
        done = False
        
        print(f"{'='*80}")
        print(f"🎮 Episode {episode + 1}/{num_episodes}")
        print(f"{'='*80}")
        
        while not done:
            # 모델 추론
            action, _states = model.predict(obs, deterministic=True)
            
            # 환경 스텝
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated[0] or truncated[0]
            
            episode_reward += reward[0]
            episode_length += 1
            frame_number += 1
            
            # 스크린샷 캡처 (주기적)
            if frame_number % capture_every == 0:
                if capture_screenshot(output_dir, frame_number):
                    total_captured += 1
                    if total_captured % 50 == 0:
                        print(f"  📸 {total_captured} 프레임 캡처됨...")
            
            # 프레임 렌더링
            simulation_app.update()
        
        # 에피소드 통계
        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)
        
        env_info = info[0] if isinstance(info, list) else info
        if env_info.get("is_success", False):
            success_count += 1
        
        print(f"\n  ✅ Episode {episode + 1} 완료:")
        print(f"     Total Reward: {episode_reward:.2f}")
        print(f"     Length: {episode_length} steps")
        print(f"     Captured: {total_captured} frames")
        print()
        
        time.sleep(1)
    
    # 최종 통계
    print("\n" + "=" * 80)
    print("📊 최종 테스트 결과")
    print("=" * 80)
    print(f"  에피소드 수: {num_episodes}")
    print(f"  평균 보상: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"  평균 길이: {np.mean(episode_lengths):.0f} steps")
    print(f"  성공률: {success_count}/{num_episodes} ({success_count/num_episodes*100:.1f}%)")
    print(f"  총 캡처 프레임: {total_captured}")
    print(f"  출력 디렉토리: {output_dir}")
    print("=" * 80)
    
    print("\n💡 동영상 생성 명령:")
    print(f"   ffmpeg -framerate 30 -pattern_type glob -i '{output_dir}/*.png' \\")
    print(f"     -c:v libx264 -pix_fmt yuv420p -crf 20 \\")
    print(f"     logs/demo_video.mp4")
    
    return {
        "mean_reward": np.mean(episode_rewards),
        "success_rate": success_count / num_episodes,
        "total_frames": total_captured,
    }


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Test model and capture screenshots")
    parser.add_argument(
        "--model",
        type=str,
        default="logs/rl_training_curriculum/final_model/roarm_ppo_dense_final.zip",
        help="모델 경로"
    )
    parser.add_argument(
        "--vecnorm",
        type=str,
        default="logs/rl_training_curriculum/final_model/vecnormalize.pkl",
        help="VecNormalize 경로"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=3,
        help="테스트 에피소드 수 (기본: 3)"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="logs/screenshots",
        help="스크린샷 출력 디렉토리"
    )
    parser.add_argument(
        "--capture-every",
        type=int,
        default=2,
        help="몇 프레임마다 캡처할지 (기본: 2, 30fps → 15fps)"
    )
    
    args = parser.parse_args()
    
    # 경로 확인
    model_path = Path(args.model)
    vecnorm_path = Path(args.vecnorm)
    
    if not model_path.exists():
        print(f"❌ 모델 파일을 찾을 수 없습니다: {model_path}")
        simulation_app.close()
        return
    
    if not vecnorm_path.exists():
        print(f"❌ VecNormalize 파일을 찾을 수 없습니다: {vecnorm_path}")
        simulation_app.close()
        return
    
    try:
        results = test_and_capture(
            model_path=str(model_path),
            vecnorm_path=str(vecnorm_path),
            num_episodes=args.episodes,
            output_dir=args.output_dir,
            capture_every=args.capture_every,
        )
        
        print("\n✅ 캡처 완료!")
        
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🔚 Isaac Sim 종료 중...")
        simulation_app.close()


if __name__ == "__main__":
    main()
