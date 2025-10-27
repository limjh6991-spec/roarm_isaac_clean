"""
강화학습 학습 진행 상황 로깅 콜백
에피소드별 상세 정보 출력 및 TensorBoard 기록
"""

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
from collections import deque
import time


class TrainingProgressCallback(BaseCallback):
    """
    학습 진행 상황을 추적하고 로깅하는 콜백
    
    기능:
    - 에피소드별 보상 출력
    - 성공률 추적 (sliding window)
    - 단계별 달성률 (REACH, GRIP, LIFT, PLACE)
    - TensorBoard 커스텀 메트릭
    """
    
    def __init__(self, verbose=1, log_freq=10):
        super().__init__(verbose)
        self.log_freq = log_freq
        
        # 에피소드 추적
        self.episode_rewards = deque(maxlen=100)
        self.episode_lengths = deque(maxlen=100)
        self.episode_successes = deque(maxlen=100)
        
        # 단계별 달성 추적
        self.reach_count = deque(maxlen=100)
        self.grip_count = deque(maxlen=100)
        self.lift_count = deque(maxlen=100)
        self.place_count = deque(maxlen=100)
        
        # 현재 에피소드
        self.current_episode_reward = 0
        self.current_episode_length = 0
        self.episode_count = 0
        
        # 시간 추적
        self.start_time = time.time()
        self.last_log_time = time.time()
        
    def _on_step(self) -> bool:
        # 현재 스텝 정보
        self.current_episode_reward += self.locals["rewards"][0]
        self.current_episode_length += 1
        
        # 에피소드 종료 확인
        if self.locals["dones"][0]:
            self._on_episode_end()
        
        return True
    
    def _on_episode_end(self):
        """에피소드 종료 시 처리"""
        self.episode_count += 1
        
        # 에피소드 정보 저장
        self.episode_rewards.append(self.current_episode_reward)
        self.episode_lengths.append(self.current_episode_length)
        
        # info에서 성공 여부 및 단계별 달성 확인
        info = self.locals.get("infos", [{}])[0]
        
        success = info.get("is_success", False)
        self.episode_successes.append(1.0 if success else 0.0)
        
        # 단계별 달성 (info에 있다면)
        self.reach_count.append(1.0 if info.get("reached_near_cube", False) else 0.0)
        self.grip_count.append(1.0 if info.get("reached_grasp", False) else 0.0)
        self.lift_count.append(1.0 if info.get("reached_lift", False) else 0.0)
        self.place_count.append(1.0 if info.get("reached_near_target", False) else 0.0)
        
        # 주기적 로깅
        if self.episode_count % self.log_freq == 0:
            self._log_progress()
        
        # TensorBoard 로깅 (매 에피소드)
        self._log_tensorboard()
        
        # 에피소드 리셋
        self.current_episode_reward = 0
        self.current_episode_length = 0
    
    def _log_progress(self):
        """콘솔에 진행 상황 출력"""
        if len(self.episode_rewards) == 0:
            return
        
        # 통계 계산
        mean_reward = np.mean(self.episode_rewards)
        mean_length = np.mean(self.episode_lengths)
        success_rate = np.mean(self.episode_successes) * 100
        
        reach_rate = np.mean(self.reach_count) * 100 if len(self.reach_count) > 0 else 0
        grip_rate = np.mean(self.grip_count) * 100 if len(self.grip_count) > 0 else 0
        lift_rate = np.mean(self.lift_count) * 100 if len(self.lift_count) > 0 else 0
        place_rate = np.mean(self.place_count) * 100 if len(self.place_count) > 0 else 0
        
        # 시간 정보
        elapsed = time.time() - self.start_time
        elapsed_since_log = time.time() - self.last_log_time
        fps = (self.log_freq * mean_length) / elapsed_since_log if elapsed_since_log > 0 else 0
        
        # 출력
        print(f"\n{'='*80}")
        print(f"📊 Episode {self.episode_count} | Timesteps: {self.num_timesteps}")
        print(f"{'='*80}")
        print(f"  ⏱️  Time: {elapsed/60:.1f}min | FPS: {fps:.0f}")
        print(f"  🎯 Reward: {mean_reward:.2f} (last 100 episodes)")
        print(f"  📏 Length: {mean_length:.0f} steps")
        print(f"  ✅ Success: {success_rate:.1f}%")
        print(f"\n  📈 Milestone Rates (last 100 episodes):")
        print(f"     REACH : {reach_rate:5.1f}%  (EE 큐브 10cm 이내)")
        print(f"     GRIP  : {grip_rate:5.1f}%  (큐브 잡기)")
        print(f"     LIFT  : {lift_rate:5.1f}%  (큐브 들어올리기)")
        print(f"     PLACE : {place_rate:5.1f}%  (타겟 10cm 이내)")
        print(f"{'='*80}\n")
        
        self.last_log_time = time.time()
    
    def _log_tensorboard(self):
        """TensorBoard에 커스텀 메트릭 기록"""
        if len(self.episode_rewards) == 0:
            return
        
        # 기본 메트릭
        self.logger.record("episode/reward", self.episode_rewards[-1])
        self.logger.record("episode/length", self.episode_lengths[-1])
        self.logger.record("episode/success", self.episode_successes[-1])
        
        # 이동 평균 (last 100)
        self.logger.record("episode/mean_reward", np.mean(self.episode_rewards))
        self.logger.record("episode/success_rate", np.mean(self.episode_successes) * 100)
        
        # 단계별 달성률
        if len(self.reach_count) > 0:
            self.logger.record("milestone/reach_rate", np.mean(self.reach_count) * 100)
        if len(self.grip_count) > 0:
            self.logger.record("milestone/grip_rate", np.mean(self.grip_count) * 100)
        if len(self.lift_count) > 0:
            self.logger.record("milestone/lift_rate", np.mean(self.lift_count) * 100)
        if len(self.place_count) > 0:
            self.logger.record("milestone/place_rate", np.mean(self.place_count) * 100)


class CurriculumCallback(BaseCallback):
    """
    Curriculum Learning 자동 승급 콜백
    
    성공률이 threshold를 넘으면 다음 Phase로 전환
    """
    
    def __init__(self, 
                 env_wrapper,
                 success_window=100,
                 phase_0_threshold=0.30,
                 phase_1_threshold=0.60,
                 verbose=1):
        super().__init__(verbose)
        self.env_wrapper = env_wrapper
        self.success_window = success_window
        self.phase_0_threshold = phase_0_threshold
        self.phase_1_threshold = phase_1_threshold
        
        # 성공 추적
        self.recent_successes = deque(maxlen=success_window)
        self.episode_count = 0
        
        # 현재 Phase
        self.current_phase = 0
    
    def _on_step(self) -> bool:
        # 에피소드 종료 시 성공 여부 기록
        if self.locals["dones"][0]:
            info = self.locals.get("infos", [{}])[0]
            success = info.get("is_success", False)
            self.recent_successes.append(1.0 if success else 0.0)
            self.episode_count += 1
            
            # 승급 체크 (충분한 에피소드 후)
            if len(self.recent_successes) >= self.success_window:
                self._check_promotion()
        
        return True
    
    def _check_promotion(self):
        """승급 조건 체크"""
        success_rate = np.mean(self.recent_successes)
        
        # Phase 0 → Phase 1
        if self.current_phase == 0 and success_rate >= self.phase_0_threshold:
            self._promote_to_phase_1()
        
        # Phase 1 → Phase 2
        elif self.current_phase == 1 and success_rate >= self.phase_1_threshold:
            self._promote_to_phase_2()
    
    def _promote_to_phase_1(self):
        """Phase 1으로 승급"""
        print("\n" + "🎓" * 40)
        print("🎉 CURRICULUM PROMOTION: Phase 0 → Phase 1")
        print("🎓" * 40)
        print(f"  성공률: {np.mean(self.recent_successes)*100:.1f}% (목표: {self.phase_0_threshold*100:.0f}%)")
        print(f"  새로운 난이도:")
        print(f"    - 큐브 거리: 35-50cm (이전: 15-20cm)")
        print(f"    - 타겟 거리: 35-50cm (이전: 25-30cm)")
        print("🎓" * 40 + "\n")
        
        # 환경 설정 변경
        self.env_wrapper.env.cfg.curriculum_phase = 1
        self.current_phase = 1
        
        # 성공 추적 리셋
        self.recent_successes.clear()
    
    def _promote_to_phase_2(self):
        """Phase 2로 승급"""
        print("\n" + "🏆" * 40)
        print("🎉 CURRICULUM PROMOTION: Phase 1 → Phase 2 (FINAL)")
        print("🏆" * 40)
        print(f"  성공률: {np.mean(self.recent_successes)*100:.1f}% (목표: {self.phase_1_threshold*100:.0f}%)")
        print(f"  새로운 난이도:")
        print(f"    - 완전 랜덤 배치 (작업 공간 전체)")
        print("🏆" * 40 + "\n")
        
        # 환경 설정 변경
        self.env_wrapper.env.cfg.curriculum_phase = 2
        self.current_phase = 2
        
        # 성공 추적 리셋
        self.recent_successes.clear()
