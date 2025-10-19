#!/usr/bin/env python3
"""
조기 경보 시스템 (Early Warning System)
전문가 의견 우선순위 #5: EV/Value Loss 모니터링
"""

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback


class EarlyWarningCallback(BaseCallback):
    """
    가치 함수 붕괴 조기 감지 및 자동 중단
    
    중단 조건:
    1. Explained Variance < 0.05가 5회 연속
    2. Value Loss가 5회 연속 증가
    """
    
    def __init__(self, 
                 ev_threshold: float = 0.05,
                 ev_consecutive: int = 5,
                 vl_consecutive: int = 5,
                 verbose: int = 1):
        super().__init__(verbose)
        
        self.ev_threshold = ev_threshold
        self.ev_consecutive = ev_consecutive
        self.vl_consecutive = vl_consecutive
        
        # 추적 변수
        self.ev_low_count = 0
        self.vl_increase_count = 0
        self.prev_value_loss = None
        
        # 경고 플래그
        self.warning_triggered = False
    
    def _on_step(self) -> bool:
        """매 스텝마다 호출"""
        return True
    
    def _on_rollout_end(self) -> bool:
        """Rollout 종료 시 체크"""
        
        # Tensorboard 로그에서 지표 추출
        if len(self.model.ep_info_buffer) > 0:
            # Explained Variance 체크
            if hasattr(self.logger, 'name_to_value'):
                ev = self.logger.name_to_value.get('train/explained_variance', None)
                vl = self.logger.name_to_value.get('train/value_loss', None)
                
                if ev is not None:
                    # EV < threshold 연속 체크
                    if ev < self.ev_threshold:
                        self.ev_low_count += 1
                        if self.verbose > 0:
                            print(f"⚠️  EV 낮음: {ev:.6f} (연속 {self.ev_low_count}/{self.ev_consecutive}회)")
                    else:
                        self.ev_low_count = 0  # 리셋
                    
                    # 5회 연속 → 중단
                    if self.ev_low_count >= self.ev_consecutive:
                        print(f"\n🚨 조기 경보 발동!")
                        print(f"   Explained Variance < {self.ev_threshold}가 {self.ev_consecutive}회 연속 발생")
                        print(f"   현재 EV: {ev:.6f}")
                        print(f"   🛑 학습 중단 및 체크포인트 저장\n")
                        
                        # 긴급 체크포인트 저장
                        self.model.save(f"logs/rl_training_sparse/emergency_checkpoint_{self.num_timesteps}.zip")
                        
                        self.warning_triggered = True
                        return False  # 학습 중단
                
                if vl is not None:
                    # Value Loss 증가 추세 체크
                    if self.prev_value_loss is not None:
                        if vl > self.prev_value_loss:
                            self.vl_increase_count += 1
                            if self.verbose > 0:
                                print(f"⚠️  VL 증가: {vl:.2f} (연속 {self.vl_increase_count}/{self.vl_consecutive}회)")
                        else:
                            self.vl_increase_count = 0  # 리셋
                    
                    self.prev_value_loss = vl
                    
                    # 5회 연속 증가 → 중단
                    if self.vl_increase_count >= self.vl_consecutive:
                        print(f"\n🚨 조기 경보 발동!")
                        print(f"   Value Loss가 {self.vl_consecutive}회 연속 증가")
                        print(f"   현재 VL: {vl:.2f}")
                        print(f"   🛑 학습 중단 및 체크포인트 저장\n")
                        
                        # 긴급 체크포인트 저장
                        self.model.save(f"logs/rl_training_sparse/emergency_checkpoint_{self.num_timesteps}.zip")
                        
                        self.warning_triggered = True
                        return False  # 학습 중단
        
        return True  # 학습 계속
    
    def _on_training_end(self) -> None:
        """학습 종료 시"""
        if self.warning_triggered:
            print("\n" + "=" * 60)
            print("⚠️  조기 경보로 학습이 중단되었습니다")
            print("=" * 60)
            print("📊 원인 분석:")
            print(f"   - EV 낮음 연속 횟수: {self.ev_low_count}")
            print(f"   - VL 증가 연속 횟수: {self.vl_increase_count}")
            print(f"\n💡 권장 사항:")
            print("   1. 보상 함수 재검토")
            print("   2. vf_coef 조정 고려")
            print("   3. 학습률 감소 고려")
            print("=" * 60)
