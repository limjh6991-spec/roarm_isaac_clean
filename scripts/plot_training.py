#!/home/roarm_m3/isaacsim/python.sh
"""
간단한 학습 진행 상황 플롯 생성 스크립트
TensorBoard 대신 matplotlib로 학습 곡선 시각화
"""

import os
import sys
import csv
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def load_monitor_data(monitor_file):
    """Monitor CSV 파일 로드"""
    episodes = []
    rewards = []
    lengths = []
    times = []
    
    with open(monitor_file, 'r') as f:
        # 첫 2줄 건너뛰기 (메타데이터 + 헤더)
        next(f)
        next(f)
        reader = csv.reader(f)
        for row in reader:
            if len(row) >= 3:
                rewards.append(float(row[0]))
                lengths.append(float(row[1]))
                times.append(float(row[2]))
                episodes.append(len(episodes) + 1)
    
    return episodes, rewards, lengths, times

def moving_average(data, window=10):
    """이동 평균 계산"""
    if len(data) < window:
        return data
    return np.convolve(data, np.ones(window)/window, mode='valid')

def plot_training_progress(monitor_file, output_dir):
    """학습 진행 상황 플롯"""
    episodes, rewards, lengths, times = load_monitor_data(monitor_file)
    
    if len(episodes) == 0:
        print("❌ 에피소드 데이터가 없습니다")
        return
    
    # 플롯 생성
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('RoArm-M3 RL 학습 진행 상황', fontsize=16, fontweight='bold')
    
    # 1. 에피소드별 보상
    ax1 = axes[0, 0]
    ax1.plot(episodes, rewards, alpha=0.3, color='blue', label='에피소드 보상')
    if len(rewards) >= 10:
        ma_rewards = moving_average(rewards, window=10)
        ma_episodes = episodes[9:]  # 이동 평균 길이에 맞춤
        ax1.plot(ma_episodes, ma_rewards, color='red', linewidth=2, label='이동 평균 (10)')
    ax1.set_xlabel('에피소드')
    ax1.set_ylabel('보상')
    ax1.set_title('에피소드별 보상')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2. 에피소드별 길이
    ax2 = axes[0, 1]
    ax2.plot(episodes, lengths, alpha=0.3, color='green', label='에피소드 길이')
    if len(lengths) >= 10:
        ma_lengths = moving_average(lengths, window=10)
        ma_episodes = episodes[9:]
        ax2.plot(ma_episodes, ma_lengths, color='orange', linewidth=2, label='이동 평균 (10)')
    ax2.set_xlabel('에피소드')
    ax2.set_ylabel('스텝 수')
    ax2.set_title('에피소드별 길이')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. 누적 시간
    ax3 = axes[1, 0]
    cumulative_time = np.cumsum(times)
    ax3.plot(episodes, cumulative_time / 60, color='purple')
    ax3.set_xlabel('에피소드')
    ax3.set_ylabel('누적 시간 (분)')
    ax3.set_title('누적 학습 시간')
    ax3.grid(True, alpha=0.3)
    
    # 4. 통계 요약
    ax4 = axes[1, 1]
    ax4.axis('off')
    
    # 최근 10개 에피소드 통계
    recent_rewards = rewards[-10:] if len(rewards) >= 10 else rewards
    
    stats_text = f"""
    📊 학습 통계
    ━━━━━━━━━━━━━━━━━━━━━━━━━━
    
    총 에피소드: {len(episodes)}
    총 학습 시간: {cumulative_time[-1]/60:.1f}분
    
    전체 보상:
      평균: {np.mean(rewards):.1f}
      최고: {np.max(rewards):.1f}
      최저: {np.min(rewards):.1f}
    
    최근 10개 에피소드:
      평균 보상: {np.mean(recent_rewards):.1f}
      평균 길이: {np.mean(lengths[-10:] if len(lengths)>=10 else lengths):.0f}
    
    개선도:
      초기 10개 평균: {np.mean(rewards[:10]):.1f}
      최근 10개 평균: {np.mean(recent_rewards):.1f}
      변화: {np.mean(recent_rewards) - np.mean(rewards[:10]):.1f}
    """
    
    ax4.text(0.1, 0.5, stats_text, fontsize=11, family='monospace',
             verticalalignment='center')
    
    # 저장
    plt.tight_layout()
    output_file = os.path.join(output_dir, 'training_progress.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✅ 플롯 저장 완료: {output_file}")
    
    # 통계 출력
    print("\n" + "="*50)
    print(stats_text)
    print("="*50)

def main():
    # 파일 경로
    project_root = Path(__file__).parent.parent
    monitor_file = project_root / "logs" / "rl_training" / "monitor.monitor.csv"
    output_dir = project_root / "logs" / "rl_training"
    
    if not monitor_file.exists():
        print(f"❌ Monitor 파일을 찾을 수 없습니다: {monitor_file}")
        print("   학습이 아직 시작되지 않았거나 데이터가 없습니다.")
        return
    
    print(f"📊 학습 데이터 분석 중...")
    print(f"   Monitor 파일: {monitor_file}")
    
    plot_training_progress(str(monitor_file), str(output_dir))
    
    print(f"\n📁 결과 이미지: {output_dir}/training_progress.png")

if __name__ == "__main__":
    main()
