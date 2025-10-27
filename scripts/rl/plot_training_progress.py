#!/usr/bin/env python3
"""
학습 진행 상황 모니터링 및 그래프 생성
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import seaborn as sns

# 스타일 설정
sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (15, 10)
plt.rcParams['font.size'] = 10

def plot_training_progress(log_dir: str = "logs/rl_training_curriculum"):
    """학습 진행 상황 그래프 생성"""
    log_dir = Path(log_dir)
    
    # Monitor CSV 파일 읽기
    monitor_file = log_dir / "monitor.monitor.csv"
    
    if not monitor_file.exists():
        print(f"❌ Monitor 파일을 찾을 수 없습니다: {monitor_file}")
        return
    
    # CSV 읽기 (첫 줄은 메타데이터)
    df = pd.read_csv(monitor_file, skiprows=1)
    
    print(f"✅ 데이터 로드 완료: {len(df)} 에피소드")
    print(f"   컬럼: {df.columns.tolist()}")
    
    # 이동 평균 계산
    window = 10
    df['reward_ma'] = df['r'].rolling(window=window, min_periods=1).mean()
    df['length_ma'] = df['l'].rolling(window=window, min_periods=1).mean()
    
    # 그래프 생성
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('RoArm-M3 Pick and Place Training Progress', fontsize=16, fontweight='bold')
    
    # 1. Episode Reward
    ax = axes[0, 0]
    ax.plot(df.index, df['r'], alpha=0.3, color='blue', label='Episode Reward')
    ax.plot(df.index, df['reward_ma'], color='red', linewidth=2, label=f'MA({window})')
    ax.set_xlabel('Episode')
    ax.set_ylabel('Reward')
    ax.set_title('Episode Reward Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 2. Episode Length
    ax = axes[0, 1]
    ax.plot(df.index, df['l'], alpha=0.3, color='green', label='Episode Length')
    ax.plot(df.index, df['length_ma'], color='red', linewidth=2, label=f'MA({window})')
    ax.set_xlabel('Episode')
    ax.set_ylabel('Steps')
    ax.set_title('Episode Length Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 3. Cumulative Time
    ax = axes[1, 0]
    ax.plot(df.index, df['t'], color='purple', linewidth=2)
    ax.set_xlabel('Episode')
    ax.set_ylabel('Time (seconds)')
    ax.set_title('Cumulative Training Time')
    ax.grid(True, alpha=0.3)
    
    # 4. Reward Distribution (최근 100 에피소드)
    ax = axes[1, 1]
    recent_rewards = df['r'].tail(min(100, len(df)))
    ax.hist(recent_rewards, bins=30, color='orange', alpha=0.7, edgecolor='black')
    ax.axvline(recent_rewards.mean(), color='red', linestyle='--', linewidth=2, 
               label=f'Mean: {recent_rewards.mean():.2f}')
    ax.set_xlabel('Reward')
    ax.set_ylabel('Frequency')
    ax.set_title(f'Reward Distribution (Last {len(recent_rewards)} Episodes)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # 저장
    output_path = log_dir / "training_progress.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\n💾 그래프 저장: {output_path}")
    
    # 통계 출력
    print("\n" + "="*80)
    print("📊 학습 통계")
    print("="*80)
    print(f"총 에피소드: {len(df)}")
    print(f"총 학습 시간: {df['t'].iloc[-1]:.1f}초 ({df['t'].iloc[-1]/60:.1f}분)")
    print(f"\n보상 통계:")
    print(f"  평균: {df['r'].mean():.2f}")
    print(f"  최대: {df['r'].max():.2f}")
    print(f"  최소: {df['r'].min():.2f}")
    print(f"  최근 100 에피소드 평균: {df['r'].tail(100).mean():.2f}")
    print(f"\n에피소드 길이:")
    print(f"  평균: {df['l'].mean():.0f} steps")
    print(f"  최대: {df['l'].max():.0f} steps")
    print(f"  최소: {df['l'].min():.0f} steps")
    print("="*80)
    
    plt.show()


def plot_tensorboard_data(log_dir: str = "logs/rl_training_curriculum/tensorboard"):
    """TensorBoard 데이터 시각화"""
    from tensorboard.backend.event_processing import event_accumulator
    
    log_dir = Path(log_dir)
    
    # Event 파일 찾기
    event_files = list(log_dir.rglob("events.out.tfevents.*"))
    
    if not event_files:
        print(f"❌ TensorBoard 이벤트 파일을 찾을 수 없습니다: {log_dir}")
        return
    
    print(f"✅ TensorBoard 데이터 로드 중...")
    
    ea = event_accumulator.EventAccumulator(str(event_files[0]))
    ea.Reload()
    
    # 사용 가능한 태그
    print(f"   사용 가능한 태그: {ea.Tags()}")
    
    # Milestone 데이터 플롯
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Training Milestones and Metrics', fontsize=16, fontweight='bold')
    
    milestones = ['milestone/reach_rate', 'milestone/grip_rate', 
                  'milestone/lift_rate', 'milestone/place_rate']
    
    for idx, tag in enumerate(milestones):
        if tag in ea.Tags()['scalars']:
            data = ea.Scalars(tag)
            steps = [d.step for d in data]
            values = [d.value for d in data]
            
            ax = axes[idx // 2, idx % 2]
            ax.plot(steps, values, linewidth=2)
            ax.set_xlabel('Timesteps')
            ax.set_ylabel('Rate (%)')
            ax.set_title(tag.replace('milestone/', '').replace('_', ' ').title())
            ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # 저장
    output_path = log_dir.parent / "milestones.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\n💾 Milestone 그래프 저장: {output_path}")
    
    plt.show()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Plot training progress")
    parser.add_argument(
        "--log-dir",
        type=str,
        default="logs/rl_training_curriculum",
        help="로그 디렉토리"
    )
    parser.add_argument(
        "--tensorboard",
        action="store_true",
        help="TensorBoard 데이터도 플롯"
    )
    
    args = parser.parse_args()
    
    # Monitor 데이터 플롯
    plot_training_progress(args.log_dir)
    
    # TensorBoard 데이터 플롯 (선택)
    if args.tensorboard:
        try:
            plot_tensorboard_data(f"{args.log_dir}/tensorboard")
        except Exception as e:
            print(f"⚠️ TensorBoard 데이터 플롯 실패: {e}")


if __name__ == "__main__":
    main()
