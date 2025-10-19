#!/bin/bash
# RoArm-M3 강화학습 환경 설정 스크립트

echo "🚀 RoArm-M3 강화학습 환경 설정"
echo "================================================"

# Isaac Sim Python 환경 확인
if [ ! -d "$HOME/isaacsim" ]; then
    echo "❌ Isaac Sim이 설치되지 않았습니다: ~/isaacsim"
    exit 1
fi

echo "✅ Isaac Sim 발견: $HOME/isaacsim"

# 필요한 패키지 설치
echo ""
echo "📦 필수 패키지 설치 중..."

# Python 패키지 목록
PACKAGES=(
    "torch"
    "stable-baselines3[extra]"
    "gymnasium"
    "tensorboard"
)

for pkg in "${PACKAGES[@]}"; do
    echo "  - Installing $pkg..."
    pip install --quiet "$pkg"
    if [ $? -eq 0 ]; then
        echo "    ✅ $pkg installed"
    else
        echo "    ❌ $pkg installation failed"
    fi
done

echo ""
echo "✅ 환경 설정 완료!"
echo ""
echo "📝 사용 방법:"
echo "  # 학습"
echo "  ~/isaac-sim.sh -m scripts/train_roarm_rl.py --mode train --timesteps 50000"
echo ""
echo "  # 테스트"
echo "  ~/isaac-sim.sh -m scripts/train_roarm_rl.py --mode test --model logs/rl_training/best_model/best_model.zip"
echo ""
