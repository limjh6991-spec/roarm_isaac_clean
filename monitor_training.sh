#!/bin/bash
# 10M 학습 모니터링 스크립트

echo "🔍 10M 학습 모니터링"
echo "=" * 60

# 프로세스 확인
echo "📊 프로세스 상태:"
ps aux | grep -E "python.sh.*train_dense" | grep -v grep | head -1

echo ""
echo "📈 최근 로그 (마지막 30줄):"
tail -30 logs/training_10M_resume.log

echo ""
echo "💾 최신 체크포인트:"
ls -lht logs/rl_training_curriculum/checkpoints/*.zip | head -3

echo ""
echo "📊 에피소드 통계:"
tail -5 logs/rl_training_curriculum/monitor.monitor.csv

