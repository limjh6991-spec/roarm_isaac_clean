#!/bin/bash
# 학습 진행 상황 모니터링 스크립트

LOG_DIR="logs/rl_training"
MONITOR_FILE="$LOG_DIR/monitor/0.monitor.csv"
CHECKPOINT_DIR="$LOG_DIR/checkpoints"

echo "📊 RoArm-M3 RL 학습 모니터링"
echo "================================================"
echo ""

# 학습 진행 상황 확인
check_training() {
    echo "🔍 학습 상태 확인..."
    echo ""
    
    # Python 프로세스 확인
    if pgrep -f "simple_train.py" > /dev/null; then
        echo "✅ 학습 진행 중"
        PID=$(pgrep -f "simple_train.py")
        echo "   PID: $PID"
        
        # CPU/메모리 사용량
        ps -p $PID -o %cpu,%mem,etime,cmd --no-headers
    else
        echo "❌ 학습 프로세스 없음"
    fi
    
    echo ""
}

# Monitor 파일 통계
show_monitor_stats() {
    if [ ! -f "$MONITOR_FILE" ]; then
        echo "⏳ Monitor 파일 대기 중..."
        return
    fi
    
    echo "📈 학습 통계 (Monitor 파일):"
    echo ""
    
    # 에피소드 수
    EPISODES=$(tail -n +2 "$MONITOR_FILE" | wc -l)
    echo "  총 에피소드: $EPISODES"
    
    if [ $EPISODES -gt 0 ]; then
        # 평균 보상
        AVG_REWARD=$(tail -n +2 "$MONITOR_FILE" | awk -F',' '{sum+=$1; count++} END {print sum/count}')
        echo "  평균 보상: $AVG_REWARD"
        
        # 최근 10개 에피소드 평균
        if [ $EPISODES -ge 10 ]; then
            RECENT_AVG=$(tail -10 "$MONITOR_FILE" | awk -F',' '{sum+=$1; count++} END {print sum/count}')
            echo "  최근 10개 평균: $RECENT_AVG"
        fi
        
        # 최고 보상
        MAX_REWARD=$(tail -n +2 "$MONITOR_FILE" | awk -F',' '{if(max<$1) max=$1} END {print max}')
        echo "  최고 보상: $MAX_REWARD"
    fi
    
    echo ""
}

# 체크포인트 확인
show_checkpoints() {
    if [ ! -d "$CHECKPOINT_DIR" ]; then
        echo "⏳ 체크포인트 대기 중..."
        return
    fi
    
    echo "💾 저장된 체크포인트:"
    echo ""
    
    CHECKPOINTS=$(ls -t "$CHECKPOINT_DIR"/*.zip 2>/dev/null | wc -l)
    if [ $CHECKPOINTS -gt 0 ]; then
        ls -lht "$CHECKPOINT_DIR"/*.zip | head -5 | awk '{print "  " $9 " (" $5 ", " $6 " " $7 " " $8 ")"}'
        if [ $CHECKPOINTS -gt 5 ]; then
            echo "  ... ($((CHECKPOINTS - 5))개 더)"
        fi
    else
        echo "  아직 없음"
    fi
    
    echo ""
}

# 시각화 안내
show_visualization_info() {
    echo "📊 학습 진행 상황 시각화:"
    echo ""
    echo "  # 학습 곡선 이미지 생성:"
    echo "  ~/isaacsim/python.sh scripts/plot_training.py"
    echo ""
    echo "  # 이미지 확인:"
    echo "  xdg-open logs/rl_training/training_progress.png"
    echo ""
    echo "  💡 TIP: 이미지가 자동으로 열립니다."
    echo "      에피소드별 보상, 길이, 통계를 한눈에 확인!"
    echo ""
}

# 메인 루프
main() {
    while true; do
        clear
        echo "📊 RoArm-M3 RL 학습 모니터링"
        echo "================================================"
        echo "업데이트 시간: $(date '+%Y-%m-%d %H:%M:%S')"
        echo ""
        
        check_training
        show_monitor_stats
        show_checkpoints
        show_visualization_info
        
        echo "================================================"
        echo "자동 새로고침: 10초마다 (Ctrl+C로 종료)"
        
        # 10초 대기
        sleep 10
    done
}

# 인자 확인
if [ "$1" == "--once" ]; then
    # 1회만 실행
    check_training
    show_monitor_stats
    show_checkpoints
    show_visualization_info
else
    # 지속적으로 모니터링
    main
fi
