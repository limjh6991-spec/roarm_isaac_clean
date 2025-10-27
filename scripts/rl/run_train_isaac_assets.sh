#!/bin/bash
# Isaac Assets 기반 Pick and Place 학습/테스트 스크립트

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# 기본값
MODE="train"
LEVEL="easy"
TIMESTEPS=50000
EPISODES=10
RENDER="true"

# 도움말
show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Isaac Assets 기반 RoArm Pick and Place 강화학습

OPTIONS:
    --mode MODE          실행 모드: train, test, demo (기본: train)
    --level LEVEL        난이도: easy, medium, hard, mixed (기본: easy)
    --timesteps STEPS    학습 스텝 수 (기본: 50000)
    --episodes N         테스트 에피소드 수 (기본: 10)
    --no-render          Headless 모드 (렌더링 끄기)
    --warehouse          Warehouse 배경 사용
    -h, --help           이 도움말 표시

EXAMPLES:
    # Easy mode 학습 (50K steps)
    $0 --mode train --level easy --timesteps 50000
    
    # Medium mode 학습 (100K steps, headless)
    $0 --mode train --level medium --timesteps 100000 --no-render
    
    # Warehouse 환경에서 학습
    $0 --mode train --level medium --warehouse
    
    # 학습된 모델 테스트 (10 episodes)
    $0 --mode test --episodes 10
    
    # 환경 데모 (랜덤 액션)
    $0 --mode demo --level easy

CURRICULUM LEARNING 권장 순서:
    1. Easy (50K):    ./run_train_isaac_assets.sh --mode train --level easy --timesteps 50000
    2. Medium (100K): ./run_train_isaac_assets.sh --mode train --level medium --timesteps 100000
    3. Hard (50K):    ./run_train_isaac_assets.sh --mode train --level hard --timesteps 50000 --warehouse

EOF
}

# 인자 파싱
WAREHOUSE="false"
while [[ $# -gt 0 ]]; do
    case $1 in
        --mode)
            MODE="$2"
            shift 2
            ;;
        --level)
            LEVEL="$2"
            shift 2
            ;;
        --timesteps)
            TIMESTEPS="$2"
            shift 2
            ;;
        --episodes)
            EPISODES="$2"
            shift 2
            ;;
        --no-render)
            RENDER="false"
            shift
            ;;
        --warehouse)
            WAREHOUSE="true"
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# 환경 변수 설정
export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"

# 로그 디렉토리
LOG_DIR="$PROJECT_ROOT/logs/rl_training_isaac"
mkdir -p "$LOG_DIR"

echo "=========================================="
echo "🤖 RoArm Pick & Place (Isaac Assets)"
echo "=========================================="
echo "Mode:       $MODE"
echo "Level:      $LEVEL"
echo "Render:     $RENDER"
echo "Warehouse:  $WAREHOUSE"
if [ "$MODE" == "train" ]; then
    echo "Timesteps:  $TIMESTEPS"
elif [ "$MODE" == "test" ]; then
    echo "Episodes:   $EPISODES"
fi
echo "=========================================="
echo ""

# Python 스크립트 실행
PYTHON_SCRIPT="$PROJECT_ROOT/scripts/train_roarm_isaac_assets.py"

if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "❌ 스크립트를 찾을 수 없습니다: $PYTHON_SCRIPT"
    echo "먼저 train_roarm_isaac_assets.py를 생성하세요."
    exit 1
fi

# Isaac Sim 실행
ISAAC_SIM_CMD="$HOME/isaac-sim.sh"

if [ ! -f "$ISAAC_SIM_CMD" ]; then
    echo "❌ Isaac Sim을 찾을 수 없습니다: $ISAAC_SIM_CMD"
    exit 1
fi

# 명령어 구성
CMD_ARGS="--mode $MODE --level $LEVEL --render $RENDER --warehouse $WAREHOUSE"

if [ "$MODE" == "train" ]; then
    CMD_ARGS="$CMD_ARGS --timesteps $TIMESTEPS"
elif [ "$MODE" == "test" ]; then
    CMD_ARGS="$CMD_ARGS --episodes $EPISODES"
fi

echo "🚀 Starting Isaac Sim..."
echo "Command: $ISAAC_SIM_CMD $PYTHON_SCRIPT $CMD_ARGS"
echo ""

exec "$ISAAC_SIM_CMD" "$PYTHON_SCRIPT" $CMD_ARGS
