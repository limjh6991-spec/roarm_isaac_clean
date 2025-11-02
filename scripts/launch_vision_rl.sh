#!/bin/bash
################################################################################
# Vision RL Training Launcher
# 
# Usage:
#   bash scripts/launch_vision_rl.sh
#
# Options:
#   --test     : Test environment only (10 episodes)
#   --train    : Full training (500K steps, ~5-10 hours)
#   --quick    : Quick test (50K steps, ~30 min)
#
# 날짜: 2025-11-02
################################################################################

set -e  # Exit on error

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project root
PROJECT_DIR="/home/roarm_m3/roarm_isaac_clean"
ISAAC_PYTHON="/home/roarm_m3/isaacsim/python.sh"

echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}🚀 Vision RL Training Launcher${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""

# Check Isaac Sim
if [ ! -f "$ISAAC_PYTHON" ]; then
    echo -e "${RED}❌ Error: Isaac Sim not found at $ISAAC_PYTHON${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Isaac Sim found${NC}"

# Parse arguments
MODE="${1:---train}"

case "$MODE" in
    --test)
        echo -e "${YELLOW}📋 Mode: Environment Test${NC}"
        echo -e "   - 10 random episodes"
        echo -e "   - Observation validation"
        echo -e "   - Time: ~2 minutes"
        echo ""
        
        read -p "Press Enter to start test..."
        
        cd "$PROJECT_DIR"
        $ISAAC_PYTHON scripts/test/test_vision_env.py
        ;;
        
    --quick)
        echo -e "${YELLOW}📋 Mode: Quick Training${NC}"
        echo -e "   - Algorithm: SAC"
        echo -e "   - Steps: 50K"
        echo -e "   - Time: ~30 minutes"
        echo -e "   - Checkpoints: every 10K"
        echo ""
        
        read -p "Press Enter to start training..."
        
        cd "$PROJECT_DIR"
        $ISAAC_PYTHON scripts/train/train_vision_sac.py --steps 50000
        ;;
        
    --train)
        echo -e "${YELLOW}📋 Mode: Full Training${NC}"
        echo -e "   - Algorithm: SAC"
        echo -e "   - Steps: 500K"
        echo -e "   - Time: ~5-10 hours"
        echo -e "   - Checkpoints: every 10K"
        echo -e "   - Evaluation: every 10K (10 episodes)"
        echo ""
        echo -e "${GREEN}💡 Monitor progress:${NC}"
        echo -e "   tensorboard --logdir output/train_vision_sac/"
        echo ""
        
        read -p "Press Enter to start training..."
        
        cd "$PROJECT_DIR"
        $ISAAC_PYTHON scripts/train/train_vision_sac.py
        ;;
        
    *)
        echo -e "${RED}❌ Unknown mode: $MODE${NC}"
        echo ""
        echo "Usage: bash scripts/launch_vision_rl.sh [--test|--quick|--train]"
        echo ""
        echo "Options:"
        echo "  --test     Environment test (10 episodes, ~2 min)"
        echo "  --quick    Quick training (50K steps, ~30 min)"
        echo "  --train    Full training (500K steps, ~5-10 hours)"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}======================================================================${NC}"
echo -e "${GREEN}✅ Completed!${NC}"
echo -e "${GREEN}======================================================================${NC}"
