.PHONY: help preflight setup-all train train-quick isaac-gui diagnose clean

# =============================================================================
# RoArm M3 Makefile - 통합 실행 명령
# =============================================================================

# 색상 정의
BLUE := \033[0;34m
GREEN := \033[0;32m
YELLOW := \033[1;33m
NC := \033[0m # No Color

# 환경 변수 로드
include .env
export

# =============================================================================
# 기본 타겟
# =============================================================================

help: ## 사용 가능한 명령어 표시
	@echo "$(BLUE)╔═══════════════════════════════════════════════════════════╗$(NC)"
	@echo "$(BLUE)║              RoArm M3 Makefile Commands                   ║$(NC)"
	@echo "$(BLUE)╚═══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@echo "$(GREEN)📋 Environment:$(NC)"
	@awk 'BEGIN {FS = ":.*##"; category=""} \
		/^## / { category=substr($$0,4); print "\n$(YELLOW)" category "$(NC)"} \
		/^[a-zA-Z_-]+:.*?##/ { if (category) printf "  $(GREEN)%-20s$(NC) %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""

.DEFAULT_GOAL := help

# =============================================================================
## 환경 관리
# =============================================================================

preflight: ## ⭐ 환경 검증 (매일 첫 실행)
	@echo "$(BLUE)Running preflight checks...$(NC)"
	@bash scripts/env_setup/preflight.sh

setup-all: setup-dirs create-env ## 전체 환경 초기 설정
	@echo "$(GREEN)✅ Setup complete!$(NC)"
	@echo "$(YELLOW)Next: make preflight$(NC)"

setup-dirs: ## 필요한 디렉토리 생성
	@echo "$(BLUE)Creating directories...$(NC)"
	@mkdir -p envs data logs models/checkpoints models/backup
	@mkdir -p scripts/env_setup scripts/isaac
	@echo "$(GREEN)✅ Directories created$(NC)"

create-env: ## .env 파일 생성 (없을 경우)
	@if [ ! -f .env ]; then \
		cp .env.example .env; \
		echo "$(GREEN)✅ Created .env from .env.example$(NC)"; \
		echo "$(YELLOW)⚠️  Edit .env to match your paths$(NC)"; \
	else \
		echo "$(GREEN)✅ .env already exists$(NC)"; \
	fi

# =============================================================================
## Isaac Sim 실행
# =============================================================================

isaac-gui: ## Isaac Sim GUI 실행
	@echo "$(BLUE)Launching Isaac Sim GUI...$(NC)"
	@$(ISAAC_PYTHON) scripts/rl/test_trained_model.py --episodes 1

isaac-headless: ## Isaac Sim Headless 실행
	@echo "$(BLUE)Launching Isaac Sim Headless...$(NC)"
	@ISAAC_HEADLESS=true $(ISAAC_PYTHON) scripts/rl/train_dense_reward.py

# =============================================================================
## 강화학습 훈련
# =============================================================================

train-quick: preflight ## 빠른 테스트 (10K steps, ~15분)
	@echo "$(BLUE)Starting quick training (10K steps)...$(NC)"
	@PYTHONUNBUFFERED=1 $(ISAAC_PYTHON) scripts/rl/train_dense_reward.py \
		--timesteps 10000 \
		2>&1 | tee logs/train_quick_$(shell date +%Y%m%d_%H%M%S).log
	@echo "$(GREEN)✅ Training complete! Check logs/$(NC)"

train: preflight ## 정규 훈련 (50K steps, ~1시간)
	@echo "$(BLUE)Starting training (50K steps)...$(NC)"
	@PYTHONUNBUFFERED=1 $(ISAAC_PYTHON) scripts/rl/train_dense_reward.py \
		--timesteps 50000 \
		2>&1 | tee logs/train_50k_$(shell date +%Y%m%d_%H%M%S).log
	@echo "$(GREEN)✅ Training complete! Check logs/$(NC)"

train-long: preflight ## 장기 훈련 (500K steps, ~10시간)
	@echo "$(BLUE)Starting long training (500K steps)...$(NC)"
	@PYTHONUNBUFFERED=1 $(ISAAC_PYTHON) scripts/rl/train_dense_reward.py \
		--timesteps 500000 \
		2>&1 | tee logs/train_500k_$(shell date +%Y%m%d_%H%M%S).log
	@echo "$(GREEN)✅ Training complete! Check logs/$(NC)"

train-1m: preflight ## 최종 훈련 (1M steps, ~20시간)
	@echo "$(BLUE)Starting final training (1M steps)...$(NC)"
	@PYTHONUNBUFFERED=1 $(ISAAC_PYTHON) scripts/rl/train_dense_reward.py \
		--timesteps 1000000 \
		2>&1 | tee logs/train_1m_$(shell date +%Y%m%d_%H%M%S).log
	@echo "$(GREEN)✅ Training complete! Check logs/$(NC)"

play: ## 학습된 모델 재생 (GUI)
	@echo "$(BLUE)Playing trained model...$(NC)"
	@$(ISAAC_PYTHON) scripts/rl/test_trained_model.py \
		--model models/ppo_roarm_latest.zip \
		--episodes 10

# =============================================================================
## 진단 및 분석
# =============================================================================

diagnose: ## 환경 진단 (7단계 체크)
	@echo "$(BLUE)Running environment diagnostics...$(NC)"
	@PYTHONUNBUFFERED=1 $(ISAAC_PYTHON) scripts/rl/diagnose_env.py --full

diagnose-obs: ## Observation 체크
	@PYTHONUNBUFFERED=1 $(ISAAC_PYTHON) scripts/rl/diagnose_env.py --check observation

diagnose-reward: ## Reward 체크
	@PYTHONUNBUFFERED=1 $(ISAAC_PYTHON) scripts/rl/diagnose_env.py --check reward --verbose

tensorboard: ## TensorBoard 실행
	@echo "$(BLUE)Starting TensorBoard...$(NC)"
	@echo "$(YELLOW)Open: http://localhost:6006$(NC)"
	@tensorboard --logdir=$(TENSORBOARD_LOG_DIR) --port=6006

# =============================================================================
## 의존성 관리
# =============================================================================

lock: ## requirements 업데이트 (pip-compile)
	@echo "$(BLUE)Updating requirements...$(NC)"
	@if [ -d "$(RL_VENV)" ]; then \
		source $(RL_VENV)/bin/activate && \
		pip-compile requirements/rl.in -o requirements/rl.txt; \
		echo "$(GREEN)✅ requirements/rl.txt updated$(NC)"; \
	else \
		echo "$(YELLOW)⚠️  RL venv not found. Run: make setup-rl$(NC)"; \
	fi

freeze: ## 현재 설치된 패키지 목록
	@echo "$(BLUE)Installed packages:$(NC)"
	@$(ISAAC_PYTHON) -m pip list

# =============================================================================
## 정리 및 유지보수
# =============================================================================

clean: ## 로그 및 캐시 삭제
	@echo "$(BLUE)Cleaning logs and caches...$(NC)"
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@rm -rf logs/*.log 2>/dev/null || true
	@echo "$(GREEN)✅ Cleaned$(NC)"

clean-models: ## 오래된 모델 백업 후 삭제
	@echo "$(BLUE)Backing up and cleaning old models...$(NC)"
	@mkdir -p models/backup
	@find models -name "*.zip" -mtime +7 -exec mv {} models/backup/ \; 2>/dev/null || true
	@echo "$(GREEN)✅ Old models moved to backup/$(NC)"

clean-all: clean clean-models ## 전체 정리 (venv 제외)
	@echo "$(BLUE)Deep cleaning...$(NC)"
	@rm -rf logs/* data/temp/* 2>/dev/null || true
	@echo "$(GREEN)✅ Deep clean complete$(NC)"

# =============================================================================
## 백업 및 복원
# =============================================================================

backup: ## 모델 및 설정 백업
	@echo "$(BLUE)Creating backup...$(NC)"
	@mkdir -p backups
	@tar -czf backups/roarm_backup_$(shell date +%Y%m%d_%H%M%S).tar.gz \
		models/*.zip \
		.env \
		configs/ \
		docs/ \
		2>/dev/null || true
	@echo "$(GREEN)✅ Backup created in backups/$(NC)"

# =============================================================================
## Git 관리
# =============================================================================

git-status: ## Git 상태 확인
	@git status

git-diff: ## 변경 사항 확인
	@git diff

commit-docs: ## 문서만 커밋
	@git add docs/
	@git commit -m "docs: Update documentation"
	@echo "$(GREEN)✅ Documentation committed$(NC)"

# =============================================================================
## 테스트
# =============================================================================

test-env: preflight ## 환경 테스트
	@echo "$(BLUE)Testing environment...$(NC)"
	@$(ISAAC_PYTHON) -c "import torch; print(f'PyTorch: {torch.__version__}')"
	@$(ISAAC_PYTHON) -c "import numpy; print(f'NumPy: {numpy.__version__}')"
	@echo "$(GREEN)✅ Environment test passed$(NC)"

test-urdf: ## URDF 검증
	@echo "$(BLUE)Testing URDF...$(NC)"
	@$(ISAAC_PYTHON) scripts/urdf/test_improved_urdf.py

# =============================================================================
## 문서
# =============================================================================

docs: ## 문서 목록 표시
	@echo "$(BLUE)📚 Available Documentation:$(NC)"
	@echo ""
	@echo "$(GREEN)Environment:$(NC)"
	@echo "  docs/environment/ENVIRONMENT_SETUP.md"
	@echo "  docs/environment/DAILY_CHECKLIST.md"
	@echo "  docs/environment/TROUBLESHOOTING.md"
	@echo ""
	@echo "$(GREEN)Reinforcement Learning:$(NC)"
	@echo "  docs/rl/README.md"
	@echo "  docs/rl/RL_TRAINING_PLAN_V2.md"
	@echo "  docs/rl/DIAGNOSTIC_CHECKLIST.md"
	@echo "  docs/rl/REWARD_DESIGN_GUIDE.md"
	@echo "  docs/rl/PPO_HYPERPARAMETERS_CHEATSHEET.md"
	@echo "  docs/rl/SUCCESS_CASES.md"

checklist: ## 매일 체크리스트 표시
	@cat docs/environment/DAILY_CHECKLIST.md

# =============================================================================
## 개발 워크플로우
# =============================================================================

morning: preflight docs ## 아침 루틴 (preflight + 문서)
	@echo ""
	@echo "$(GREEN)Good morning! Ready to work.$(NC)"
	@echo "$(YELLOW)Today's tasks:$(NC)"
	@echo "  1. Check yesterday's training logs"
	@echo "  2. Review TensorBoard: make tensorboard"
	@echo "  3. Continue training or adjust hyperparameters"

quick-test: train-quick ## 빠른 테스트 (train-quick의 alias)

full-workflow: preflight train tensorboard ## 전체 워크플로우
	@echo "$(GREEN)✅ Full workflow executed$(NC)"

# =============================================================================
# 내부 헬퍼
# =============================================================================

.check-env:
	@if [ ! -f .env ]; then \
		echo "$(YELLOW)⚠️  .env not found. Run: make create-env$(NC)"; \
		exit 1; \
	fi
