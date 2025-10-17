#!/usr/bin/env bash
# ==============================================================================
# Master Preflight Check
# 모든 Preflight 검사를 순차적으로 실행
# ==============================================================================

set -u  # -e는 각 스텝 내부에서 제어
IFS=$'\n\t'

# Colors
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
printf "========================================================================\n"
printf "              Master Preflight Check\n"
printf "========================================================================\n\n"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT" || exit 2
mkdir -p logs/preflight
TS="$(date +'%Y%m%d_%H%M%S')"
LOG="logs/preflight/preflight_${TS}.log"
JSON="logs/preflight/preflight_${TS}.json"

# 스텝 정의: [이름|스크립트 경로|타임아웃(초)]
STEPS=(
  "System Check|devops/preflight/check_system.sh|30"
  "Isaac Extensions|devops/preflight/check_isaac_extensions.py|120"
  "USD Integrity|devops/preflight/check_usd_integrity.sh|45"
)

# 집계
PASS=0; FAIL=0; WARN=0; SKIP=0
RESULTS=()

run_step () {
  local name="$1" path="$2" to="$3"
  
  # Python 스크립트 확인
  if [[ "$path" == *.py ]]; then
    if [[ ! -f "$path" ]]; then
      printf "${YELLOW}[SKIP]${NC} %s (script not found: %s)\n" "$name" "$path" | tee -a "$LOG"
      SKIP=$((SKIP+1))
      RESULTS+=("{\"name\":\"$name\",\"status\":\"SKIP\"}")
      return 0
    fi
  elif [[ ! -x "$path" ]]; then
    printf "${YELLOW}[SKIP]${NC} %s (script not found: %s)\n" "$name" "$path" | tee -a "$LOG"
    SKIP=$((SKIP+1))
    RESULTS+=("{\"name\":\"$name\",\"status\":\"SKIP\"}")
    return 0
  fi

  printf "[RUN] %s (timeout=%ss)\n" "$name" "$to" | tee -a "$LOG"
  set +e
  if [[ "$path" == *.py ]]; then
    timeout "$to" bash -c "source ~/isaacsim-venv/bin/activate && python \"$path\" 2>/dev/null" | tee -a "$LOG"
  else
    timeout "$to" bash "$path" 2>&1 | tee -a "$LOG"
  fi
  rc=${PIPESTATUS[0]}
  set -e

  case "$rc" in
    0)
      printf "${GREEN}[PASS]${NC} %s\n\n" "$name" | tee -a "$LOG"
      PASS=$((PASS+1)); RESULTS+=("{\"name\":\"$name\",\"status\":\"PASS\"}")
      ;;
    124)
      printf "${RED}[TIMEOUT]${NC} %s\n\n" "$name" | tee -a "$LOG"
      FAIL=$((FAIL+1)); RESULTS+=("{\"name\":\"$name\",\"status\":\"TIMEOUT\"}")
      ;;
    *)
      printf "${RED}[FAIL]${NC} %s (rc=%s)\n\n" "$name" "$rc" | tee -a "$LOG"
      FAIL=$((FAIL+1)); RESULTS+=("{\"name\":\"$name\",\"status\":\"FAIL\",\"rc\":$rc}")
      ;;
  esac
}

i=1; total=${#STEPS[@]}
for s in "${STEPS[@]}"; do
  name="${s%%|*}"; rest="${s#*|}"; path="${rest%%|*}"; to="${rest##*|}"
  printf "[%s/%s] %s\n" "$i" "$total" "$name" | tee -a "$LOG"
  run_step "$name" "$path" "$to"
  i=$((i+1))
done

printf "========================================================================\n"
printf "Total: %s  ${GREEN}PASS:%s${NC}  ${RED}FAIL:%s${NC}  ${YELLOW}SKIP:%s${NC}\n" "$total" "$PASS" "$FAIL" "$SKIP" | tee -a "$LOG"
[[ $FAIL -gt 0 ]] && STATUS="FAIL" || STATUS="PASS"

# JSON 아카이브
printf '{"timestamp":"%s","status":"%s","summary":{"total":%d,"pass":%d,"fail":%d,"skip":%d},"steps":[%s]}\n' \
  "$TS" "$STATUS" "$total" "$PASS" "$FAIL" "$SKIP" "$(IFS=, ; echo "${RESULTS[*]}")" > "$JSON"

if [[ "$STATUS" == "FAIL" ]]; then
  printf "${RED}PREFLIGHT FAILED!${NC} Fix errors above before proceeding.\n"
  exit 1
else
  printf "${GREEN}ALL PREFLIGHT CHECKS PASSED!${NC}\nSystem ready for Isaac Sim development.\n"
  exit 0
fi
