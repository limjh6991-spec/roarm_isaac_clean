#!/usr/bin/env bash
# ==============================================================================
# Master Preflight Check
# 모든 Preflight 검사를 순차적으로 실행
# ==============================================================================

set -Eeuo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

echo -e "${GREEN}========================================================================"
echo "              Master Preflight Check"
echo "========================================================================${NC}"
echo ""

TOTAL_CHECKS=3
PASSED=0
FAILED=0

# ------------------------------------------------------------------------------
# 1. System Check
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[1/$TOTAL_CHECKS] System Check (GPU, Driver, Vulkan, Python)${NC}"
if bash devops/preflight/check_system.sh; then
    ((PASSED++))
else
    ((FAILED++))
fi
echo ""

# ------------------------------------------------------------------------------
# 2. Isaac Sim Extensions Check
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[2/$TOTAL_CHECKS] Isaac Sim Extensions Check${NC}"
if python devops/preflight/check_isaac_extensions.py; then
    ((PASSED++))
else
    ((FAILED++))
    echo -e "${RED}Skipping remaining checks due to Isaac Sim unavailability.${NC}"
    exit 1
fi
echo ""

# ------------------------------------------------------------------------------
# 3. USD Model Integrity Check (if available)
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[3/$TOTAL_CHECKS] USD Model Integrity Check${NC}"
USD_FILE="assets/roarm_m3/usd/roarm_m3.usd"

if [ -f "$USD_FILE" ]; then
    if python devops/preflight/check_usd_integrity.py "$USD_FILE"; then
        ((PASSED++))
    else
        ((FAILED++))
    fi
else
    echo -e "${YELLOW}⚠ USD file not found: $USD_FILE (skipping check)${NC}"
    echo "  This is normal if you haven't generated USD yet."
fi
echo ""

# ------------------------------------------------------------------------------
# Summary
# ------------------------------------------------------------------------------
echo -e "${GREEN}========================================================================"
echo "              Preflight Summary"
echo "========================================================================${NC}"
echo -e "Total Checks: $TOTAL_CHECKS"
echo -e "Passed: ${GREEN}$PASSED${NC}"
echo -e "Failed: ${RED}$FAILED${NC}"
echo ""

if [ $FAILED -gt 0 ]; then
    echo -e "${RED}PREFLIGHT FAILED!${NC} Fix errors above before proceeding."
    exit 1
else
    echo -e "${GREEN}ALL PREFLIGHT CHECKS PASSED!${NC}"
    echo "System ready for Isaac Sim development."
    exit 0
fi
