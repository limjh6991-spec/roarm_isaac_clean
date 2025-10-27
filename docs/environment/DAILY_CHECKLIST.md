# RoArm M3 매일 체크리스트

**목적**: 환경 문제 사전 방지 및 안정적인 개발 환경 유지

**소요 시간**: 5분

---

## ☀️ 아침 루틴 (프로젝트 시작 시)

### 1. 터미널 열기 및 이동

```bash
cd ~/roarm_isaac_clean
```

### 2. Git 상태 확인

```bash
git status
git pull  # 팀 작업 시
```

**체크 포인트**:
- [ ] 브랜치 확인 (main or feature branch)
- [ ] 미커밋 변경사항 확인
- [ ] 최신 코드 동기화 완료

---

### 3. ⭐ Preflight Check 실행 (필수!)

```bash
make preflight
```

**또는 VS Code**:
- `Ctrl+Shift+B` → `🔍 Preflight Check` 선택

**예상 결과**:
```
╔═══════════════════════════════════════════════════════════╗
║        ✅ ALL CHECKS PASSED - Ready to work! ✅           ║
╚═══════════════════════════════════════════════════════════╝
```

**체크 포인트**:
- [ ] GPU 정상 작동 (nvidia-smi)
- [ ] Vulkan 정상 작동
- [ ] Isaac Sim 경로 확인
- [ ] 디스크 공간 충분 (> 10GB)
- [ ] 메모리 충분 (> 16GB)
- [ ] 환경 변수 로드 확인

**❌ 실패 시**: `docs/environment/TROUBLESHOOTING.md` 참조

---

### 4. 이전 로그 확인

```bash
# 최근 훈련 로그
ls -lht logs/ | head -10

# 마지막 로그 확인
tail -50 logs/train_*.log | less
```

**체크 포인트**:
- [ ] 마지막 훈련 완료 상태 확인
- [ ] 에러 발생 여부 확인
- [ ] 마일스톤 달성 현황 확인

---

### 5. TensorBoard 확인 (선택)

```bash
make tensorboard &
# 브라우저: http://localhost:6006
```

**체크 포인트**:
- [ ] 학습 곡선 추세 확인
- [ ] 평균 보상 (ep_rew_mean) 증가 추세
- [ ] 이상 패턴 확인 (폭발, 정체 등)

---

## 💻 작업 전 체크

### 6. 현재 작업 계획 명확화

**질문**:
- 오늘 목표는? (예: 50K 학습, GRIP 조건 완화, 보상 함수 수정)
- 예상 소요 시간은?
- 실패 시 대안은?

**문서 참조**:
```bash
make docs  # 문서 목록 표시
```

- [ ] `docs/rl/RL_TRAINING_PLAN_V2.md` - 전체 계획
- [ ] `docs/rl/DIAGNOSTIC_CHECKLIST.md` - 진단 가이드
- [ ] `docs/rl/REWARD_DESIGN_GUIDE.md` - 보상 함수

---

### 7. 빠른 테스트 (권장)

**처음 시도하는 변경 사항이라면**:

```bash
make train-quick  # 10K steps, ~15분
```

**목적**: 환경 정상 작동 확인, 빠른 피드백

**체크 포인트**:
- [ ] 스크립트 에러 없이 시작
- [ ] 환경 초기화 정상
- [ ] 첫 100 steps 정상 진행
- [ ] GPU 사용률 확인 (`nvidia-smi`)

---

## 🎯 작업 중 체크

### 8. 주기적 모니터링 (30분마다)

```bash
# GPU 사용률
nvidia-smi

# 로그 확인
tail -f logs/train_*.log

# 프로세스 확인
ps aux | grep python
```

**체크 포인트**:
- [ ] GPU 메모리 사용률 정상 (< 90%)
- [ ] CPU 과부하 없음
- [ ] 로그 정상 출력 (에러 없음)
- [ ] 학습 진행 중 (ep_rew_mean 업데이트)

---

### 9. 중간 결과 확인 (학습 완료 시)

```bash
# TensorBoard에서 확인
# 또는 로그 파일 분석

grep "REACH\|GRIP\|LIFT" logs/train_*.log | tail -20
```

**체크 포인트**:
- [ ] 마일스톤 달성 횟수 확인
- [ ] 평균 보상 변화 확인
- [ ] 예상 결과와 비교

---

## 🌙 작업 후 체크

### 10. 결과 정리

```bash
# 모델 파일 확인
ls -lh models/*.zip

# 로그 백업
cp logs/train_*.log logs/backup/
```

**체크 포인트**:
- [ ] 모델 파일 저장 확인
- [ ] 로그 파일 백업
- [ ] 중요 결과 스크린샷 저장

---

### 11. Git 커밋

```bash
# 변경사항 확인
git diff

# 문서만 커밋 (안전)
make commit-docs

# 또는 전체 커밋
git add .
git commit -m "feat: Improve GRIP condition (ee_dist 0.08->0.10)"
git push
```

**체크 포인트**:
- [ ] 의미 있는 변경사항만 커밋
- [ ] 커밋 메시지 명확
- [ ] `.env.local` 등 제외 확인

---

### 12. 내일을 위한 메모

**다음 작업 기록**:
```bash
# TODO.md 또는 노트에 기록
echo "## 2025-10-21 TODO" >> TODO.md
echo "- [ ] 50K 학습 결과 분석" >> TODO.md
echo "- [ ] GRIP 조건 완화 테스트" >> TODO.md
```

**체크 포인트**:
- [ ] 다음 작업 명확히 기록
- [ ] 실패한 실험 원인 기록
- [ ] 새로운 아이디어 메모

---

## 🚨 비상 상황 대응

### 환경 문제 발생 시

```bash
# 1. Preflight 재실행
make preflight

# 2. 진단 실행
make diagnose

# 3. 트러블슈팅 문서
cat docs/environment/TROUBLESHOOTING.md
```

### 학습 실패 시

```bash
# 1. 로그 분석
tail -100 logs/train_*.log

# 2. 에러 검색
grep -i "error\|failed\|exception" logs/train_*.log

# 3. 진단 가이드 참조
cat docs/rl/DIAGNOSTIC_CHECKLIST.md
```

---

## 📊 주간 체크리스트 (금요일)

### 1. 전체 백업

```bash
make backup
```

### 2. 디스크 정리

```bash
make clean-models  # 7일 이상 된 모델 백업
make clean         # 로그 및 캐시 정리
```

### 3. 문서 업데이트

- [ ] 실험 결과 요약
- [ ] 문제 해결 방법 추가
- [ ] 다음 주 계획 수립

### 4. Git 정리

```bash
git log --oneline --since="1 week ago"
git diff HEAD~10..HEAD --stat
```

---

## 🎯 성공 지표

### 일일 목표 달성

- ✅ Preflight 통과
- ✅ 계획한 작업 완료
- ✅ 에러 없이 학습 진행
- ✅ 결과 백업 및 커밋

### 주간 목표 달성

- ✅ 50K steps 학습 완료
- ✅ GRIP 마일스톤 5회 이상
- ✅ 문서 업데이트
- ✅ 다음 주 계획 수립

---

## 💡 팁

### 시간 절약

```bash
# 아침 루틴 자동화
make morning

# 빠른 테스트
make train-quick

# VS Code 단축키 사용
# Ctrl+Shift+B → 작업 선택
```

### 에러 예방

1. **항상 `make preflight` 먼저 실행**
2. 새 변경사항은 `train-quick`으로 먼저 테스트
3. 로그 주기적 확인
4. Git 자주 커밋

### 효율적 학습

1. TensorBoard로 실시간 모니터링
2. 하이퍼파라미터 1개씩만 변경
3. 실패도 기록 (귀중한 데이터!)
4. 문서 참조 습관화

---

**체크리스트 출력**:
```bash
make checklist
```

**도움말**:
```bash
make help
```

---

**작성일**: 2025-10-20  
**업데이트**: 매주 금요일  
**문의**: docs/environment/TROUBLESHOOTING.md
