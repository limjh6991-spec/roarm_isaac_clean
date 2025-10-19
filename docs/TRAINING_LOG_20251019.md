# Training Log - 2025년 10월 19일

## 📊 학습 진행 상황

### Phase 0 (Easy Mode) - 50K Steps 완료

**학습 설정:**
- Curriculum: Phase 0 (큐브 10~15cm, 타겟 20~25cm)
- Reward: Shaped-Sparse (게이팅 + 1회성 이벤트)
- Hyperparameters: ent_coef=0.01, target_kl=0.03, clip_range_vf=1.0

**학습 결과:**
- ep_rew_mean: -6.01 → -4.21 (30% 개선!)
- Checkpoints: 9개 저장 (5K~50K steps)
- **첫 마일스톤 달성**: REACH (+5.0) ✅

**마일스톤 진행 상황:**
1. ✅ 근접 (+5): EE→큐브 5cm - **달성!**
2. ❌ 그립 (+10): 유효 그립 3프레임 [게이팅]
3. ❌ 리프트 (+15): 큐브 5cm 상승 [게이팅]
4. ❌ 목표 (+20): 큐브→타겟 8cm [게이팅]
5. ❌ Success (+100): 타겟 5cm 5프레임

## 🎬 GUI 테스트 결과

**테스트 환경:**
- Checkpoint: roarm_ppo_curriculum_50000_steps.zip
- Episodes: 2

**관찰 결과:**
1. ✅ **로봇팔 초기 상태 개선**: 이전 테스트보다 많이 안정화됨
2. ⚠️ **특정 라인에서 반복 종료**: 큐브를 찾는 행동 미관찰
3. 📊 **평가**: REACH 마일스톤은 달성했으나, 실제 그립 행동으로 이어지지 않음

**분석:**
- 로봇이 EE를 큐브 근처로 이동시키는 것은 학습했음
- 그러나 그리퍼를 실제로 닫는 행동(grasp_valid)은 아직 학습 부족
- 200K steps 장기 학습으로 추가 마일스톤 달성 필요

## 🎯 현재 보상 시스템

### Shaped-Sparse Reward 구조

```python
# 1회성 이벤트 보상 (게이팅)
근접 (+5.0)   : EE가 큐브 5cm 이내 - ✅ 달성
그립 (+10.0)  : 유효 그립 3프레임 유지 [게이팅: grasp_valid]
리프트 (+15.0) : 큐브 5cm 이상 들어올림 [게이팅: grasp_valid]
목표 (+20.0)   : 큐브가 타겟 8cm 이내 [게이팅: grasp_valid]
Success (+100) : 타겟 5cm 이내 5프레임 유지
Time (-0.01)   : 매 스텝 효율성 패널티

# grasp_valid 조건
ee_to_cube_dist < 0.08 AND gripper_width < 0.02 AND cube_pos[2] > 0.03
```

### 개선 효과 (Dense → Sparse → Shaped-Sparse)

| 지표 | Dense (실패) | Sparse (안정) | Shaped-Sparse (현재) |
|------|-------------|--------------|---------------------|
| ep_rew_mean | +916 (폭발) | -6.01 (안정) | -4.21 (개선) |
| EV | 0.00006 (붕괴) | 0.283 | - |
| VL | 855 (폭발) | 0.136 | - |
| 마일스톤 | 없음 | 없음 | REACH ✅ |

## 🚧 URDF 개선 필요 사항

### 1. 그리퍼 형태 문제
**현재**: 단순 프리즘 형태 → 물건을 잡기에 부적합
**필요**: 평행 그리퍼 (Parallel Jaw) 재설계

### 2. Link 크기/비율 불일치
**문제**: 실제 RoArm M3와 dimensions 불일치
**개선 필요**:
- Link 2: 실제 160mm 반영
- Link 3: 실제 150mm 반영
- Link 4: 실제 90mm 반영
- Gripper: 50mm opening 반영

## 📈 학습 이력

### 1차: Dense Reward (실패)
- Steps: 100K
- 문제: 개선 보상(△거리) 누적으로 정책 붕괴
- 결과: 폐기

### 2차: Sparse Reward (안정화)
- Steps: 100K
- 개선: EV 4,717배, VL 6,287배 개선
- 문제: Success 신호 부족 (GUI 테스트 실패)

### 3차: Shaped-Sparse + Curriculum (현재)
- Steps: 50K (Phase 0)
- 성과: REACH 마일스톤 달성
- 다음: 200K steps 장기 학습 예정

## 🎯 다음 단계

### 즉시 (오늘)
1. ✅ GUI 테스트 완료 및 분석
2. ⏳ 200K steps 장기 학습 백그라운드 실행
3. ⏳ 문서 정리 및 GitHub push

### 단기 (내일)
1. 장기 학습 결과 분석
2. 추가 마일스톤 달성 여부 확인
3. URDF 개선 작업 시작

### 중기
1. Phase 0 성공률 ≥60% 달성 시 Phase 1 전환
2. URDF 개선 버전으로 재학습
3. 최종 성능 평가

---

**작성일**: 2025-10-19 19:00 KST  
**학습 Phase**: Phase 0 (Easy Mode)  
**총 Steps**: 50K  
**달성 마일스톤**: REACH (+5.0)  
**다음 목표**: GRIP (+10.0)
