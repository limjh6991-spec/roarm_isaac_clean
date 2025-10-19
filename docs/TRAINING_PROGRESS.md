# RoArm-M3 RL Training Progress

## 📊 현재 보상 기준 (Shaped-Sparse Reward)

### 보상 구조
```python
1. 근접 보상 (+5.0)
   - 조건: EE가 큐브 5cm 이내 진입
   - 게이팅: 없음
   - 1회성: ✅ (first_reach 플래그)
   - 상태: ✅ 달성! (50K steps)

2. 그립 보상 (+10.0)
   - 조건: 유효 그립 3프레임 유지
   - 게이팅: ✅ grasp_valid
   - 1회성: ✅ (valid_grip 플래그)
   - 상태: ❌ 미달성

3. 리프트 보상 (+15.0)
   - 조건: 큐브 5cm 이상 들어올림
   - 게이팅: ✅ grasp_valid
   - 1회성: ✅ (lifted 플래그)
   - 상태: ❌ 미달성

4. 목표 근접 보상 (+20.0)
   - 조건: 큐브가 타겟 8cm 이내
   - 게이팅: ✅ grasp_valid
   - 1회성: ✅ (goal_near 플래그)
   - 상태: ❌ 미달성

5. Success 보상 (+100.0)
   - 조건: 타겟 5cm 이내 5프레임 유지
   - 게이팅: 없음 (결과 검증)
   - 1회성: ✅ (에피소드당)
   - 상태: ❌ 미달성

6. Time Penalty (-0.01)
   - 매 스텝 적용
   - 효율성 유도
```

### grasp_valid 게이팅 조건
```python
grasp_valid = (
    ee_to_cube_dist < 0.08 and      # EE가 8cm 이내
    gripper_width < 0.02 and        # 그리퍼가 닫힘
    cube_pos[2] > 0.03              # 큐브가 바닥 위
)
```

## 📚 Curriculum Learning

### Phase 0: Easy Mode (현재)
- **큐브 거리**: 10~15cm (로봇 가까이)
- **타겟 거리**: 20~25cm (가까운 목표)
- **승급 조건**: 성공률 ≥60% (최근 200 에피소드)
- **목적**: 빠른 Success 경험 축적

### Phase 1: Normal Mode (미래)
- **큐브 거리**: 25~35cm (원래 거리)
- **타겟 거리**: 25~35cm
- **목적**: 일반화 및 실전 대비

## 📈 학습 성과

### Dense Reward (실패)
- **Steps**: 100K
- **ep_rew_mean**: +916 (폭발!)
- **Explained Variance**: 0.00006 (붕괴)
- **Value Loss**: 855 (극도로 높음)
- **문제**: 개선 보상 누적으로 정책 붕괴

### Sparse Reward (안정)
- **Steps**: 100K
- **ep_rew_mean**: -6.01 → -6.01 (안정)
- **Explained Variance**: 0.283 (4,717배 개선!)
- **Value Loss**: 0.136 (6,287배 개선!)
- **문제**: Success 신호 부족 (GUI 테스트 실패)

### Shaped-Sparse + Curriculum (현재)
- **Steps**: 50K (Phase 0)
- **ep_rew_mean**: -6.01 → -4.21 (30% 개선!)
- **마일스톤**: 
  - ✅ REACH (+5.0) 달성!
  - ❌ GRIP, LIFT, GOAL, SUCCESS 미달성
- **Checkpoints**: 9개 저장
- **다음 목표**: 200K steps 장기 학습

## 🔧 하이퍼파라미터

```python
# PPO 설정
learning_rate = 3e-4
n_steps = 2048
batch_size = 64
n_epochs = 10
gamma = 0.99
gae_lambda = 0.95

# 안정화
clip_range = 0.2
clip_range_vf = 1.0      # Value Clipping
max_grad_norm = 0.5
target_kl = 0.03         # 정책 안정성

# 탐색
ent_coef = 0.01          # Shaped-Sparse용 탐색 강화

# 정규화
VecNormalize:
  - norm_obs = True
  - norm_reward = True
  - clip_reward = 10.0
```

## 📝 전문가 의견 적용

### TOP 5 우선순위 (완료)
1. ✅ **Sparse Reward**: 개선 보상 완전 제거
2. ✅ **Value Clipping**: clip_range_vf=1.0
3. ✅ **하이퍼파라미터 기본값**: lr, vf_coef, n_epochs
4. ✅ **VecNormalize**: obs+reward 정규화
5. ✅ **조기 경보**: EV/VL 자동 감지

### 추가 개선 (완료)
6. ✅ **Shaped-Sparse**: 게이팅 + 1회성 이벤트
7. ✅ **Curriculum Learning**: Phase 0 Easy Mode
8. ✅ **로깅 시스템**: events, done_reason, success_rate

## 🚧 URDF 개선 필요

### 문제점
1. **그리퍼 형태**: 물건을 잡기에 부적합
   - 현재: 단순 프리즘 형태
   - 필요: 평행 그리퍼 (Parallel Jaw)

2. **Link 크기/비율**: 실제 RoArm M3와 불일치
   - Link 2: 현재 120mm → 실제 160mm
   - Link 3: 현재 불명확 → 실제 150mm
   - Link 4: 현재 불명확 → 실제 90mm

### 개선 계획
- RoArm M3 공식 문서 참조
- Link dimensions 정확한 측정값 반영
- 그리퍼 메커니즘 재설계 (parallel jaw)
- Collision meshes 업데이트

## 🎯 다음 단계

### 단기 (오늘)
1. ✅ GUI 테스트 실행
2. ⏳ 200K steps 장기 학습 백그라운드 실행
3. 📝 문서 정리 (README, logs)
4. 🔄 GitHub push

### 중기 (내일)
1. URDF 개선 작업
2. 장기 학습 결과 분석
3. Phase 1 Normal Mode 전환 (성공률 ≥60% 시)

### 장기
1. Phase 1 완료 후 최종 성능 평가
2. Real Robot 테스트 준비
3. 논문/보고서 작성

---

**Last Updated**: 2025-10-19 18:50 KST
**Training Phase**: Phase 0 (Easy Mode)
**Total Steps**: 50K
**Next Milestone**: GRIP (+10.0)
