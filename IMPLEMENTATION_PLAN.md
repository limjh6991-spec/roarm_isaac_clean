# Curriculum + Shaped-Sparse 구현 계획

## 완료된 작업
- ✅ Dense Reward 실패 원인 파악 (보상 폭발)
- ✅ Sparse Reward 전환 및 학습 안정화 (EV 0.283)
- ✅ 전문가 의견 TOP5 적용 성공
- ✅ GUI 테스트 및 문제 진단 (Success 신호 부족)

## 현재 상황
- 학습 안정성: 완벽 ✅
- 실제 성능: 큐브 인식 실패 ❌
- 근본 원인: Success 보상(+100) 미획득으로 학습 신호 부족

## 다음 단계 구현 (전문가 의견 반영)

### 1. Shaped-Sparse Reward (우선순위 최고)
**목표**: 학습 신호 제공하되 누적 방지

**보상 구조**:
```python
# 1회성 이벤트 보상 (게이팅)
- 근접 달성 (EE-큐브 < 0.05m): +5 (1회)
- 유효 그립 (grasp_valid + m프레임): +10 (1회)
- 리프트 (Z > 0.05m, grasp_valid): +15 (1회)
- 목표 근접 (큐브-목표 < 0.08m, grasp_valid): +20 (1회)
- Success (< 0.05m, M프레임): +100

# 매 스텝
- Time penalty: -0.01
```

**안전장치**:
- grasp_valid 게이팅 (흔들기 차단)
- 1회성 플래그 (중복 지급 차단)
- m 프레임 히스테리시스 (노이즈 방지)

### 2. Curriculum Learning (간소화 버전)
**Phase 0 (Easy Mode)**:
- 큐브 거리: 0.10~0.15m (현재: 0.30m)
- 목표 거리: 0.20~0.25m
- 목표: Success 경험 축적

**Phase 1 (Normal Mode)**:
- 성공률 > 60% 달성 시 원래 거리로 복귀

### 3. 로깅 시스템
```python
info = {
    'events': {
        'first_reach': bool,
        'valid_grip': bool,
        'lifted': bool,
        'goal_near': bool,
        'success': bool
    },
    'done_reason': 'success|timeout|safety',
    'curriculum_phase': int,
    'success_rate': float
}
```

### 4. 하이퍼파라미터
```python
# 기본값 유지 + 미세 조정
ent_coef = 0.01  # 초기 탐색 강화
target_kl = 0.03
clip_range_vf = 1.0
```

## 구현 우선순위

1. **Shaped-Sparse 보상 함수** (가장 중요!)
   - 예상 시간: 30분
   - 효과: 즉시 학습 신호 제공

2. **Phase 0 Easy Mode**
   - 예상 시간: 15분
   - 효과: 성공 경험 빠른 축적

3. **로깅 시스템**
   - 예상 시간: 15분
   - 효과: 학습 진행 명확한 추적

4. **50K steps 학습**
   - 예상 시간: 7분
   - 목표: 첫 Success 경험!

## 예상 결과
- 초기 10K steps 내 첫 Success 달성
- 30K steps: 성공률 > 30%
- 50K steps: 성공률 > 60%
- GUI 테스트: 큐브 인식 및 그리핑 시도 확인

## 참고: 전문가 의견 핵심
- ✅ Curriculum + Shaped-Sparse 조합
- ✅ grasp_valid 게이팅 필수
- ✅ 1회성 이벤트로 누적 차단
- ⚠️ Dense 재도입 위험

