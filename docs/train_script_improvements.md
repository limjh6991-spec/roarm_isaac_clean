# Train Dense Reward 스크립트 개선 사항

## 📋 현재 상태 분석

### ✅ 잘 구현된 부분
1. VecNormalize 적용 (관측/보상 정규화)
2. Curriculum Learning 구조
3. TimeLimit 명시적 처리
4. 체크포인트 저장
5. Early Warning Callback

### 🔧 개선 필요 사항

#### 1. 관측 공간 차원 불일치
**문제**: 환경은 28 dim이지만 명시적 선언 부족
**해결**: `observation_space` 생성 시 실제 dim 사용

#### 2. 로깅 부족
**문제**: 학습 중 주요 메트릭 출력 없음
**해결**: 커스텀 콜백으로 에피소드별 보상/성공률 로깅

#### 3. Tensorboard 활용 미흡
**문제**: 기본 로깅만 사용
**해결**: 커스텀 메트릭 추가 (성공률, 단계별 달성률)

#### 4. Curriculum 승급 로직 미구현
**문제**: Phase 0에서 고정
**해결**: 성공률 기반 자동 승급

#### 5. 멀티프로세싱 미지원
**문제**: 단일 환경만 사용
**해결**: SubprocVecEnv로 병렬화 옵션 추가

## 🚀 개선 계획

### Phase 1: 로깅 강화
- [x] 에피소드별 보상 출력
- [ ] TensorBoard 커스텀 메트릭
- [ ] 단계별 달성률 추적 (REACH, GRIP, LIFT, PLACE)

### Phase 2: Curriculum 자동화
- [ ] 성공률 추적 (sliding window)
- [ ] 자동 승급 로직
- [ ] Phase 전환 시 알림

### Phase 3: 병렬화
- [ ] SubprocVecEnv 옵션
- [ ] num_envs 설정 가능
- [ ] 환경별 독립 랜덤 시드

### Phase 4: 시각화
- [ ] 실시간 학습 그래프
- [ ] 보상 분해 차트
- [ ] 에피소드 비디오 녹화

