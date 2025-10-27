# 🔧 RL 환경 개선 로그 V3

**날짜**: 2025-10-20  
**버전**: V3 - 관측 신호 개선 + Hybrid Reward

---

## 📊 이전 결과 분석 (V2: 50K Shaped-Sparse)

### 학습 성과
- ⏱️ 총 시간: 5.7분 (51,200 steps)
- 📈 FPS: 150-151 (안정적)
- 🎯 마일스톤: REACH 2회 (2.4%)
- 📉 평균 보상: -5.89
- ✅ EV: 0.72 (Good!)
- 🔴 문제: 모든 에피소드 타임아웃, 학습 정체

### 근본 원인
1. **관측 신호 문제** (가장 중요!)
   - 월드 좌표 사용 → 정책이 학습하기 어려움
   - 큐브 정보가 EE와 독립적으로 제공됨
   
2. **Sparse Reward 한계**
   - 마일스톤 도달이 너무 어려움
   - Step penalty만 누적 → 탐색 동기 부족
   
3. **Curriculum 진행 실패**
   - Phase 0 → 1: REACH 10회 필요 (너무 엄격)
   - 50K에서 2회만 달성 → 250K+ 필요 예상

---

## 🚀 V3 개선 사항

### 1. 관측 신호 개선 (핵심! ⭐⭐⭐)

#### Before (V2): 월드 좌표 기반
```python
Observation (25 dim):
  - Joint positions (8)
  - EE position (3) ← 월드 좌표
  - Cube position (3) ← 월드 좌표
  - Target position (3) ← 월드 좌표
  - EE → Cube vector (3)
  - Cube → Target vector (3)
  - Gripper width (1)
  - Is grasped (1)
```

**문제점**:
- EE 위치와 Cube 위치가 독립적
- 정책이 "나(EE)에게서 큐브가 어디 있는지" 직관적으로 이해 불가
- 월드 좌표는 절대 위치라 일반화 어려움

#### After (V3): EE 기준 상대 좌표 ✅
```python
Observation (28 dim):
  - Joint positions (8)
  - Cube relative to EE (3) ← 핵심! EE 기준 상대 좌표
  - Target relative to EE (3) ← 핵심! EE 기준 상대 좌표
  - Cube to Target (3)
  - EE velocity (3) ← 추가! 시간적 정보
  - Cube velocity (3) ← 추가! 시간적 정보
  - Gripper width (1)
  - Is grasped (1)
  - Distance to cube (1) ← 추가! 명시적 거리
  - Distance cube to target (1) ← 추가!
  - Previous reward (1) ← 추가! 학습 안정화
```

**개선 효과**:
- ✅ **Egocentric View**: 정책이 "나(EE)를 기준으로" 큐브/타겟 위치 이해
- ✅ **좌표계 정합**: EE 움직임과 큐브 위치가 일관된 좌표계
- ✅ **시간적 정보**: 속도로 동적 변화 인식
- ✅ **학습 안정화**: 이전 보상으로 시간적 연속성

**구현 코드**:
```python
def _get_observation(self) -> np.ndarray:
    # 1. 월드 좌표 수집
    ee_pos = self._get_ee_position()
    cube_pos, _ = self.cube.get_world_pose()
    target_pos = np.array(self.cfg.target_position)
    
    # 2. EE 기준 상대 좌표 변환 ← 핵심!
    cube_relative_to_ee = cube_pos - ee_pos
    target_relative_to_ee = target_pos - ee_pos
    
    # 3. 속도 계산
    if self.prev_ee_pos is not None:
        ee_velocity = (ee_pos - self.prev_ee_pos) * 60.0
        cube_velocity = (cube_pos - self.prev_cube_pos) * 60.0
    else:
        ee_velocity = np.zeros(3)
        cube_velocity = np.zeros(3)
    
    # 4. 관측 벡터 구성
    obs = np.concatenate([
        joint_positions,
        cube_relative_to_ee,  # ← EE 기준!
        target_relative_to_ee,  # ← EE 기준!
        cube_to_target,
        ee_velocity,  # ← 추가!
        cube_velocity,  # ← 추가!
        [gripper_width],
        [is_grasped],
        [dist_to_cube],  # ← 추가!
        [dist_cube_to_target],  # ← 추가!
        [self.previous_reward],  # ← 추가!
    ])
    
    return obs
```

### 2. Hybrid Reward (Dense + Shaped-Sparse)

#### Dense Reward (매 스텝, 지속적 피드백)
```python
# 1. EE → Cube 접근 (항상)
dense_reach = -distance_to_cube * 3.0

# 2. Cube → Target 접근 (grasp_valid 시만)
dense_move = -distance_to_target * 2.0

# 3. 진전 보너스
if distance_decreased:
    reward += 0.5 ~ 1.0
```

**효과**:
- ✅ 매 스텝 학습 신호 제공
- ✅ 탐색 동기 강화
- ✅ Sparse 보상 도달 전까지 안내

#### Shaped-Sparse (마일스톤, 1회성)
```python
# 기존 V2 유지 (검증됨)
REACH: +5.0
GRIP: +10.0
LIFT: +15.0
GOAL NEAR: +20.0
SUCCESS: +100.0
```

### 3. Curriculum 완화

#### Before (V2)
```python
Phase 0 → 1: REACH 10회, 성공률 60%, 200 에피소드
```

#### After (V3)
```python
Phase 0 → 1: REACH 5회, 성공률 30%, 100 에피소드
```

**근거**:
- V2에서 50K로 REACH 2회 달성
- 5회는 합리적 (25K-30K 예상)
- 성공률 30%로 완화 (초기 학습 고려)

### 4. 관측 신호 디버깅

```python
# 첫 스텝에 자동 출력
if self.current_step == 0:
    print(f"\n🔍 관측 신호 점검:")
    print(f"  - Observation dim: {len(obs)}")
    print(f"  - EE pos (world): {ee_pos}")
    print(f"  - Cube pos (world): {cube_pos}")
    print(f"  - Cube relative to EE: {cube_relative_to_ee}")
    print(f"  - Distance to cube: {dist_to_cube:.3f}m")
```

---

## 🎯 예상 효과

### V2 대비 개선 예상
1. **학습 속도**: 2-3배 빠름 (Dense 신호)
2. **REACH 달성률**: 2% → 20-30%
3. **GRIP 달성 가능성**: 높음 (Dense 안내)
4. **Curriculum 진행**: Phase 1 진입 가능

### 50K 재학습 목표
- ✅ REACH: 10회 이상 (V2: 2회)
- ✅ GRIP: 5회 이상 (V2: 0회)
- ✅ LIFT: 2회 이상 (V2: 0회)
- ⭐ Phase 1 진입 가능성

---

## 📋 체크리스트

- [x] 관측 공간: 28 dim (25 → 28)
- [x] EE 기준 상대 좌표 변환
- [x] 속도 정보 추가
- [x] Dense Reward 추가
- [x] Hybrid Reward 통합
- [x] Curriculum 완화
- [x] 디버깅 출력 추가
- [ ] 50K 재학습 실행
- [ ] 결과 분석 및 비교

---

## 🚀 다음 단계

1. **50K 재학습**:
   ```bash
   make train  # 50K steps, ~5-6분
   ```

2. **결과 확인**:
   - REACH 달성 횟수
   - Dense Reward 효과
   - 학습 곡선 비교

3. **GUI 검증**:
   ```bash
   ~/isaacsim/python.sh scripts/rl/test_trained_model.py --gui
   ```

4. **필요 시 추가 개선**:
   - Dense Reward 가중치 조정
   - Curriculum 조건 미세 조정
   - Max steps 증가 고려

---

## 📚 참고 문헌

**전문가 권장사항** (Jarvis 제안):
1. ✅ 관측 신호: EE 기준 상대 좌표
2. ✅ 좌표계 정합: 일관된 기준점
3. ✅ Dense Reward: 지속적 학습 신호
4. ✅ 게이팅: grasp_valid 조건
5. ✅ Curriculum 완화: 현실적 목표

---

**변경 이력**:
- 2025-10-20: V3 개선 적용 (관측 신호, Hybrid Reward, Curriculum 완화)
- 2025-10-20: V2 결과 분석 (50K, EV 0.72, REACH 2회)
