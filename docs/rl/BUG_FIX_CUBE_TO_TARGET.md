# 🐛 Critical Bug Fix: Cube-to-Target Index Error

**일자**: 2025-10-20  
**버전**: V5 (Post-V4 Critical Fix)  
**작성자**: AI Assistant + User

---

## 📋 요약

**핵심 버그**: 관측 벡터에서 `cube_to_target` 인덱스를 **잘못 사용**하여 성공 판정이 전혀 작동하지 않음.

**결과**: 
- ✅ 모든 에피소드가 TimeLimit(600)으로 종료
- ❌ SUCCESS 조건이 절대 참이 될 수 없음
- ❌ 조기 종료 불가능 (l<600 에피소드 없음)

**수정**: 
- `obs[20:23]` (cube_velocity) → `obs[14:17]` (cube_to_target)
- 월드 좌표로 재계산하여 신뢰도 ↑

---

## 🔍 문제 분석

### 관측 벡터 구성 (28 dim)

```python
0:6   joints(6)
6:8   gripper(2)
8:11  cube_rel_to_ee(3)      # EE → Cube 벡터
11:14 target_rel_to_ee(3)    # EE → Target 벡터
14:17 cube_to_target(3)      # ✅ Cube → Target 벡터 (정답 위치!)
17:20 ee_velocity(3)
20:23 cube_velocity(3)        # ❌ 잘못 사용된 인덱스 (속도!)
23    gripper_width
24    is_grasped
25    dist_to_cube
26    dist_cube_to_target
27    previous_reward
```

### 버그 위치

**1. `step()` 함수 (line 530)**
```python
# 🐛 BUG: cube_velocity를 cube_to_target로 착각
cube_to_target_dist = np.linalg.norm(obs[20:23])  # ❌ 잘못된 인덱스!
```

**2. `_check_done()` 함수 (line 697)**
```python
# 🐛 BUG: cube_velocity를 cube_to_target로 착각
cube_to_target_vec = obs[20:23]  # ❌ 잘못된 인덱스!
cube_to_target_dist = np.linalg.norm(cube_to_target_vec)
```

### 영향

1. **성공 조건 미작동**:
   - `cube_velocity`는 보통 작은 값 (< 0.1m/s)
   - `success_threshold = 0.02m` (2cm)
   - **속도가 거리 임계치보다 작아 성공 판정이 우연히 발생**할 수 있지만, **큐브가 실제로 타겟에 도달하지 않아도 성공으로 잘못 판단**

2. **TimeLimit 100%**:
   - V4 10K 테스트: 17/17 episodes 모두 l=600 (TimeLimit)
   - 조기 종료 불가능

3. **학습 효율 저하**:
   - 정책이 실제 목표 달성 여부를 학습하지 못함
   - Dense 보상만으로 학습 (Sparse 이벤트 보상 미작동)

---

## ✅ 수정 사항

### 1. 올바른 인덱스 사용 + 월드 좌표 재계산

**`step()` 함수**:
```python
# ═══════════════════════════════════════════════════════════
# 📊 로깅: 이벤트 및 진행 상황 추적
# ═══════════════════════════════════════════════════════════
# 🔧 BUG FIX: 올바른 인덱스 사용 (14:17 = cube_to_target 벡터)
# 더 신뢰도 높은 방법: 월드 좌표로 재계산
cube_pos, _ = self.cube.get_world_pose()
target_pos = np.array(self.cfg.target_position)
cube_to_target_dist = float(np.linalg.norm(target_pos - cube_pos))
```

**`_check_done()` 함수**:
```python
# 🔧 BUG FIX: 월드 좌표로 정확하게 재계산
cube_pos, _ = self.cube.get_world_pose()
target_pos = np.array(self.cfg.target_position)
cube_to_target_dist = float(np.linalg.norm(target_pos - cube_pos))

# 관측 벡터의 큐브 위치도 참고 (디버깅용)
cube_pos_obs = obs[11:14]
```

### 2. 보상 시스템 개선 (Δ형 - 개선량 기반)

**문제**: Dense 보상이 `-dist * (3 또는 2)` 형태라 600스텝 누적 시 -300~-600대로 쉽게 커짐.

**해결**: 절대값 대신 **개선량(Δ)** 기반으로 변경:
```python
# ═══════════════════════════════════════════════════════════
# 🎁 A. DENSE REWARD (Δ형 - 개선량 기반)
# ═══════════════════════════════════════════════════════════
reward = 0.0

# Time penalty 완화 (기존 0.01 → 0.001)
reward -= 0.001

# 1. EE → Cube 접근 보상 (개선량 기반)
if self.prev_ee_to_cube_dist is not None:
    ee_progress = self.prev_ee_to_cube_dist - dist_to_cube
    reward += 5.0 * ee_progress  # 가까워지면 +, 멀어지면 -

# 2. Cube → Target 접근 보상 (grasp_valid 시만, 개선량 기반)
if grasp_valid and self.prev_cube_to_target_dist is not None:
    cube_progress = self.prev_cube_to_target_dist - dist_cube_to_target
    reward += 4.0 * cube_progress  # 가까워지면 +, 멀어지면 -

# 거리 이력 업데이트
self.prev_ee_to_cube_dist = dist_to_cube
self.prev_cube_to_target_dist = dist_cube_to_target
```

### 3. 이벤트 보상 상향 조정

```python
# 1️⃣ 근접 보상: +5 → +10
# 2️⃣ 그립 보상: +10 → +40
# 3️⃣ 리프트 보상: +15 → +50
# 4️⃣ 목표 근접 보상: 제거 (LIFT와 SUCCESS 사이 간격이 크지 않음)
# 5️⃣ Success 보상: +100 (유지)
```

### 4. 보상 클램핑 추가

```python
# ═══════════════════════════════════════════════════════════
# 🎁 C. REWARD CLIPPING (1스텝 보상 제한)
# ═══════════════════════════════════════════════════════════
# 스파이크 보상 제외하고 Dense 보상만 클램핑
if reward < 90.0:  # 큰 이벤트 보상 제외
    reward = np.clip(reward, -2.0, 2.0)
```

### 5. 그리퍼 임계치 완화

```python
# is_grasped: 0.02 → 0.025 (2.5mm)
is_grasped = 1.0 if (ee_to_cube_dist < 0.08 and gripper_width < 0.025) else 0.0

# grasp_valid: 0.02 → 0.025
grasp_valid = (
    dist_to_cube < 0.08 and
    gripper_width < 0.025 and  # 완화
    cube_pos[2] > 0.03
)
```

### 6. 렌더링 끄기 (학습 속도 향상)

```python
# Physics 시뮬레이션 스텝 (학습 시 render=False 권장)
self.world.step(render=False)
```

### 7. 에피소드 통계 추가

```python
# 마일스톤 카운터
self.episode_reach_count = 0
self.episode_grip_count = 0
self.episode_lift_count = 0

# info에 추가
info = {
    ...
    "milestone_counts": {
        "reach": self.episode_reach_count,
        "grip": self.episode_grip_count,
        "lift": self.episode_lift_count,
    },
    "gripper": {
        "width": float(gripper_width),
        "is_grasped": float(is_grasped),
        "grip_frames": self.grip_frames,
    },
    ...
}
```

---

## 🎯 기대 효과

### 1. 성공 판정 작동
- ✅ SUCCESS 조건이 올바르게 평가됨
- ✅ 조기 종료 가능 (l<600 에피소드 발생)
- ✅ done_reason에 "success" 등장

### 2. 학습 효율 향상
- ✅ Δ형 보상으로 누적 음수 완화 (-600 → -50 예상)
- ✅ 이벤트 보상 상향으로 마일스톤 학습 강화
- ✅ 보상 클램핑으로 학습 안정성 ↑

### 3. 그리퍼 동작 개선
- ✅ 임계치 완화로 grasp 달성률 ↑
- ✅ 에피소드 통계로 디버깅 용이

### 4. 학습 속도 향상
- ✅ render=False로 FPS 2배 이상 향상 (예상: 100 FPS → 200+ FPS)

---

## 📊 검증 계획

### 1. 즉시 확인 (10K 테스트)
```bash
~/isaacsim/python.sh scripts/rl/train_dense_reward.py --timesteps 10000
```

**확인 사항**:
- [ ] monitor.csv에 **l<600 에피소드** 등장
- [ ] done_reason에 **"success"** 등장
- [ ] 평균 보상 개선 (-600 → -150 이하)
- [ ] reach/grip/lift 카운트 증가

### 2. 정규 학습 (50K)
```bash
~/isaacsim/python.sh scripts/rl/train_dense_reward.py --timesteps 50000
```

**확인 사항**:
- [ ] ep_rew_mean: -280 → -150 → -50 → 0 추세
- [ ] SUCCESS 달성 에피소드 발생
- [ ] Phase 0→1 승급 (성공률 30%)

---

## 🚀 다음 단계

1. **V5 10K 테스트** (최우선! ⚠️)
   - 버그 수정 효과 즉시 확인
   - l<600 에피소드 발생 여부

2. **V5 50K 정규 학습**
   - 완전한 학습 실행
   - Phase 승급 확인

3. **GUI 테스트**
   - 학습된 정책 시각화
   - SUCCESS 달성 확인

4. **추가 개선 검토** (V5 결과 기반)
   - EE 위치 추정 개선 (근사 FK → 정확한 링크 pose)
   - 그리퍼 파이프라인 점검
   - 보상 스케일 미세 조정

---

## 📝 체크리스트

- [x] 관측 벡터 인덱스 수정 (step, _check_done)
- [x] 월드 좌표 재계산 추가
- [x] Δ형 보상 구조 변경
- [x] 이벤트 보상 상향 조정
- [x] 보상 클램핑 추가
- [x] 그리퍼 임계치 완화
- [x] 렌더링 끄기
- [x] 에피소드 통계 추가
- [ ] V5 10K 테스트 실행
- [ ] 결과 분석 및 문서화
- [ ] V5 50K 정규 학습
- [ ] GUI 테스트

---

## 🏆 결론

이 버그는 **V4 개선의 핵심 목표**를 완전히 무효화시키는 치명적인 문제였습니다. 

**V4의 의도**:
- SUCCESS 조건 강화 (2cm, 10프레임)
- Phase 난이도 상향
- TimeLimit 명시적 설정

**실제 결과** (V4 버그):
- SUCCESS 조건이 잘못된 값으로 평가됨
- 모든 에피소드가 TimeLimit으로 종료
- 정책이 실제 목표 달성을 학습하지 못함

**V5 수정 후 기대**:
- ✅ SUCCESS 조건이 올바르게 작동
- ✅ 조기 종료 가능 (l<600)
- ✅ 학습 효율 대폭 향상
- ✅ 마일스톤 기반 학습 정상화

이제 **V5 10K 테스트**를 실행하여 수정 효과를 즉시 확인해야 합니다! 🚀
