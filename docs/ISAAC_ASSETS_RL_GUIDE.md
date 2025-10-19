# Isaac Assets 기반 Pick and Place 강화학습 가이드

**날짜**: 2025-10-19  
**목적**: 로컬 Isaac Assets을 활용한 현실적인 Pick and Place 학습 환경 구축

---

## 📦 사용 가능한 Isaac Assets

### 경로
```
~/isaacsim_assets/Assets/Isaac/5.0/Isaac/
```

### 주요 Assets

#### 1. **Props/YCB** (물체)
표준 Pick and Place 벤치마크 물체:
- ✅ `003_cracker_box.usd` - 크래커 박스 (16.0 x 7.7 x 21.0 cm)
- ✅ `004_sugar_box.usd` - 설탕 박스 (9.2 x 4.5 x 17.8 cm)  
- ✅ `005_tomato_soup_can.usd` - 토마토 수프 캔 (6.7 x 6.7 x 10.2 cm)
- ✅ `006_mustard_bottle.usd` - 머스타드 병 (6.0 x 9.6 x 19.5 cm)

#### 2. **Props/Mounts** (테이블)
- ✅ `table.usd` - 기본 작업 테이블
- ✅ `ThorlabsTable/table_instanceable.usd` - 광학 테이블
- ✅ `SeattleLabTable/table.usd` - 실험실 테이블

#### 3. **Props/Blocks** (간단한 물체)
- 큐브, 직육면체 등 기하학적 물체

#### 4. **Environments** (배경)
- ✅ `Simple_Warehouse/warehouse.usd` - 간단한 창고
- `Grid/` - 그리드 환경
- `Simple_Room/` - 실내 환경

---

## 🎯 추천 학습 전략

### Phase 1: 기초 학습 (Easy Mode)
**목표**: 큰 물체로 기본 동작 학습

```python
env = RoArmPickPlaceIsaacEnv(
    curriculum_level="easy",       # 큰 박스만
    use_warehouse=False,           # 간단한 Ground Plane
    render=True
)
```

**특징**:
- 물체: 크래커 박스, 설탕 박스 (크고 잡기 쉬움)
- 배경: Ground Plane (주의 분산 최소화)
- 목표: 50K steps, Success Rate > 40%

**예상 소요 시간**: 1-2시간 학습

---

### Phase 2: 중급 학습 (Medium Mode)
**목표**: 다양한 물체로 일반화

```python
env = RoArmPickPlaceIsaacEnv(
    curriculum_level="medium",     # 박스 + 캔/병
    use_warehouse=False,
    render=True
)
```

**특징**:
- 물체: 박스 + 캔 + 병 (4종류)
- 다양한 크기/무게 → 일반화 능력 향상
- 목표: 100K steps, Success Rate > 30%

**예상 소요 시간**: 2-3시간 학습

---

### Phase 3: 고급 학습 (Warehouse Mode)
**목표**: 현실적 환경에서 동작

```python
env = RoArmPickPlaceIsaacEnv(
    curriculum_level="medium",
    use_warehouse=True,            # 창고 배경 추가
    render=True
)
```

**특징**:
- 배경: 창고 환경 (시각적 복잡도 증가)
- Domain Randomization 효과
- 목표: 50K steps (transfer learning)

---

## 🚀 빠른 시작

### 1. 환경 테스트 (5분)

```bash
# Isaac Sim으로 환경 테스트
~/isaac-sim.sh /home/roarm_m3/roarm_isaac_clean/envs/roarm_pickplace_isaac_assets.py
```

**확인 사항**:
- ✅ 테이블이 로봇 앞쪽에 배치되는지
- ✅ YCB 물체가 랜덤 위치에 생성되는지
- ✅ 초록색 타겟 영역이 보이는지
- ✅ 로봇이 정상 동작하는지

---

### 2. Easy Mode 학습 (1-2시간)

#### 2-1. 학습 스크립트 수정
`scripts/train_roarm_isaac_assets.py` 생성 (다음 단계)

#### 2-2. 학습 시작
```bash
./scripts/run_train_isaac_assets.sh --mode train --timesteps 50000 --level easy
```

#### 2-3. TensorBoard 모니터링 (병렬)
```bash
tensorboard --logdir logs/rl_training_isaac/tensorboard
# → http://localhost:6006
```

**주요 지표**:
- `rollout/ep_rew_mean`: 평균 보상 (증가 추세 확인)
- `rollout/success_rate`: 성공률 (목표: > 40%)
- `train/policy_loss`: Policy loss (감소 확인)

---

### 3. 학습 결과 평가 (5분)

```bash
./scripts/run_train_isaac_assets.sh --mode test --episodes 10
```

**기대 결과**:
- Success Rate: 40-50%
- Smooth한 동작
- 물체를 타겟까지 이동

---

## 🎓 Curriculum Learning 상세

### Level 1: Easy (초기 학습)
**물체**: 크래커 박스, 설탕 박스  
**특징**: 크고 안정적, 그리퍼로 잡기 쉬움  
**목표**: 기본 Pick & Place 동작 학습

**예상 성능**:
- 10K steps: Success Rate ~10%
- 30K steps: Success Rate ~30%
- 50K steps: Success Rate ~40-50%

---

### Level 2: Medium (일반화)
**물체**: Easy + 캔 + 병  
**특징**: 다양한 크기/무게/형상  
**목표**: 물체 불변 정책 학습

**예상 성능**:
- 50K steps: Success Rate ~20%
- 100K steps: Success Rate ~30-40%

---

### Level 3: Hard (실전)
**물체**: 모든 YCB 물체  
**배경**: Warehouse 환경  
**특징**: 시각적 복잡도 높음  
**목표**: Robust한 정책

---

## 💡 학습 팁

### 1. Curriculum 순서 지키기
```
Easy (50K) → Medium (100K) → Hard (50K)
```
한 번에 어려운 환경으로 가지 말고 단계적으로!

### 2. Hyperparameter 조정
- **Learning Rate**: `3e-4` (기본) → 수렴 느리면 `5e-4`
- **n_steps**: `2048` (기본) → GPU 좋으면 `4096`
- **batch_size**: `64` (기본) → 메모리 충분하면 `128`

### 3. Reward Shaping
현재 보상 함수:
```python
reward = -distance * 10.0        # Dense feedback
       + 100.0 if success         # Success bonus
       - 50.0 if dropped          # Penalty
```

**개선 아이디어**:
- 그리퍼-물체 거리 추가
- 물체 높이 유지 보상
- 부드러운 동작 보상 (action smoothness)

### 4. Success Criteria 조정
현재: `distance < 5cm`

**단계적 완화**:
- Easy: 10cm (학습 초기)
- Medium: 5cm (현재)
- Hard: 3cm (최종 목표)

---

## 🔧 문제 해결

### 문제 1: 로봇이 물체에 닿지 못함
**원인**: Forward Kinematics 부정확

**해결**:
1. `_get_ee_position()` 개선 (정확한 FK)
2. Observation에 그리퍼-물체 거리 추가
3. Reward에 접근 보상 추가

---

### 문제 2: 그리퍼가 물체를 놓치는
**원인**: Gripper 제어 불안정

**해결**:
1. Gripper velocity 더 느리게 (0.5 → 0.2)
2. Gripper에 별도 Reward 추가
3. Action space에 gripper position 직접 제어

---

### 문제 3: 학습 속도가 느림
**원인**: Warehouse 환경 렌더링 부담

**해결**:
```python
env = RoArmPickPlaceIsaacEnv(
    use_warehouse=False,    # 학습 중엔 간단한 환경
    render=False            # Headless 모드
)
```

---

### 문제 4: Success Rate가 낮음
**원인**: 너무 어려운 물체/환경

**해결**:
1. Easy level로 돌아가기
2. Success criteria 완화 (5cm → 10cm)
3. 더 긴 학습 (50K → 100K)
4. Reward 함수 개선

---

## 📊 벤치마크 목표

| Level | Steps | Success Rate | Avg Reward |
|-------|-------|--------------|------------|
| Easy  | 50K   | 40-50%       | > 0        |
| Medium| 100K  | 30-40%       | > -20      |
| Hard  | 150K  | 20-30%       | > -40      |

---

## 🎬 다음 단계

1. **Phase 1 완료 후**:
   - [ ] Easy mode 학습 (50K steps)
   - [ ] Success Rate 40% 달성
   - [ ] Best model 저장

2. **Phase 2 진입**:
   - [ ] Medium mode 학습 (100K steps)
   - [ ] 4가지 물체로 일반화
   - [ ] Transfer learning 활용

3. **Phase 3 도전**:
   - [ ] Warehouse 배경 추가
   - [ ] Domain Randomization
   - [ ] Real-to-Sim 준비

---

**작성**: GitHub Copilot (Jarvis)  
**검증**: Phase별 순차 진행 권장  
**업데이트**: 2025-10-19
