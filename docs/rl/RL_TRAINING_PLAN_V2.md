# RoArm M3 강화학습 계획 V2

**목적**: 전문가 진단 프로세스를 통합한 체계적 학습 로드맵

**기반**: Pick-and-Place 환경 진단 체크리스트 + 성공 사례 분석

---

## 🎯 전체 목표

### 최종 목표
- **1M steps**: Pick & Place 성공률 60% 이상
- **평균 보상**: 100+ (ep_rew_mean)
- **안정적 학습**: 500K steps 이후 성능 수렴

### 중간 목표
- **50K steps**: REACH 마일스톤 10회 이상
- **200K steps**: GRIP 마일스톤 5회 이상
- **500K steps**: LIFT 마일스톤 3회 이상
- **1M steps**: SUCCESS 1회 이상

---

## 📋 Phase 0: 사전 진단 (필수)

**목적**: 환경 검증 후 학습 시작

### Checklist

#### ✅ 1. Observation Check
```bash
# 진단 명령
cd ~/roarm_isaac_clean/scripts/rl
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh diagnose_env.py --check observation
```

**확인 사항**:
- [ ] `cube_pos` 관측에 포함
- [ ] `cube_quat` (선택) 포함
- [ ] Observation dimension: 25 (현재) 또는 더 많음
- [ ] 값 범위 정상: cube_pos ∈ [0.05, 0.30]

**패치 (필요 시)**:
```python
# env.py - _compute_observations()
policy_obs = torch.cat([
    self.robot.data.joint_pos,
    self.robot.data.joint_vel,
    self.ee_pos,
    self.cube.data.root_pos_w,     # ✅ 추가
    self.goal_pos,
], dim=-1)
```

#### ✅ 2. Coordinate Frame Alignment
```bash
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh diagnose_env.py --check coordinates
```

**확인 사항**:
- [ ] EE와 큐브 모두 `/World` 좌표계
- [ ] 또는 모두 상대 좌표 (robot_base 기준)
- [ ] 거리 계산 일치: `torch.norm(ee_pos - cube_pos)`

**패치 (필요 시)**:
```python
# 상대 좌표로 통일
base_pos = self.robot.data.root_pos_w
obs["ee_pos"] = self.ee_frame.data.target_pos_w[..., 0, :] - base_pos
obs["cube_pos"] = self.cube.data.root_pos_w - base_pos
```

#### ✅ 3. Physics and Collision
```bash
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh diagnose_env.py --check physics
```

**확인 사항**:
- [ ] 큐브 `kinematic_enabled=False` (Dynamic)
- [ ] 큐브 mass > 0 (예: 0.1 kg)
- [ ] `collision_enabled=True`
- [ ] 그리퍼 마찰 계수: 0.6 (URDF v1.1)

**현재 상태**:
```python
# ✅ URDF v1.1 개선 완료 (2025-10-20)
# - 그리퍼 접촉 면적: 5배 증가
# - 마찰 계수: 0.6
# - 그리퍼 힘: 8 N·m
```

#### ✅ 4. Reward Signal
```bash
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh diagnose_env.py --check reward
```

**확인 사항**:
- [ ] Dense reaching reward 있음
- [ ] Milestone 보상: REACH/GRIP/LIFT/GOAL/SUCCESS
- [ ] 1회성 보상 플래그 작동 (`self.reached`, `self.gripped` 등)
- [ ] Time penalty: -0.01

**현재 문제**: GRIP 조건 너무 엄격

**패치 계획**:
```python
# env.py - compute_reward()

# ❌ 현재 (엄격)
grasp_valid = (
    (ee_to_cube_dist < 0.08) &
    (gripper_width < 0.02) &
    (cube_pos[:, 2] > 0.03)
)

# ✅ 개선 (완화)
grasp_valid = (
    (ee_to_cube_dist < 0.10) &      # 8cm → 10cm
    (gripper_width < 0.03) &         # 2cm → 3cm
    (cube_pos[:, 2] > 0.02)          # 3cm → 2cm
)
```

#### ✅ 5. Task Integration
```bash
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh diagnose_env.py --check integration
```

**확인 사항**:
- [ ] `self.cube` 등록됨
- [ ] `self.scene.rigid_objects["cube"]` 존재
- [ ] `_reset_idx()` 큐브 초기화 정상
- [ ] Episode 길이: 200 steps (적절)

---

## 🚀 Phase 1: 초기 학습 (0 ~ 50K steps)

**목표**: 
- REACH 마일스톤 10회 이상 달성
- 평균 보상: -10 → 0

### 1.1 학습 시작

```bash
cd ~/roarm_isaac_clean/scripts/rl
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh train_dense_reward.py --timesteps 50000 2>&1 | tee /tmp/training_phase1.log
```

### 1.2 모니터링 (별도 터미널)

```bash
# TensorBoard
tensorboard --logdir=~/roarm_isaac_clean/scripts/rl/tensorboard_logs

# 주요 지표:
# - ep_rew_mean: 증가 추세
# - ep_len_mean: 초기 200 → 점차 감소
# - value_loss: 안정적 감소
# - entropy: 높음 (탐험 중)
```

### 1.3 예상 결과

| Metric | 초기 (0~10K) | 중간 (10K~30K) | 후반 (30K~50K) |
|--------|-------------|---------------|---------------|
| **ep_rew_mean** | -20 | -10 | 0 |
| **REACH 달성** | 0~5회 | 5~10회 | 10~20회 |
| **GRIP 달성** | 0회 | 0~1회 | 목표 |

### 1.4 Phase 1 평가 기준

✅ **통과 조건**:
- REACH 마일스톤 10회 이상
- ep_rew_mean > -5

❌ **실패 조건**:
- REACH 5회 미만 → Reaching reward 강화
- ep_rew_mean < -15 → 보상 스케일 조정

---

## 🔧 Phase 2: GRIP 디버깅 (50K ~ 100K steps)

**목표**:
- GRIP 마일스톤 5회 이상 달성
- 평균 보상: 0 → 10

### 2.1 진단 (Phase 1 종료 후)

```bash
# 마일스톤 통계 확인
grep "REACH\|GRIP\|LIFT" /tmp/training_phase1.log | tail -50

# Expected:
# REACH: 15회 (0.015% of 100K steps)
# GRIP: 0~2회 (예상)
```

### 2.2 GRIP 조건 완화 (필요 시)

```python
# scripts/rl/env.py

# Option 1: 조건 완화
grasp_valid = (
    (ee_to_cube_dist < 0.10) &      # 완화
    (gripper_width < 0.03) &         # 완화
    (cube_pos[:, 2] > 0.02)          # 완화
)

# Option 2: Contact force 추가
cube_contact_forces = self.cube.get_net_contact_forces()
contact_force_norm = torch.norm(cube_contact_forces, dim=-1)

grasp_valid = (
    (ee_to_cube_dist < 0.10) &
    (gripper_width < 0.03) &
    (contact_force_norm > 0.5)      # 접촉력 감지
)
```

### 2.3 재학습 (패치 후)

```bash
# 기존 모델에서 이어서 학습
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh train_dense_reward.py \
    --timesteps 50000 \
    --load-model models/ppo_roarm_50k.zip \
    2>&1 | tee /tmp/training_phase2.log
```

### 2.4 Phase 2 평가 기준

✅ **통과 조건**:
- GRIP 마일스톤 5회 이상
- ep_rew_mean > 5

❌ **실패 조건**:
- GRIP 3회 미만 → URDF 재검토 또는 보상 함수 재설계

---

## 📈 Phase 3: 안정화 (100K ~ 500K steps)

**목표**:
- LIFT 마일스톤 3회 이상
- 평균 보상: 10 → 50

### 3.1 연속 학습

```bash
# 100K → 500K (총 400K steps 추가)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh train_dense_reward.py \
    --timesteps 400000 \
    --load-model models/ppo_roarm_100k.zip \
    2>&1 | tee /tmp/training_phase3.log
```

### 3.2 Curriculum Phase 1 도입 (선택)

**조건**: Phase 0 성공률 60% 달성 시

```python
# env_cfg.py

# Phase 0 (현재)
CUBE_INITIAL_POS_RANGE = [[0.15, -0.05, 0.05], [0.20, 0.05, 0.05]]
GOAL_POS_RANGE = [[0.25, -0.05, 0.10], [0.30, 0.05, 0.15]]

# Phase 1 (더 어려움)
CUBE_INITIAL_POS_RANGE = [[0.10, -0.10, 0.05], [0.25, 0.10, 0.05]]
GOAL_POS_RANGE = [[0.20, -0.15, 0.10], [0.40, 0.15, 0.20]]
```

### 3.3 하이퍼파라미터 튜닝 (필요 시)

**증상**: 학습 정체 (ep_rew_mean 평평)

**해결책**:
```python
# train_dense_reward.py

# Option 1: 탐험 증가
model = PPO(
    ...,
    ent_coef=0.05,  # 0.01 → 0.05
)

# Option 2: 학습률 감소 (Over-fitting 방지)
model = PPO(
    ...,
    learning_rate=1e-4,  # 3e-4 → 1e-4
)
```

### 3.4 Phase 3 평가 기준

✅ **통과 조건**:
- LIFT 마일스톤 3회 이상
- ep_rew_mean > 30
- 학습 곡선 안정적 증가

---

## 🎯 Phase 4: 최종 달성 (500K ~ 1M steps)

**목표**:
- SUCCESS 마일스톤 1회 이상
- 평균 보상: 50 → 100+
- 성공률: 60%

### 4.1 연속 학습

```bash
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh train_dense_reward.py \
    --timesteps 500000 \
    --load-model models/ppo_roarm_500k.zip \
    2>&1 | tee /tmp/training_phase4.log
```

### 4.2 Domain Randomization 추가 (선택)

```python
# env.py - _reset_idx()

# 큐브 속성 랜덤화
cube_mass = torch.rand(len(env_ids)) * 0.1 + 0.05  # 50g ~ 150g
cube_friction = torch.rand(len(env_ids)) * 0.4 + 0.4  # 0.4 ~ 0.8

# 적용
self.cube.set_mass(env_ids, cube_mass)
self.cube.set_friction(env_ids, cube_friction)
```

### 4.3 최종 평가

```bash
# 100 에피소드 평가
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh test_trained_model.py \
    --model models/ppo_roarm_1m.zip \
    --episodes 100 \
    --save-video \
    2>&1 | tee /tmp/evaluation_1m.log
```

**성공 기준**:
- 성공률: 60회 / 100 에피소드 = 60%
- 평균 보상: 100+
- 평균 에피소드 길이: 100~150 steps

---

## 🐛 문제 해결 플레이북

### 문제 1: REACH 미달성 (50K에서 10회 미만)

**진단**:
```bash
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh diagnose_env.py --check observation
```

**가능한 원인**:
1. 큐브 위치가 관측에 없음 → `cube_pos` 추가
2. 좌표계 불일치 → 상대 좌표로 변환
3. Reaching reward 약함 → `-0.1 * dist` → `-0.5 * dist`

### 문제 2: GRIP 미달성 (100K에서 5회 미만)

**진단**:
```bash
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh diagnose_env.py --check reward --verbose
```

**가능한 원인**:
1. grasp_valid 조건 엄격 → 완화 (위 참조)
2. URDF 그리퍼 문제 → v1.1 확인 (✅ 완료)
3. 보상 신호 부족 → Contact force 추가

### 문제 3: 학습 불안정 (value_loss 폭발)

**해결책**:
```python
# PPO 파라미터 조정
clip_range = 0.1        # 0.2 → 0.1 (더 보수적)
max_grad_norm = 0.3     # 0.5 → 0.3
learning_rate = 1e-4    # 3e-4 → 1e-4
```

### 문제 4: Over-fitting (eval 성능 ≪ train 성능)

**해결책**:
1. Domain Randomization 추가
2. `ent_coef` 증가 (0.01 → 0.05)
3. Early Stopping (ep_rew_mean 수렴 시 중단)

---

## 📊 모니터링 대시보드

### TensorBoard 필수 지표

```bash
tensorboard --logdir=./tensorboard_logs --port=6006
```

**그래프**:
1. **ep_rew_mean**: 평균 보상 (↑ 목표)
2. **ep_len_mean**: 평균 길이 (초기: 200 → 학습 후: 100~150)
3. **value_loss**: Critic 손실 (안정적 감소)
4. **policy_loss**: Actor 손실 (작은 변화)
5. **entropy**: 탐험 정도 (초기: 높음 → 학습 후: 낮음)

### 커스텀 로그 (추가)

```python
# train_dense_reward.py - Callback

class MilestoneCallback(BaseCallback):
    def _on_step(self):
        if self.n_calls % 1000 == 0:
            self.logger.record("milestones/reach_count", self.env.reach_count)
            self.logger.record("milestones/grip_count", self.env.grip_count)
            self.logger.record("milestones/lift_count", self.env.lift_count)
            self.logger.record("diagnostics/mean_ee_to_cube", self.env.ee_to_cube_dist.mean())
        return True
```

---

## 📅 타임라인 (RTX 5090 기준)

| Phase | Steps | 예상 시간 | 누적 시간 | 목표 |
|-------|-------|----------|----------|------|
| **Phase 0** | 진단 | 30분 | 0.5h | 환경 검증 |
| **Phase 1** | 50K | 1시간 | 1.5h | REACH 10회 |
| **Phase 2** | 50K | 1시간 | 2.5h | GRIP 5회 |
| **Phase 3** | 400K | 8시간 | 10.5h | LIFT 3회 |
| **Phase 4** | 500K | 10시간 | 20.5h | SUCCESS 1회 |
| **Total** | 1M | **~21h** | - | **성공률 60%** |

---

## 🎓 학습 체크리스트

### 학습 전
- [ ] DIAGNOSTIC_CHECKLIST.md 모든 항목 통과
- [ ] URDF v1.1 확인 (그리퍼 개선)
- [ ] VecNormalize 활성화
- [ ] TensorBoard 설정
- [ ] 백업 디렉토리 생성 (`models/backup/`)

### 학습 중 (매 50K)
- [ ] ep_rew_mean 증가 추세 확인
- [ ] 마일스톤 달성 횟수 확인
- [ ] value_loss 안정적
- [ ] 모델 저장 (`ppo_roarm_<steps>.zip`)

### 학습 후
- [ ] 최종 평가 (100 에피소드)
- [ ] 비디오 녹화 (`--save-video`)
- [ ] 성공률 60% 달성 확인
- [ ] README 업데이트

---

## 🔗 참고 문서

1. **DIAGNOSTIC_CHECKLIST.md**: 7단계 환경 진단
2. **REWARD_DESIGN_GUIDE.md**: 보상 함수 설계
3. **PPO_HYPERPARAMETERS_CHEATSHEET.md**: 파라미터 튜닝
4. **SUCCESS_CASES.md**: 성공 사례 분석

---

## 🚀 즉시 실행 명령

```bash
# 1. 사전 진단
cd ~/roarm_isaac_clean/scripts/rl
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh diagnose_env.py --full

# 2. Phase 1 시작 (진단 통과 후)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh train_dense_reward.py --timesteps 50000

# 3. TensorBoard (별도 터미널)
tensorboard --logdir=./tensorboard_logs
```

---

**작성자**: GitHub Copilot  
**최종 업데이트**: 2025-10-20  
**상태**: ✅ 사전 진단 준비 완료, Phase 1 시작 대기
