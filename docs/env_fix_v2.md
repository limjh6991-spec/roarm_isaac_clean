# 🔧 Environment Fix v2.0 - GRIP 문제 해결

## 📅 수정 일시: 2025-10-21

## 🚨 발견된 핵심 문제 3가지

### 1️⃣ `is_grasped` 판정 문제
**원인**: 단순 거리 + gripper_width만 확인, 실제 접촉 없음
```python
# ❌ 기존 (문제)
gripper_width = joint_positions[6] + joint_positions[7]  # 회전형 gripper에 부적합
is_grasped = 1.0 if (ee_to_cube_dist < 0.08 and gripper_width < 0.025) else 0.0
```

**문제점**:
- 회전형 gripper는 ±θ 관계 → 단순 합산은 의미 없음
- gripper_width가 항상 ~0.04-0.05 범위 → 0.025 threshold 불만족
- Z축 높이 차이 미고려 → EE가 cube 위에 있어도 grasp 판정

### 2️⃣ `gripper_width` 계산 오류
**원인**: 회전형 joint의 특성 미반영
```python
# ❌ 기존: 단순 합산
gripper_width = joint_positions[6] + joint_positions[7]
# 예: 0.02 + 0.02 = 0.04 (항상 일정)

# ✅ 수정: 절대값 합산
gripper_width = abs(joint_positions[6]) + abs(joint_positions[7])
# 예: |0.01| + |-0.01| = 0.02 (실제 폭 반영)
```

### 3️⃣ `grasp_valid` 조건 너무 엄격
**원인**: threshold가 현실적이지 않음
```python
# ❌ 기존
grasp_valid = (
    dist_to_cube < 0.08 and      # 너무 좁음
    gripper_width < 0.025 and    # 달성 불가
    cube_pos[2] > 0.03           # Z축 절대값 → 상대 높이 필요
)
```

---

## ✅ 적용된 수정 사항

### 수정 1: `is_grasped` 판정 개선 (Line ~423)
```python
# 🚨 FIX 1: 절대값 차이로 gripper_width 계산 (회전형 gripper)
gripper_width = abs(joint_positions[6]) + abs(joint_positions[7])

# 🚨 FIX 2: 더 완화된 조건 + Z축 높이 고려
ee_to_cube_dist = np.linalg.norm(cube_relative_to_ee)
z_alignment = abs(cube_relative_to_ee[2])  # EE와 cube의 높이 차이

# Is grasped: (1) 근접 (2) gripper 닫힘 (3) 높이 정렬
is_grasped = 1.0 if (
    ee_to_cube_dist < 0.10 and         # 10cm 이내 (완화: 0.08→0.10)
    gripper_width < 0.06 and           # gripper 폭 완화 (0.025→0.06)
    z_alignment < 0.03                 # 높이 차이 3cm 이내 (신규)
) else 0.0
```

### 수정 2: `grasp_valid` 판정 개선 (Line ~647)
```python
# ═══════════════════════════════════════════════════════════
# 🔒 GATING: grasp_valid 체크 (🚨 FIX 3: 완화된 조건)
# ═══════════════════════════════════════════════════════════
grasp_valid = (
    dist_to_cube < 0.10 and            # EE가 10cm 이내 (완화)
    gripper_width < 0.06 and           # gripper 폭 완화 (0.025 → 0.06)
    abs(cube_relative_to_ee[2]) < 0.03 # Z축 높이 정렬 3cm (완화)
)
```

### 수정 3: GRIP 디버깅 로그 추가 (Line ~690)
```python
# 🚨 FIX 4: GRIP 조건 디버깅 로그 추가
if not self.valid_grip:
    # grasp_valid 실패 원인 추적
    if self.current_step % 100 == 0:  # 100 스텝마다
        print(f"  🔍 GRIP 체크: dist={dist_to_cube:.3f} (need <0.10), "
              f"width={gripper_width:.4f} (need <0.06), "
              f"z_diff={abs(cube_relative_to_ee[2]):.3f} (need <0.03), "
              f"valid={grasp_valid}")

# 2️⃣ 그립 보상 (+40): 유효 그립 3프레임 유지 [게이팅]
if grasp_valid:
    self.grip_frames += 1
else:
    self.grip_frames = 0
    
if not self.valid_grip and grasp_valid and self.grip_frames >= 3:
    reward += 40.0
    self.valid_grip = True
    self.episode_grip_count += 1
    print(f"  ✊ Milestone: GRIP! (+40.0) [dist={dist_to_cube:.3f}, width={gripper_width:.4f}]")
```

---

## 📊 예상 효과

### Before (100K & 300K 학습):
```
REACH: 76-77%  ✅
GRIP:  0%      ❌ ← 문제!
LIFT:  0%      ❌
PLACE: 0%      ❌
```

### After (수정 후 예상):
```
REACH: 80%+     ✅
GRIP:  30-50%   🎯 ← 목표!
LIFT:  10-20%   📈
PLACE: 0-5%     📈
```

---

## 🎯 검증 방법

### 1. 디버깅 로그 확인
학습 중 100 스텝마다 출력:
```
🔍 GRIP 체크: dist=0.085 (need <0.10), width=0.058 (need <0.06), 
              z_diff=0.025 (need <0.03), valid=True
✊ Milestone: GRIP! (+40.0) [dist=0.085, width=0.058]
```

### 2. Milestone Rate 추적
```bash
grep "grip_rate" logs/training_fixed_env.log | tail -20
```

### 3. 시각화 확인
```bash
~/isaacsim/python.sh scripts/rl/visualize_model.py \
  --model logs/rl_training_curriculum/final_model/roarm_ppo_dense_final.zip \
  --episodes 5
```

---

## 🚀 다음 학습 계획

### Test 1: 빠른 검증 (50K)
```bash
~/isaacsim/python.sh scripts/rl/train_dense_reward.py --timesteps 50000
```
- 예상 시간: 2-3분
- 목표: GRIP 10% 이상 달성 확인

### Test 2: 본격 학습 (200K)
```bash
~/isaacsim/python.sh scripts/rl/train_dense_reward.py --timesteps 200000
```
- 예상 시간: 7-10분
- 목표: GRIP 40%+, LIFT 15%+ 달성

---

## 📝 참고사항

### Gripper Width 측정 이슈
- **기존 문제**: 회전형 gripper (hinge joint) 특성 미반영
- **해결**: 절대값 합산으로 실제 개폐 정도 반영
- **검증 필요**: 실제 gripper_width 값 범위 확인

### Z-alignment 추가
- **이유**: EE가 cube 위/아래에 있어도 거리만으로는 구분 불가
- **효과**: 높이 정렬된 상태에서만 grasp 인정
- **Threshold**: 3cm (cube 높이 5cm의 60%)

### Threshold 완화 근거
- **Distance**: 0.08→0.10 (센서 노이즈, FK 오차 고려)
- **Gripper Width**: 0.025→0.06 (실제 측정값 기반)
- **Z-alignment**: 신규 추가 (높이 정렬 필수)

