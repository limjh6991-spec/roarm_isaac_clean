# V4 개선 사항 (2025-10-20)

## 🎯 목표
**V3 l=7 문제 해결**: SUCCESS 조건 강화로 정상적인 학습 유도

---

## 🔧 주요 변경 사항

### 1️⃣ **SUCCESS 조건 강화** ⭐⭐⭐
```python
# envs/roarm_pick_place_env.py

# Before (V3)
success_threshold: float = 0.05      # 5cm
# _check_done()에서 즉시 done=True

# After (V4)
success_threshold: float = 0.02      # 2cm (5cm → 2cm, 더 정밀!)
success_hold_frames: int = 10        # 10프레임 연속 유지 필수!
# _check_done()에서 N프레임 유지 확인 후 done=True
```

**효과**:
- ✅ 에피소드 길이 증가: 7 steps → 50-200 steps 예상
- ✅ SUCCESS 보상 획득: +100 보상 달성 가능
- ✅ 더 정밀한 제어 학습: 2cm 정확도 요구

---

### 2️⃣ **Phase 난이도 상향** ⭐⭐
```python
# Before (V3)
# Phase 0: Easy
easy_cube_distance: (0.10, 0.15)   # 10~15cm
easy_target_distance: (0.20, 0.25)  # 20~25cm

# Phase 1: Normal
normal_cube_distance: (0.25, 0.35)  # 25~35cm
normal_target_distance: (0.25, 0.35)

# After (V4)
# Phase 0: Easy (중간 거리)
easy_cube_distance: (0.15, 0.20)   # 15~20cm (더 멀게!)
easy_target_distance: (0.25, 0.30)  # 25~30cm (더 멀게!)

# Phase 1: Normal (먼 거리)
normal_cube_distance: (0.35, 0.50)  # 35~50cm (훨씬 더 멀게!)
normal_target_distance: (0.35, 0.50)
```

**효과**:
- ✅ 더 도전적인 태스크
- ✅ 일반화 성능 향상
- ✅ 로봇 작업 공간 전체 활용

---

### 3️⃣ **TimeLimit Truncation Info 추가** ⭐
```python
# scripts/rl/train_dense_reward.py

# Before
if truncated:
    print(f"  ⏱️ TimeLimit reached...")

# After
if truncated:
    info["TimeLimit.truncated"] = True  # ✅ SB3/Monitor 인식!
    print(f"  ⏱️ TimeLimit reached: {steps}/{max_steps} steps")
```

**효과**:
- ✅ SB3/Monitor가 TimeLimit 정식으로 인식
- ✅ monitor.csv 분석 용이
- ✅ terminated vs truncated 명확한 구분

---

### 4️⃣ **시드 전파 강화 (재현성)** ⭐
```python
# Before
if seed is not None:
    np.random.seed(seed)

# After
if seed is not None:
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed(seed)
```

**효과**:
- ✅ NumPy + PyTorch 시드 동기화
- ✅ 같은 설정 → 같은 결과 (디버깅 용이)
- ✅ 재현 가능한 실험

---

### 5️⃣ **로그 개선** ⭐
```python
# 환경 초기화 시 상세 정보
print(f"  - Episode length: {episode_length_s}s * 60 FPS = {max_steps} steps")
print(f"  - Success threshold: {threshold}m ({threshold*100}cm)")
print(f"  - Success hold frames: {hold_frames} (연속 유지)")
print(f"    • Cube: 15-20cm")
print(f"    • Target: 25-30cm")

# 에피소드 진행 로그 (10 → 50 에피소드마다, 스팸 줄이기!)
if self.episode_count % 50 == 0:
    print(f"📊 Episode {episode_count} | Total steps: {total_steps}")

# SUCCESS 유지 진행 상황
if self.success_frames % 5 == 0 and self.success_frames > 0:
    print(f"  ⏳ Holding... {frames}/{hold_frames} frames (dist: {dist:.3f}m)")
```

**효과**:
- ✅ max_steps 계산 근거 명확
- ✅ 콘솔 스팸 감소 (가독성↑, 성능↑)
- ✅ SUCCESS 유지 과정 추적

---

### 6️⃣ **_check_done() 로직 개선** ⭐⭐⭐
```python
# Before (V3)
def _check_done(self, obs):
    if cube_to_target_dist < self.cfg.success_threshold:
        print(f"  ✅ SUCCESS! Distance: {dist:.3f}m")
        return True  # ← 즉시 종료!

# After (V4)
def _check_done(self, obs):
    if cube_to_target_dist < self.cfg.success_threshold:
        # 아직 N프레임 유지 안 됨 → 계속 진행
        if self.success_frames < self.cfg.success_hold_frames:
            if self.success_frames % 5 == 0:
                print(f"  ⏳ Holding... {frames}/{hold_frames} frames")
            return False  # ← 계속 진행!
        else:
            # N프레임 유지 완료 → SUCCESS!
            print(f"  ✅ SUCCESS CONFIRMED! (held {frames} frames)")
            return True
```

**효과**:
- ✅ **핵심!** N프레임 연속 유지 필수
- ✅ 에피소드 길이 정상화 (7 → 50-200 steps)
- ✅ 더 안정적인 제어 학습

---

## 📊 예상 결과

### Before (V3)
```
ep_len_mean: 7.03
ep_rew_mean: -5.16
SUCCESS 보상: 0회 (5프레임 조건 미달성)
```

### After (V4) - 예상
```
ep_len_mean: 100-150 steps
ep_rew_mean: +30-50 (SUCCESS 보상 포함)
SUCCESS 보상: 60-80% (10프레임 유지 성공)
```

---

## 🎯 학습 계획

### 1단계: 빠른 검증 (10K steps)
```bash
~/isaacsim/python.sh scripts/rl/train_dense_reward.py --timesteps 10000
```
- **목표**: V4 개선 효과 확인
- **확인 사항**:
  - `ep_len_mean` 증가 (7 → 50+)
  - `ep_rew_mean` 증가 (-5 → 양수)
  - monitor.csv에서 `l` 분포 확인

### 2단계: 정규 학습 (50K-500K steps)
```bash
~/isaacsim/python.sh scripts/rl/train_dense_reward.py --timesteps 50000
```
- **목표**: Phase 0→1 승급 달성
- **예상 시간**: 30-40분 (50K)

### 3단계: GUI 테스트
```bash
~/isaacsim/python.sh scripts/rl/test_trained_model.py \
  --model logs/rl_training_curriculum/final_model/roarm_ppo_dense_final.zip \
  --vecnorm logs/rl_training_curriculum/final_model/vecnormalize.pkl
```

---

## ✅ 변경 파일 목록

1. **`envs/roarm_pick_place_env.py`**:
   - `success_threshold`: 0.05 → 0.02
   - `success_hold_frames`: 신규 추가 (10)
   - `easy_cube_distance`: (0.10, 0.15) → (0.15, 0.20)
   - `easy_target_distance`: (0.20, 0.25) → (0.25, 0.30)
   - `normal_cube_distance`: (0.25, 0.35) → (0.35, 0.50)
   - `normal_target_distance`: (0.25, 0.35) → (0.35, 0.50)
   - `_check_done()`: N프레임 유지 로직 추가
   - 로그 개선 (max_steps 계산, Phase 정보)

2. **`scripts/rl/train_dense_reward.py`**:
   - `reset()`: 시드 전파 (torch, cuda)
   - `step()`: `info["TimeLimit.truncated"]` 추가
   - 에피소드 로그 주기: 10 → 50

3. **`docs/rl/IMPROVEMENT_LOG_V4.md`**: 이 문서 (V4 개선 사항)

---

## 🔍 모니터링 포인트

### 콘솔 로그
```
✅ Max episode steps: 600 (from env.max_steps)
  - Episode length: 10.0s * 60 FPS = 600 steps
  - Success threshold: 0.02m (2cm)
  - Success hold frames: 10 (연속 유지)
  - Curriculum: Phase 0 (Easy)
    • Cube: 15-20cm
    • Target: 25-30cm
```

### 에피소드 진행
```
  ⏳ Holding... 5/10 frames (dist: 0.018m)
  ⏳ Holding... 10/10 frames (dist: 0.015m)
  🏆 Milestone: SUCCESS! (+100.0) [10 frames]
  ✅ SUCCESS CONFIRMED! Distance: 0.015m (held 10 frames)
```

### monitor.csv
```
r,l,t
25.3,87,52.1   # 에피소드 87 steps, 보상 +25
105.2,152,104.3  # SUCCESS! (+100 보상 포함)
-10.5,600,160.2  # TimeLimit (truncated)
```

---

## 🎉 기대 효과

1. **에피소드 길이 정상화**: 7 → 50-200 steps
2. **SUCCESS 보상 획득**: +100 보상 달성
3. **더 정밀한 제어**: 2cm 정확도 학습
4. **일반화 성능 향상**: 더 넓은 작업 공간
5. **재현 가능한 실험**: 시드 전파
6. **분석 용이성**: TimeLimit info, 로그 개선
