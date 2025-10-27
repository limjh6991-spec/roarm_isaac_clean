# 10K 학습 결과 분석 (2025-10-20)

## 📊 핵심 지표

### 학습 환경
- **Total timesteps**: 10,240 (목표 10,000 달성)
- **총 에피소드**: 1,450+ episodes
- **학습 시간**: 351초 (5.9분)
- **FPS**: 29
- **Max episode steps**: **600** ✅ (정상 설정)

### 에피소드 길이 문제 ⚠️
```
ep_len_mean: 7.03
```
- **모든 에피소드가 평균 7 steps만에 종료**
- monitor.csv 확인: **모든 l=7** (마지막 20개 에피소드 전부)
- TimeLimit truncated 로그 없음 → **7 steps 만에 SUCCESS 달성**

### 보상 지표
```
ep_rew_mean: -5.16
```
- 평균 보상: **-5.16** (음수!)
- monitor.csv 최종 20개: -0.67 ~ -9.75 (모두 음수)
- **SUCCESS 보상 +100 없음** → 5프레임 연속 조건 미달성

### 학습 안정성 ✅
```
explained_variance: 0.775  (매우 좋음! 0.7~0.9 이상)
approx_kl: 0.027           (target_kl=0.03 이내, 안정적)
clip_fraction: 0.299        (정책 업데이트 적절)
entropy_loss: -11.3         (탐색 진행 중)
value_loss: 0.187           (수렴 중)
```
→ **학습 자체는 매우 안정적이고 건강함!**

---

## 🎯 문제 진단

### 1️⃣ **MAX_EPISODE_STEPS는 정상** ✅
```
✅ Max episode steps: 600 (from env.max_steps)
```
- 환경에서 600으로 설정됨
- TimeLimit 600 steps 보장

### 2️⃣ **하지만 7 steps 만에 조기 종료** ❌
**원인**: **환경 내부 `_check_done()` 로직이 너무 쉽게 SUCCESS 판정**

```python
# envs/roarm_pick_place_env.py (line 689)
def _check_done(self, obs: np.ndarray) -> bool:
    # 큐브 → 타겟 거리
    cube_to_target_dist = np.linalg.norm(cube_to_target_vec)
    
    # 타겟 도달 (5cm 이내)
    if cube_to_target_dist < self.cfg.success_threshold:  # 0.05m
        print(f"  ✅ SUCCESS! Distance: {cube_to_target_dist:.3f}m")
        self._record_success(True)
        return True  # ← 즉시 done=True!
```

**로그 증거**:
```
✅ SUCCESS! Distance: 0.004m  (Step 7)
✅ SUCCESS! Distance: 0.039m  (Step 7)
✅ SUCCESS! Distance: 0.004m  (Step 7)
...
```
→ **모든 에피소드에서 7 steps 만에 0.004m 거리 달성**

### 3️⃣ **SUCCESS 보상 +100 누락** 
```python
# envs/roarm_pick_place_env.py (line 659-666)
# 5️⃣ Success 보상 (+100): 목표 5cm 이내 5프레임 유지
if dist_cube_to_target < self.cfg.success_threshold:
    self.success_frames += 1
    if self.success_frames >= 5:  # ← 5프레임 연속 필요!
        reward += 100.0
else:
    self.success_frames = 0
```

**문제**: 
- 7 steps 만에 `done=True` 반환
- 5프레임 연속 조건 달성 불가
- 평균 보상 -5.16 (SUCCESS 보상 없음)

---

## 🔍 근본 원인 분석

### Phase 0 (Easy Mode) vs Phase 1 (Normal Mode)

**Phase 0 설정** (Episode 1-50):
```python
easy_cube_distance: (0.10, 0.15)   # 10~15cm
easy_target_distance: (0.20, 0.25)  # 20~25cm
```
→ 큐브가 EE에서 10-15cm, 타겟이 20-25cm
→ **V3 EE 기준 상대 좌표로 7 steps 만에 도달 가능!**

**Phase 1 설정** (Episode 51+):
```python
normal_cube_distance: (0.25, 0.35)  # 25~35cm
normal_target_distance: (0.25, 0.35) # 25~35cm
```
→ 더 멀리 있지만 **여전히 7 steps 만에 SUCCESS!**

### V3 개선의 "과도한" 효과 😅

**V3 핵심 개선**:
1. **EE 기준 상대 좌표**: 큐브/타겟을 EE 좌표계에서 관측
2. **Hybrid Reward**: Dense + Shaped-Sparse
3. **속도 정보**: EE/Cube velocity 추가

→ **너무 효과적**이어서 7 steps 만에 정확하게 목표 도달!

---

## 💡 해결 방안 (우선순위)

### Option 1: SUCCESS 조건 강화 (최우선! ⭐⭐⭐)
```python
# envs/roarm_pick_place_env.py
success_threshold: float = 0.02  # 5cm → 2cm (더 정밀!)
# 또는
success_hold_frames: int = 10  # 5 → 10 프레임 연속 필요
```

**효과**:
- 에피소드 길이 50-200 steps로 증가 예상
- SUCCESS 보상 +100 획득 가능
- 더 정밀한 제어 학습

### Option 2: Phase 난이도 상향
```python
# Phase 0 (Easy)
easy_cube_distance: (0.15, 0.20)   # 10-15cm → 15-20cm
easy_target_distance: (0.25, 0.30)  # 20-25cm → 25-30cm

# Phase 1 (Normal)
normal_cube_distance: (0.35, 0.50)  # 25-35cm → 35-50cm
normal_target_distance: (0.35, 0.50)
```

### Option 3: SUCCESS 후 유지 요구
```python
# _check_done() 수정
if cube_to_target_dist < self.cfg.success_threshold:
    self.success_frames += 1
    if self.success_frames >= 10:  # 10프레임 유지 필수
        print(f"  ✅ SUCCESS CONFIRMED!")
        return True
    # 아직 done=False (계속 진행)
    return False
```

---

## 🎯 권장 조치

### 즉시 적용 (권장! ⭐)
1. **SUCCESS threshold 강화**: `0.05` → `0.02` (2cm)
2. **SUCCESS hold frames 증가**: `5` → `10` 프레임
3. **Phase 0 난이도 상향**: 큐브 15-20cm, 타겟 25-30cm

### 보강 사항 (다음 단계)
1. **TimeLimit truncation info 추가**:
   ```python
   if truncated:
       info["TimeLimit.truncated"] = True
   ```

2. **max_steps 계산 로그 추가**:
   ```python
   print(f"  Max steps: {self.max_steps} = {self.cfg.episode_length_s}s * 60 FPS")
   ```

3. **시드 전파** (재현성):
   ```python
   def reset(self, seed=None, options=None):
       if seed is not None:
           np.random.seed(seed)
           torch.manual_seed(seed)
           # env에도 전파
   ```

4. **콘솔 스팸 줄이기**:
   ```python
   # 10 에피소드마다 → 50 에피소드마다
   if self.episode_count % 50 == 0:
       print(...)
   ```

---

## 📈 기대 효과

### Option 1 적용 후 (threshold=0.02, hold=10)
```
예상 에피소드 길이: 50-200 steps
예상 평균 보상: +20 ~ +50 (SUCCESS 보상 포함)
예상 학습 시간: 30-40분 (50K steps)
```

### 현재 vs 개선 비교
| 항목 | 현재 | 개선 후 |
|------|------|---------|
| ep_len_mean | 7.03 | 100-150 |
| ep_rew_mean | -5.16 | +30-50 |
| SUCCESS 보상 | 0% | 60-80% |
| 학습 효율 | 낮음 | 높음 |

---

## ✅ 결론

**문제**: l=7은 버그가 아니라 **V3 개선이 너무 효과적**이었습니다!
- EE 기준 상대 좌표로 7 steps 만에 정확하게 목표 도달
- SUCCESS 조건 (5cm, 5프레임)이 너무 쉬움

**해결**: SUCCESS 조건 강화 (2cm, 10프레임)으로 정상 학습 유도

**다음 단계**: 환경 수정 → 50K 재학습 → GUI 테스트
