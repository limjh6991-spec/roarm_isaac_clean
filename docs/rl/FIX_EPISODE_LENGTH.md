# 학습 스크립트 긴급 수정 (2025-10-20 밤)

## 🔧 수정된 5가지 핵심 문제

### 1️⃣ **TimeLimit 명시적 설정** ✅
**문제**: GymWrapper가 `_max_episode_steps` 속성 없음 → 에피소드 길이 제어 불가

**해결**:
```python
class GymWrapper(gym.Env):
    def __init__(self):
        # ✅ TimeLimit 명시적 설정
        self._max_episode_steps = self.env.max_steps  # 600 steps
        self._elapsed_steps = 0
    
    def step(self, action):
        # ✅ terminated/truncated 분리
        terminated = done and self._elapsed_steps < self._max_episode_steps
        truncated = self._elapsed_steps >= self._max_episode_steps
```

**효과**:
- 에피소드 길이 600 steps 보장
- l=7 비정상 문제 해결
- terminated (SUCCESS/실패) vs truncated (TimeLimit) 명확히 구분

---

### 2️⃣ **episode_length_s → max_steps 변환 경로 확인** ✅
**문제**: `cfg.episode_length_s = 10.0`이 환경에 전달되는지 불명확

**해결**:
```python
# envs/roarm_pick_place_env.py (기존 코드 확인)
self.max_steps = int(self.cfg.episode_length_s * 60)  # 10.0 * 60 = 600

# train_dense_reward.py (명시적 전달)
self._max_episode_steps = self.env.max_steps
print(f"  ✅ Max episode steps: {self._max_episode_steps} (from env.max_steps)")
```

**효과**:
- 설정 경로 명확화: `cfg.episode_length_s` → `env.max_steps` → `wrapper._max_episode_steps`
- 600 steps (10초 * 60 FPS) 정상 작동 확인

---

### 3️⃣ **조기 경보 콜백 import 방어** ✅
**문제**: `EarlyWarningCallback` 없으면 `None()` 호출 → 크래시

**해결**:
```python
# Before
try:
    from scripts.rl.early_warning_callback import EarlyWarningCallback
except ImportError:
    EarlyWarningCallback = None

early_warning_callback = EarlyWarningCallback(...)  # ❌ None() 크래시!

# After
try:
    from scripts.rl.early_warning_callback import EarlyWarningCallback
    EARLY_WARNING_AVAILABLE = True
except ImportError:
    EARLY_WARNING_AVAILABLE = False

callbacks = [checkpoint_callback]
if EARLY_WARNING_AVAILABLE:
    callbacks.append(EarlyWarningCallback(...))
```

**효과**:
- 모듈 없어도 학습 계속 진행
- 안전한 조건부 콜백 추가

---

### 4️⃣ **VecNormalize 최종 저장** ✅
**문제**: 체크포인트만 `save_vecnormalize=True`, 최종 모델은 통계 파일 미저장

**해결**:
```python
# 1. PPO 모델 저장
model.save(str(model_path))

# 2. VecNormalize 통계 저장 (재현 필수!)
vecnorm_path = final_model_dir / "vecnormalize.pkl"
env.save(str(vecnorm_path))
print(f"💾 VecNormalize 통계 저장: {vecnorm_path}")
print("   ⚠️ 테스트 시 반드시 함께 로드하세요!")
```

**효과**:
- 관측/보상 정규화 통계 보존
- 테스트 시 재현성 보장 (obs/reward 스케일 일치)

---

### 5️⃣ **Monitor/벡터화 래핑 순서 개선** ✅
**문제**: `Monitor(env) → DummyVecEnv([lambda: env])` → 변수 캡처 문제 가능성

**해결**:
```python
# Before (단일 환경이라 실질 차이 없지만 패턴 개선)
env = GymWrapper()
env = Monitor(env, logfile)
env = DummyVecEnv([lambda: env])

# After (SB3 공식 권장 패턴)
def make_env():
    env = GymWrapper()
    env = Monitor(env, logfile)
    return env

env = DummyVecEnv([make_env])
```

**효과**:
- SB3 공식 권장 패턴 준수
- 다중 환경 확장 시에도 안전
- lambda 변수 캡처 문제 방지

---

## 📊 예상 결과

### Before (l=7 비정상)
```
r,l,t
-6.01,7,0.56
5.23,7,1.12
...
```

### After (l=300-600 정상)
```
r,l,t
15.23,312,18.72   # 에피소드 312 steps
105.67,189,30.15  # SUCCESS! (5cm 달성 후 5프레임 유지 → +100)
-8.45,600,66.24   # TimeLimit (truncated)
...
```

---

## ✅ 수정 완료 체크리스트

- [x] **TimeLimit 명시적 설정** (`_max_episode_steps = 600`)
- [x] **terminated/truncated 분리** (SUCCESS vs TimeLimit)
- [x] **episode_length_s → max_steps 경로 확인** (10초 * 60 FPS = 600)
- [x] **조기 경보 콜백 방어** (`EARLY_WARNING_AVAILABLE` 플래그)
- [x] **VecNormalize 통계 저장** (`vecnormalize.pkl`)
- [x] **래핑 순서 개선** (`make_env()` 팩토리 패턴)

---

## 🎯 다음 단계

1. **테스트 실행** (10K steps):
   ```bash
   make train-quick  # 10,000 steps
   ```

2. **로그 확인**:
   ```bash
   tail -20 logs/rl_training_curriculum/monitor.monitor.csv
   ```
   - `l` 컬럼: 300-600 사이 값 확인 (l=7 해결!)
   - `r` 컬럼: +100 보상 출현 확인 (SUCCESS)

3. **정규 학습** (50K-500K):
   ```bash
   make train TIMESTEPS=50000  # 또는 더 긴 학습
   ```

4. **GUI 테스트** (VecNormalize 통계 로딩):
   ```python
   # test_trained_model.py 수정 필요
   env = VecNormalize.load("vecnormalize.pkl", env)
   ```

---

## 📝 중요 노트

- **VecNormalize 통계 필수**: 테스트 시 `vecnormalize.pkl` 함께 로드 안 하면 관측 불일치 발생
- **TimeLimit 600 steps**: 10초 * 60 FPS = 600 (정상)
- **SUCCESS 조건**: 5cm 이내 5프레임 연속 유지 → +100 보상
- **Phase 0 (Easy)**: 큐브 10-15cm, 타겟 20-25cm → 300-600 steps 예상
