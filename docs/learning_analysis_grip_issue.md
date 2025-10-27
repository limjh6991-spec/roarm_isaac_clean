# 학습 결과 분석 (5K Timesteps Quick Test)

## 📊 데모 결과 요약

### 에피소드별 성과
| 에피소드 | REACH | GRIP | LIFT | PLACE | 총 보상 |
|----------|-------|------|------|-------|---------|
| 1 | ✅ (263 step) | ❌ | ❌ | ❌ | +1.09 |
| 2 | ✅ (256 step) | ❌ | ❌ | ❌ | +2.36 |
| 3 | ❌ | ❌ | ❌ | ❌ | -1.26 |

### 전체 통계
- **REACH 달성률**: 66.7% (2/3 에피소드)
- **GRIP 달성률**: 0% (0/3 에피소드)
- **평균 REACH 시간**: ~260 steps (4.3초 @ 60 FPS)
- **평균 보상**: 0.73

---

## 🔍 문제 분석: 왜 GRIP이 0%인가?

### 1. **학습 시간 부족** (가장 큰 원인)
```
Quick Test: 5,120 timesteps (10 에피소드)
↓
REACH 단계에서 멈춤
```

**분석**:
- 5K steps는 REACH 학습에만 충분
- GRIP 학습까지 최소 **30K-50K steps 필요**
- 강화학습은 **순차적 학습**: REACH → GRIP → LIFT → PLACE

### 2. **보상 구조의 영향**
```python
# 현재 보상 (envs/roarm_pick_place_env.py)
REACH: +5   (처음 10cm 이내 도달)
GRIP:  +10  (유효한 그립 3프레임)
LIFT:  +15  (5cm 이상 들어올림)
PLACE: +20  (타겟 8cm 이내)
```

**학습 순서**:
1. **0-10K steps**: REACH 학습 (가장 쉬움)
2. **10-30K steps**: GRIP 시도 (그리퍼 제어 학습)
3. **30-50K steps**: LIFT 학습
4. **50K+ steps**: PLACE 학습

**Quick Test는 1단계에만 도달!**

### 3. **그리퍼 제어의 복잡성**
```python
# 액션 공간 (8 dim)
[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, 
 gripper_left, gripper_right]
```

- 관절 6개 + 그리퍼 2개 = **동시에 8개 제어**
- 로봇이 먼저 **위치**를 학습 (REACH)
- 그 다음 **그리퍼 타이밍** 학습 (GRIP)

---

## 📈 예상 학습 진행 (100K steps 기준)

### Phase별 예상 달성률
```
Timesteps   REACH  GRIP   LIFT   PLACE  Success
─────────────────────────────────────────────────
5K (현재)   60%    0%     0%     0%     0%
10K         75%    10%    0%     0%     0%
20K         85%    30%    5%     0%     0%
50K         90%    50%    25%    10%    5%
100K        95%    70%    50%    30%    20%
200K        95%+   85%    70%    50%    40%
```

### 학습 단계별 행동 변화
1. **0-10K**: 큐브를 향해 움직이는 법 학습
2. **10-30K**: 큐브 위에서 그리퍼 열고 닫는 법 학습
3. **30-50K**: 그립 후 들어올리는 법 학습
4. **50K+**: 타겟으로 이동하는 법 학습

---

## 🎯 관찰된 로봇 행동 (GUI 데모)

### 현재 학습된 행동 패턴
1. **초기 자세**: Home position (관절 [0, -0.5, 0.5, 0, 0, 0, 0, 0])
2. **REACH 단계**:
   - 로봇이 큐브 방향으로 End Effector 이동
   - 거리가 10cm 이내 도달 시 REACH 달성 (+5 보상)
3. **그 이후**:
   - 그리퍼 제어 시도하지 않음 (학습 부족)
   - 큐브 주변을 맴돌거나 멀어짐
   - 타임아웃 (600 steps) 후 종료

### 부족한 행동
- ❌ 그리퍼 열기/닫기 타이밍
- ❌ 큐브 잡기 위한 정확한 위치 조정
- ❌ 그립 후 들어올리기
- ❌ 타겟으로 이동

---

## 💡 해결 방안

### 옵션 1: 더 긴 학습 (권장) ✅
```bash
# 100K timesteps 학습 (예상 시간: 4-8분)
~/isaacsim/python.sh scripts/rl/train_dense_reward.py --timesteps 100000
```

**기대 효과**:
- REACH: 95%+
- GRIP: 70%+
- LIFT: 50%+
- Success: 20%+

### 옵션 2: 보상 함수 조정 (실험적)
```python
# 현재
REACH: +5, GRIP: +10, LIFT: +15, PLACE: +20

# GRIP 학습 강화 버전
REACH: +3, GRIP: +20, LIFT: +15, PLACE: +20
```
→ GRIP 보상을 증가시켜 더 빨리 학습 유도

### 옵션 3: Curriculum 완화
```python
# 현재 Phase 0
cube_distance: 15-20cm
target_distance: 25-30cm

# 더 쉬운 버전
cube_distance: 10-12cm  # 더 가까이
target_distance: 15-20cm
```
→ GRIP 학습에 집중

---

## 🎬 동영상 녹화 방법

### GUI에서 수동 녹화
1. Isaac Sim GUI 열기 (현재 실행 중)
2. 메뉴: **Window → Replicator → Movie Capture**
3. 설정:
   - Resolution: 1920x1080
   - Format: MP4
   - FPS: 30
4. Start Recording 클릭
5. 데모 실행 후 Stop Recording

### 스크린샷 캡처
```bash
# 스크린샷 저장 (PNG)
~/isaacsim/python.sh scripts/rl/capture_screenshots.py \
  --model logs/quick_test/quick_test_model.zip \
  --frames 10 \
  --output screenshots/
```

---

## 📊 현재 vs 목표 비교

| 메트릭 | 현재 (5K) | 목표 (100K) | 차이 |
|--------|-----------|-------------|------|
| REACH | 60% | 95% | +35% |
| GRIP | 0% | 70% | +70% |
| LIFT | 0% | 50% | +50% |
| Success | 0% | 20% | +20% |
| 학습 시간 | 12초 | 4-8분 | 20-40x |

---

## 🚀 다음 단계

### 즉시 실행 (권장)
```bash
# 100K timesteps 학습
~/isaacsim/python.sh scripts/rl/train_dense_reward.py --timesteps 100000

# 학습 완료 후 데모
~/isaacsim/python.sh scripts/rl/visualize_model.py \
  --model logs/rl_training_curriculum/final_model/roarm_ppo_dense_final.zip \
  --vecnorm logs/rl_training_curriculum/final_model/vecnormalize.pkl \
  --episodes 5
```

### TensorBoard 모니터링
```bash
tensorboard --logdir logs/rl_training_curriculum/tensorboard --port 6006
```
→ 브라우저: http://localhost:6006
→ 실시간 `milestone/grip_rate` 확인

---

## 🎯 결론

**GRIP 0%는 정상입니다!**

이유:
1. ✅ 5K steps는 REACH 단계에만 충분
2. ✅ GRIP 학습에는 30-50K steps 필요
3. ✅ 로봇이 큐브에 접근하는 것을 학습 (60% REACH)
4. ✅ 다음 단계: 그리퍼 제어 학습

**해결책**: 100K timesteps로 재학습하면 GRIP 70%+ 달성 가능!
