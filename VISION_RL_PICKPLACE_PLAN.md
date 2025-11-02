# Vision RL Pick and Place 계획

**날짜**: 2025-11-02  
**Phase**: Pick and Place 환경 완성 ✅

---

## 🎯 목표

RoArm-M3 + Intel RealSense D405 카메라를 활용한 **Vision-based Pick & Place** 강화학습

---

## ✅ 완성된 Pick and Place 환경

### 1. Scene 구성
```
/World/
├── Ground              # 바닥
├── Light               # 조명 (3000 intensity)
├── Table               # 테이블 (0.8×0.8×0.4m, gray)
├── Cube                # 🔴 Pick 대상 (5×5×5cm, red, mass=0.05kg)
├── Target              # 🟢 Goal 위치 (6×6×0.1cm, green marker)
└── RoArm/
    ├── base_link
    ├── ... (6 joints)
    ├── gripper_link
    └── camera_link     # 📷 D405 카메라
```

### 2. 초기 위치
- **Cube**: (0.3, 0.0, 0.45) ± 10cm randomization
- **Target**: (-0.3, 0.0, 0.45) - 고정
- **Robot**: Home position (j2=0.5, j3=-0.5)
- **Distance**: Cube ↔ Target = 60cm

### 3. 작업 목표
1. **Reach**: Gripper → Cube (< 5cm)
2. **Grasp**: Contact and hold cube
3. **Lift**: Cube height > 5cm
4. **Transport**: Move to target area
5. **Place**: Cube → Target (< 5cm) ✅ **Success!**

---

## 🎁 Reward System (Shaped + Sparse)

### Reward Components

```python
# 1. Reach Reward (Dense)
reach_reward = -dist_to_cube * 2.0
# 예: 30cm → -60, 5cm → -10

# 2. Grasp Bonus (Sparse)
if dist_to_cube < 0.05:  # 5cm
    reward += 5.0

# 3. Lift Bonus (Sparse)
if cube_height > initial_height + 0.05:  # 5cm lifted
    reward += 10.0

# 4. Place Reward (Dense)
if dist_cube_to_target < 0.1:  # 10cm
    reward += 20.0

# 5. Success Bonus (Sparse)
if dist_cube_to_target < 0.05:  # 5cm
    reward += 100.0

# 6. Time Penalty
reward -= 0.01  # Encourage efficiency
```

### Reward Range
- **Worst case**: -60 - 0.01*200 = -62 (far away, max steps)
- **Reach only**: -10 (5cm from cube)
- **Grasp**: -10 + 5 = -5
- **Lift**: -10 + 5 + 10 = 5
- **Near target**: 5 + 20 = 25
- **Success**: 5 + 10 + 20 + 100 = 135 ✅

---

## 📊 Observation Space

### RGB-D Image (4, 84, 84)
- **RGB**: 3 channels, [0, 1] normalized
- **Depth**: 1 channel, [0, 1] normalized (0.1-2.0m range)
- **Source**: D405 camera mounted on gripper
- **View**: First-person view from gripper

### Vision Features
- Cube (red) visible in camera
- Target (green) visible in camera
- Table (gray) in background
- Robot links in peripheral

---

## 🎮 Action Space

### 7 DOF Control
- **Type**: Continuous, Box(-1, 1, (7,))
- **Control**: Delta position (±0.05 rad/step)
- **Joints**: [j1, j2, j3, j4, j5, j6, gripper]
- **Frequency**: 60 Hz (dt=1/60)

---

## 🏁 Success Criteria

### Episode Termination
- **Success**: `dist_cube_to_target < 0.05` (5cm)
- **Truncation**: `episode_step >= 200` (max steps)

### Training Milestones
1. **Reach**: Mean reward > -20 (gripper approaches cube)
2. **Grasp**: Mean reward > 0 (contact with cube)
3. **Lift**: Mean reward > 10 (cube lifted)
4. **Place**: Mean reward > 30 (cube near target)
5. **Success**: Success rate > 10% (cube at target)

---

## 🚀 Training Plan

### Phase 1: Environment Test (오늘, 5분)
```bash
/home/roarm_m3/isaacsim/python.sh scripts/test/test_vision_env.py
```

**검증 항목**:
- [ ] Cube spawned at correct position
- [ ] Target visible in green
- [ ] Camera captures RGB-D
- [ ] Reward calculation works
- [ ] Success detection works

### Phase 2: Quick Training (오늘, 30분)
```bash
bash scripts/launch_vision_rl.sh --quick
```

**목표**:
- SAC 50K steps
- Mean reward 상승 확인
- Reach milestone 달성 (reward > -20)

### Phase 3: Full Training (내일~, 5-10시간)
```bash
bash scripts/launch_vision_rl.sh --train
```

**목표**:
- SAC 500K steps
- Success rate > 10%
- Pick and Place 성공!

---

## 📈 Expected Learning Curve

### 0-100K steps: **Exploration**
- Random movements
- Reward: -60 ~ -40
- Milestone: Robot moves towards general direction

### 100K-200K steps: **Reach Learning**
- Gripper approaches cube
- Reward: -40 ~ -20
- Milestone: Consistent reaching (< 10cm)

### 200K-300K steps: **Grasp Learning**
- Contact with cube
- Reward: -20 ~ 0
- Milestone: Grasp attempts

### 300K-400K steps: **Lift Learning**
- Cube lifted occasionally
- Reward: 0 ~ 20
- Milestone: Lift success > 5%

### 400K-500K steps: **Place Learning**
- Complete pick and place
- Reward: 20 ~ 100
- Milestone: Success rate > 10% ✅

---

## 🔧 Hyperparameters (SAC)

### Network
- **CNN**: NatureCNN (512 features)
- **Actor**: 512 → 256 → 7
- **Critic**: 512 → 256 → 1

### Training
- **Algorithm**: SAC (off-policy)
- **Buffer size**: 100K transitions
- **Batch size**: 256
- **Learning rate**: 3e-4
- **Discount (γ)**: 0.99
- **Tau (τ)**: 0.005
- **Train freq**: 1 (every step)
- **Learning starts**: 10K steps

### Environment
- **Max episode steps**: 200
- **Action repeat**: 1
- **FPS target**: 60
- **Device**: CUDA (GPU)

---

## 📁 주요 파일

```
envs/simple_vision_env.py           # ✅ Pick and Place 환경
  ├── Scene setup (table, cube, target)
  ├── Reward calculation (shaped + sparse)
  ├── Success detection
  └── Randomization (cube position)

scripts/train/train_vision_sac.py   # SAC 학습 스크립트
scripts/test/test_vision_env.py     # 환경 테스트
scripts/launch_vision_rl.sh         # 실행 런처
```

---

## 🧪 Test Script

```bash
# 1. Environment 테스트
/home/roarm_m3/isaacsim/python.sh scripts/test/test_vision_env.py

# 예상 출력:
# ✅ Cube spawned at (0.30, 0.00, 0.45)
# ✅ Target at (-0.30, 0.00, 0.45)
# ✅ Distance: 0.60m
# ✅ Observation shape: (4, 84, 84)
# ✅ Reward range: [-62, 135]
# ✅ Episode completed (200 steps)
```

---

## 💡 Key Improvements

### 이전 버전 vs 현재 버전

| 항목 | 이전 | 현재 |
|------|------|------|
| **Scene** | Robot only | Table + Cube + Target ✅ |
| **Task** | None | Pick & Place ✅ |
| **Reward** | Motion penalty | Shaped + Sparse ✅ |
| **Success** | None | Distance < 5cm ✅ |
| **Info** | Steps only | Distances + Success ✅ |
| **Randomization** | None | Cube position ±10cm ✅ |

---

## 🎯 다음 단계

### Immediate (지금!)
```bash
# 1. 변경사항 커밋
git add envs/simple_vision_env.py
git commit -m "feat: add pick and place task to vision env"

# 2. 환경 테스트
/home/roarm_m3/isaacsim/python.sh scripts/test/test_vision_env.py

# 3. Quick training
bash scripts/launch_vision_rl.sh --quick
```

### Short-term (오늘)
- Quick training 결과 분석
- Reward 그래프 확인
- Reach milestone 달성 확인

### Long-term (내일~)
- Full training (500K steps)
- Success rate 측정
- Hyperparameter tuning
- Sim-to-real 준비

---

**Status**: ✅ Pick and Place 환경 완성!  
**Ready for**: 학습 시작! 🚀
