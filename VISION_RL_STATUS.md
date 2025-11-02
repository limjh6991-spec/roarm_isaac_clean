# Vision RL Training Status

**날짜**: 2025-11-02  
**Phase**: Vision-based RL 준비 완료 ✅

---

## 🎯 목표

RoArm-M3 + Intel RealSense D405를 활용한 Vision-based Pick & Place 강화학습

---

## ✅ 준비 완료 항목

### 1. Hardware Integration
- ✅ Intel RealSense D405 카메라 URDF 통합
- ✅ Camera 무게 보상 (Stiffness/Damping 튜닝)
- ✅ USD 파일 생성 및 Isaac Sim 로드 성공

### 2. Camera System
- ✅ IsaacLab Camera 센서 구현
- ✅ RGB + Depth 이미지 캡처 (480×640)
- ✅ 전처리 파이프라인 (84×84, [0,1] normalize)
- ✅ RGBD 스택 (4, 84, 84) 검증

### 3. Environment
- ✅ `simple_vision_env.py` - Gymnasium 환경 구현
- ✅ Observation space: Box(0, 1, (4, 84, 84))
- ✅ Action space: Box(-1, 1, (7,))
- ✅ Random policy 테스트 완료

### 4. Neural Network
- ✅ `models/cnn_extractor.py` - NatureCNN 구현
- ✅ Architecture: Conv(32) → Conv(64) → Conv(64) → Linear(512)
- ✅ Stable-Baselines3 호환
- ✅ Unit test 통과

### 5. Training Script
- ✅ `scripts/train/train_vision_sac.py` - SAC 학습 스크립트
- ✅ Hyperparameters 설정 (buffer=100K, batch=256)
- ✅ Callbacks (checkpoint, evaluation)
- ✅ TensorBoard 통합

### 6. Launcher
- ✅ `scripts/launch_vision_rl.sh` - 통합 실행 스크립트
- ✅ 3가지 모드: --test, --quick, --train

---

## 📋 실행 방법

### Step 1: Pre-flight Check (필수!)
```bash
cd /home/roarm_m3/roarm_isaac_clean
/home/roarm_m3/isaacsim/python.sh scripts/test/preflight_vision_rl.py
```

**예상 결과**:
- ✅ Environment 초기화
- ✅ Observation/Action space 검증
- ✅ CNN extractor 테스트
- ✅ gym.check_env 통과

### Step 2: Environment Test (권장)
```bash
bash scripts/launch_vision_rl.sh --test
```

**내용**:
- 10 episodes random policy
- 시간: ~2분
- RGB-D 이미지 저장 확인

### Step 3a: Quick Training (첫 실험)
```bash
bash scripts/launch_vision_rl.sh --quick
```

**내용**:
- SAC 50K steps
- 시간: ~30분
- 목표: 환경/학습 파이프라인 검증

### Step 3b: Full Training (본격 학습)
```bash
bash scripts/launch_vision_rl.sh --train
```

**내용**:
- SAC 500K steps
- 시간: ~5-10 hours
- 목표: Pick & Place 학습

---

## 📊 Training Monitoring

### TensorBoard
```bash
tensorboard --logdir output/train_vision_sac/
```

**주요 지표**:
- `rollout/ep_rew_mean` - Episode reward
- `train/actor_loss` - Actor loss
- `train/critic_loss` - Critic loss
- `eval/mean_reward` - Evaluation reward

### Checkpoints
```
output/train_vision_sac/YYYYMMDD_HHMMSS/
├── checkpoints/
│   ├── sac_vision_10000_steps.zip
│   ├── sac_vision_20000_steps.zip
│   └── ...
├── best_model/
│   └── best_model.zip
└── final_model/
    └── sac_vision_final.zip
```

---

## 🎯 Success Criteria

### Phase 1: Environment Validation (Day 1)
- [ ] Pre-flight check 통과
- [ ] 10 episodes 완료 (no crash)
- [ ] RGB-D 이미지 정상 생성

### Phase 2: Quick Training (Day 1-2)
- [ ] 50K steps 완료
- [ ] Mean reward 상승 트렌드
- [ ] No NaN/Inf in loss

### Phase 3: Full Training (Day 3-7)
- [ ] 500K steps 완료
- [ ] Reach object (distance < 10cm)
- [ ] Grasp success (1회 이상)
- [ ] Mean reward > -100

### Phase 4: Sim-to-Real (Day 8+)
- [ ] Transfer to real RoArm-M3
- [ ] Real D405 camera integration
- [ ] Real-world pick & place

---

## 📚 Documentation

### 주요 문서
- `docs/VISION_RL_PLAN.md` - Vision RL 전체 계획
- `docs/VISION_SAC_GUIDE.md` - SAC 구현 가이드
- `docs/VISION_RL_SUMMARY.md` - API 변경사항
- `docs/VISION_RL_START.md` - 오늘 작업 요약

### 참고 코드
- `scripts/test/test_vision_quick.py` - 검증된 vision 테스트
- `scripts/test/test_roarm_with_camera_isaaclab.py` - 카메라 통합

---

## 🚨 Troubleshooting

### Issue 1: Import Error
**증상**: `ModuleNotFoundError: No module named 'isaaclab'`

**해결**:
```bash
# Isaac Sim Python 사용 필수
/home/roarm_m3/isaacsim/python.sh your_script.py
```

### Issue 2: Camera Data Empty
**증상**: `rgb.shape = (0, 0, 0)`

**해결**:
```python
# 첫 프레임 대기
for _ in range(10):
    sim.step()
camera.update(sim.get_physics_dt())
```

### Issue 3: Slow Training
**증상**: < 100 FPS

**해결**:
- Headless 모드 사용 (`headless=True`)
- GPU 사용 확인 (`device='cuda'`)
- Batch size 조정 (256 → 128)

### Issue 4: NaN Loss
**증상**: Actor/Critic loss = NaN

**해결**:
- Learning rate 감소 (3e-4 → 1e-4)
- Reward clipping 추가
- Observation normalization 확인

---

## 📝 Next Actions

### Immediate (오늘)
1. ✅ Pre-flight check 실행
2. ⏳ Environment test (--test)
3. ⏳ Quick training (--quick, 50K steps)

### Short-term (이번 주)
4. ⏳ Full training (--train, 500K steps)
5. ⏳ Hyperparameter tuning
6. ⏳ Model evaluation

### Long-term (다음 주+)
7. ⏳ Sim-to-real transfer
8. ⏳ Real hardware integration
9. ⏳ Agricultural environment (Phase 2)

---

**Last Updated**: 2025-11-02 16:40  
**Status**: 준비 완료, 실행 대기 중 ⏳
