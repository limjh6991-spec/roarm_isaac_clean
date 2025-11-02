# 🚀 Vision RL 학습 실행 가이드

**날짜**: 2025-11-02  
**준비 상태**: ✅ 완료  
**실행 대기**: ⏳

---

## 📊 현재 상황 요약

### ✅ 완료된 작업
1. ✅ **Hardware Integration** - RoArm-M3 + D405 카메라
2. ✅ **Camera System** - RGB-D 캡처 및 전처리 (84×84)
3. ✅ **Environment** - `simple_vision_env.py` (Gymnasium 호환)
4. ✅ **CNN Network** - `NatureCNN` (512 features)
5. ✅ **Training Script** - SAC 학습 파이프라인
6. ✅ **Launcher Scripts** - 자동화된 실행 스크립트

### 📁 핵심 파일
```
roarm_isaac_clean/
├── envs/
│   └── simple_vision_env.py          # Vision 환경 ✅
├── models/
│   └── cnn_extractor.py               # NatureCNN ✅
├── scripts/
│   ├── start_vision_rl.sh             # 🎯 메인 실행 스크립트 ✅
│   ├── launch_vision_rl.sh            # 모드별 실행 ✅
│   ├── train/
│   │   └── train_vision_sac.py        # SAC 학습 ✅
│   └── test/
│       ├── preflight_vision_rl.py     # Pre-flight check ✅
│       ├── test_vision_env.py         # 환경 테스트 ✅
│       └── test_vision_quick.py       # Quick 테스트 ✅
└── VISION_RL_STATUS.md                # 상태 문서 ✅
```

---

## 🎯 실행 방법 (3단계)

### 🚀 방법 1: 통합 스크립트 (권장!)

```bash
cd /home/roarm_m3/roarm_isaac_clean
bash scripts/start_vision_rl.sh
```

**특징**:
- 🎨 예쁜 UI와 단계별 안내
- ✅ 자동 환경 체크
- 🧪 Pre-flight check 실행
- 🎯 3가지 모드 선택 (Test/Quick/Full)
- 💡 실시간 도움말

---

### ⚡ 방법 2: 직접 모드 선택

#### 2-1. Environment Test (2분)
```bash
bash scripts/launch_vision_rl.sh --test
```

#### 2-2. Quick Training (30분) - 추천!
```bash
bash scripts/launch_vision_rl.sh --quick
```

#### 2-3. Full Training (5-10시간)
```bash
bash scripts/launch_vision_rl.sh --train
```

---

### 🔧 방법 3: 수동 실행 (고급)

#### Step 1: Pre-flight Check
```bash
cd /home/roarm_m3/roarm_isaac_clean
/home/roarm_m3/isaacsim/python.sh scripts/test/preflight_vision_rl.py
```

#### Step 2: Environment Test
```bash
/home/roarm_m3/isaacsim/python.sh scripts/test/test_vision_env.py
```

#### Step 3: Training
```bash
/home/roarm_m3/isaacsim/python.sh scripts/train/train_vision_sac.py
```

---

## 📊 학습 모니터링

### TensorBoard 실행 (다른 터미널)
```bash
cd /home/roarm_m3/roarm_isaac_clean
tensorboard --logdir output/train_vision_sac/
```

**브라우저**: http://localhost:6006

**주요 지표**:
- `rollout/ep_rew_mean` - Episode reward (상승해야 함!)
- `train/actor_loss` - Actor loss
- `train/critic_loss` - Critic loss
- `eval/mean_reward` - Evaluation reward

### 실시간 로그 확인
```bash
tail -f output/train_vision_sac/LATEST/training.log
```

---

## 🎯 Success Criteria

### Quick Training (50K steps)
- [ ] ✅ No crash
- [ ] ✅ Mean reward 상승
- [ ] ✅ Loss 안정화
- [ ] 🎯 Mean reward > -200

### Full Training (500K steps)
- [ ] 🎯 Reach object (distance < 10cm)
- [ ] 🎯 Grasp success (1회 이상)
- [ ] 🎯 Mean reward > -100
- [ ] 🚀 Pick & Place 성공

---

## 🚨 Troubleshooting

### Issue 1: Pre-flight check 실패
**증상**: Environment 초기화 에러

**해결**:
```bash
# Isaac Sim Python 확인
ls -l /home/roarm_m3/isaacsim/python.sh

# 환경 변수 확인
echo $LD_LIBRARY_PATH

# 재시도
/home/roarm_m3/isaacsim/python.sh scripts/test/preflight_vision_rl.py
```

### Issue 2: Camera data empty
**증상**: `rgb.shape = (0, 0, 0)`

**해결**: 첫 실행 시 정상 (warm-up 필요)

### Issue 3: Training 느림
**증상**: < 100 FPS

**해결**:
```python
# train_vision_sac.py 수정
simulation_app = SimulationApp({
    "headless": True,  # GUI 끄기
})
```

### Issue 4: GPU 미사용
**증상**: Training이 너무 느림

**확인**:
```bash
nvidia-smi
watch -n 1 nvidia-smi
```

---

## 📚 문서 및 참고 자료

### 주요 문서
- `VISION_RL_STATUS.md` - 현재 상태 (이 문서)
- `docs/VISION_RL_PLAN.md` - 전체 계획
- `docs/VISION_SAC_GUIDE.md` - SAC 구현 가이드
- `docs/VISION_RL_SUMMARY.md` - API 변경사항

### 코드 참고
- `scripts/test/test_vision_quick.py` - 검증된 vision 테스트
- `envs/simple_vision_env.py` - Environment 구현
- `models/cnn_extractor.py` - CNN 구현

---

## 🎬 실행 시나리오 (추천!)

### 시나리오 1: 첫 실행 (오늘)

```bash
# 1. 통합 스크립트 실행
cd /home/roarm_m3/roarm_isaac_clean
bash scripts/start_vision_rl.sh

# 2. Mode 선택
#    1) Test Mode → 환경 검증
#    2) Quick Training → 30분 학습

# 3. 결과 확인
ls -lh output/train_vision_sac/
```

**예상 소요 시간**: 40분  
**목표**: 환경 검증 + 학습 파이프라인 확인

---

### 시나리오 2: 본격 학습 (내일)

```bash
# 1. TensorBoard 실행 (터미널 1)
cd /home/roarm_m3/roarm_isaac_clean
tensorboard --logdir output/train_vision_sac/

# 2. Full Training 실행 (터미널 2)
bash scripts/launch_vision_rl.sh --train

# 3. 모니터링 (터미널 3)
watch -n 10 "tail -n 50 output/train_vision_sac/*/training.log"
```

**예상 소요 시간**: 5-10시간  
**목표**: Pick & Place 학습 완료

---

### 시나리오 3: Evaluation (학습 후)

```bash
# Best model 로드 및 테스트
/home/roarm_m3/isaacsim/python.sh scripts/test/eval_vision_model.py \
  --model output/train_vision_sac/YYYYMMDD_HHMMSS/best_model/best_model.zip \
  --episodes 10 \
  --render
```

---

## 🎯 다음 단계

### Immediate (오늘)
1. ⏳ **실행**: `bash scripts/start_vision_rl.sh`
2. ⏳ **검증**: Test Mode (2분)
3. ⏳ **학습**: Quick Training (30분)

### Short-term (이번 주)
4. ⏳ Full Training (500K steps)
5. ⏳ Hyperparameter tuning
6. ⏳ Sim-to-real 준비

### Long-term (다음 주+)
7. ⏳ Real hardware integration
8. ⏳ Agricultural environment
9. ⏳ Pepper harvesting demo

---

## 💡 Pro Tips

### Tip 1: Screen/Tmux 사용
```bash
# 장시간 학습은 screen 사용
screen -S vision_rl
bash scripts/launch_vision_rl.sh --train
# Ctrl+A, D로 detach

# 나중에 재접속
screen -r vision_rl
```

### Tip 2: 자동 재시작
```bash
# 학습 중 crash 대비
while true; do
    bash scripts/launch_vision_rl.sh --train
    echo "Restarting in 10 seconds..."
    sleep 10
done
```

### Tip 3: 로그 저장
```bash
bash scripts/launch_vision_rl.sh --train 2>&1 | tee training_log.txt
```

---

## ✅ Checklist

### 실행 전
- [ ] Isaac Sim 설치 확인
- [ ] CUDA/GPU 작동 확인
- [ ] 디스크 공간 확인 (>10GB)
- [ ] 문서 읽기 완료

### 실행 중
- [ ] Pre-flight check 통과
- [ ] Environment test 완료
- [ ] TensorBoard 모니터링
- [ ] GPU 사용률 확인

### 실행 후
- [ ] Best model 저장 확인
- [ ] Checkpoints 생성 확인
- [ ] Evaluation 실행
- [ ] 결과 분석 및 문서화

---

**Last Updated**: 2025-11-02 17:00  
**Status**: ✅ 준비 완료, 실행 가능!  
**Recommended**: `bash scripts/start_vision_rl.sh`

---

## 🎉 지금 바로 시작하세요!

```bash
cd /home/roarm_m3/roarm_isaac_clean
bash scripts/start_vision_rl.sh
```

**Good luck! 🚀**
