# Easy Mode RL 학습 가이드

**작성일**: 2025-10-19  
**목적**: RoArm-M3 간단한 RL 학습 실행 및 모니터링

---

## 🚀 빠른 시작

### 1. 학습 시작 (백그라운드)

```bash
cd ~/roarm_isaac_clean

# 50K steps 학습 (백그라운드)
PYTHONUNBUFFERED=1 nohup ~/isaacsim/python.sh scripts/simple_train.py --timesteps 50000 > logs/training.log 2>&1 &

# PID 확인
echo $!
```

**예상 소요 시간**: 2-4시간 (GPU 성능에 따라)

---

## 📊 학습 모니터링 (3가지 방법)

### 방법 1: 자동 모니터링 스크립트 (권장)

```bash
# 10초마다 자동 업데이트
bash scripts/monitor_training.sh

# 또는 1회만 확인
bash scripts/monitor_training.sh --once
```

**출력 예시**:
```
📊 RoArm-M3 RL 학습 모니터링
================================================
업데이트 시간: 2025-10-19 14:30:00

🔍 학습 상태 확인...
✅ 학습 진행 중
   PID: 12345
   5.2  12.3  00:45:30 python.sh scripts/simple_train.py

📈 학습 통계 (Monitor 파일):
  총 에피소드: 45
  평균 보상: -234.5
  최근 10개 평균: -189.2
  최고 보상: -87.3

💾 저장된 체크포인트:
  logs/rl_training/checkpoints/roarm_ppo_40000_steps.zip (1.2M, Oct 19 14:25)
  logs/rl_training/checkpoints/roarm_ppo_35000_steps.zip (1.2M, Oct 19 14:20)
  ...
```

---

### 방법 2: 로그 파일 확인

```bash
# 실시간 로그 확인
tail -f logs/training.log

# 주요 정보만 필터링
tail -f logs/training.log | grep -E "(Episode|steps|완료|저장)"
```

**출력 예시**:
```
📊 Episode 10 | Total steps: 6000
  ⏱️  7000 steps completed...
📊 Episode 20 | Total steps: 12000
  ⏱️  13000 steps completed...
💾 체크포인트 저장: roarm_ppo_15000_steps.zip
```

---

### 방법 3: 최신 플롯 자동 확인

```bash
# 10분마다 자동으로 플롯 업데이트 및 열기
watch -n 600 "cd ~/roarm_isaac_clean && ~/isaacsim/python.sh scripts/plot_training.py && xdg-open logs/rl_training/training_progress.png"
```

**플롯에서 확인 가능한 정보**:
- 에피소드별 보상 그래프 (+ 이동 평균)
- 에피소드별 길이 변화
- 누적 학습 시간
- 통계 요약 (평균, 최고, 개선도)

---

## 🔍 학습 상태 확인

### 프로세스 확인

```bash
# Python 프로세스 확인
ps aux | grep "simple_train.py" | grep -v grep

# GPU 사용량 확인
nvidia-smi

# 리소스 사용량 확인 (htop 설치 필요)
htop
```

### 학습 진행률 계산

```bash
# Monitor 파일에서 에피소드 수 확인
EPISODES=$(tail -n +2 logs/rl_training/monitor/0.monitor.csv | wc -l)
echo "완료된 에피소드: $EPISODES"

# 체크포인트에서 스텝 수 확인
ls -t logs/rl_training/checkpoints/*.zip | head -1
# 파일명에서 스텝 수 확인: roarm_ppo_35000_steps.zip → 35,000 / 50,000 = 70%
```

---

## 💾 저장된 파일 위치

```
logs/rl_training/
├── tensorboard/              # TensorBoard 로그
│   └── PPO_1/
├── checkpoints/              # 체크포인트 (5000 steps마다)
│   ├── roarm_ppo_5000_steps.zip
│   ├── roarm_ppo_10000_steps.zip
│   └── ...
├── final_model/              # 최종 모델
│   └── roarm_ppo_final.zip
├── monitor/                  # Monitor 로그
│   └── 0.monitor.csv
└── interrupted_model.zip     # 중단 시 저장된 모델 (있으면)
```

---

## ⏸️ 학습 중단/재개

### 학습 중단

```bash
# 프로세스 찾기
ps aux | grep "simple_train.py" | grep -v grep

# 중단 (PID 확인 후)
kill -INT <PID>

# 또는 모든 학습 프로세스 중단
pkill -f "simple_train.py"
```

**중단 시**: `logs/rl_training/interrupted_model.zip`에 자동 저장

---

### 학습 재개 (현재 미지원)

**Note**: 현재 스크립트는 처음부터 시작만 지원합니다.

재개가 필요하면:
1. 저장된 모델 로드 후 계속 학습
2. 또는 처음부터 더 긴 timesteps로 재시작

```bash
# 예: 100K steps로 재시작
PYTHONUNBUFFERED=1 nohup ~/isaacsim/python.sh scripts/simple_train.py \
  --timesteps 100000 > logs/training.log 2>&1 &
```

---

## 📈 성능 지표

### 좋은 학습의 신호

1. **평균 보상 증가**:
   - 초기: -300 ~ -400
   - 중간: -200 ~ -100
   - 좋음: -50 이상
   - 성공: 0 이상

2. **에피소드 길이 변화**:
   - 초기: 600 (max_steps)
   - 개선: 200-400 (조기 성공)

3. **TensorBoard**:
   - `ep_rew_mean` 상승 추세
   - `loss` 감소 후 안정화

---

### 문제의 신호

1. **보상이 계속 -400 이하**:
   - 로봇이 아무것도 학습하지 못함
   - 하이퍼파라미터 조정 필요

2. **보상 그래프가 불안정**:
   - Learning rate 너무 높음
   - Batch size 조정 필요

3. **학습이 멈춤**:
   - 프로세스 확인
   - GPU 메모리 부족 가능성

---

## 🎯 학습 완료 후

### 1. 결과 확인

```bash
# 최종 통계
bash scripts/monitor_training.sh --once

# 로그 요약
tail -100 logs/training.log | grep -E "(완료|저장|평균)"
```

### 2. 모델 테스트

```bash
# 최종 모델로 테스트 (데모와 유사하게 실행)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/demo_roarm_fixed.py \
  --episodes 10 --steps 200

# 또는 별도 테스트 스크립트 작성
```

### 3. 모델 백업

```bash
# 중요한 모델 백업
cp logs/rl_training/final_model/roarm_ppo_final.zip \
   ~/roarm_models/roarm_ppo_easy_$(date +%Y%m%d).zip
```

---

## 🐛 트러블슈팅

### Q: 학습이 시작되지 않음
```bash
# 로그 확인
tail -50 logs/training.log

# Isaac Sim 프로세스 확인
ps aux | grep isaac

# 이전 프로세스 정리
pkill -f "isaac-sim"
pkill -f "python.sh"
```

### Q: GPU 메모리 부족
```bash
# GPU 메모리 확인
nvidia-smi

# 해결: headless 모드 이미 사용 중
# 다른 GPU 프로세스 종료 필요
```

### Q: 학습이 너무 느림
- **예상 속도**: 50-100 steps/sec
- **너무 느리면**: 
  - GPU 사용 확인 (`nvidia-smi`)
  - 다른 프로세스 종료
  - `n_steps` 하이퍼파라미터 조정

---

## 📝 다음 단계

### Easy Mode 완료 후

1. **결과 분석**:
   - 학습 곡선 이미지 검토 (`plot_training.py`)
   - 성공률 계산
   - 평균 보상 확인

2. **Medium Mode로 진행**:
   ```bash
   # 더 어려운 환경으로 학습
   # (별도 스크립트 필요)
   ```

3. **하이퍼파라미터 튜닝**:
   - Learning rate 조정
   - Network architecture 변경
   - Reward shaping 개선

---

## 📖 참고 자료

- [Stable-Baselines3 문서](https://stable-baselines3.readthedocs.io/)
- [PPO 알고리즘 논문](https://arxiv.org/abs/1707.06347)
- [Isaac Sim 문서](https://docs.omniverse.nvidia.com/isaacsim/)
- `docs/ISAAC_ASSETS_RL_GUIDE.md` - 전체 RL 가이드
