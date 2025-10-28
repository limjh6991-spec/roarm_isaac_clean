# RL 학습 및 결과 확인 워크플로우

**목표**: 학습은 빠르게(headless), 결과는 시각적으로(GUI) 확인

---

## 📚 전체 워크플로우

### 1단계: Headless로 빠른 학습 ⚡

```bash
cd ~/roarm_isaac_clean

# 50K steps 학습 (화면 없이, 백그라운드)
PYTHONUNBUFFERED=1 nohup ~/isaacsim/python.sh scripts/simple_train.py \
  --timesteps 50000 > logs/training.log 2>&1 &

# PID 확인
echo $!
```

**특징**:
- ✅ `headless=True` → 화면 없이 빠름
- ✅ 백그라운드 실행 (`&`)
- ✅ 로그 저장 (`logs/training.log`)
- ✅ 5000 steps마다 체크포인트 저장

**예상 시간**: 2-4시간 (GPU 성능에 따라)

---

### 2단계: 학습 진행 상황 모니터링 📊

학습이 진행되는 동안 3가지 방법으로 확인:

#### 방법 1: 모니터링 스크립트 (권장)
```bash
# 10초마다 자동 업데이트
bash scripts/monitor_training.sh
```

#### 방법 2: 학습 곡선 이미지
```bash
# 이미지 생성 및 확인
~/isaacsim/python.sh scripts/plot_training.py
xdg-open logs/rl_training/training_progress.png
```

#### 방법 3: 로그 파일
```bash
# 실시간 로그 확인
tail -f logs/training.log
```

---

### 3단계: GUI로 결과 확인 🎥

학습이 완료되면 **GUI 모드**로 결과 확인:

```bash
# 최종 모델 테스트 (5 에피소드)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --episodes 5
```

**Isaac Sim GUI 창이 열리면서 로봇 움직임을 실시간으로 볼 수 있습니다!**

#### 추가 옵션

```bash
# 느리게 보기 (동작 자세히 관찰)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --render-delay 0.2

# 특정 체크포인트 테스트
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training/checkpoints/roarm_ppo_40000_steps.zip

# 더 많은 에피소드
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --episodes 10
```

---

## 🎬 비교: 학습 진행 단계별 확인

학습이 진행되면서 성능이 개선되는지 확인:

```bash
# 1. 초기 모델 (5K steps) - 아직 학습 안 됨
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training/checkpoints/roarm_ppo_5000_steps.zip \
  --episodes 3

# 2. 중간 모델 (25K steps) - 어느 정도 학습됨
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training/checkpoints/roarm_ppo_25000_steps.zip \
  --episodes 3

# 3. 최종 모델 (50K steps) - 최대 성능
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --episodes 5
```

---

## 📂 파일 구조

```
학습 (Headless)
  ↓
logs/rl_training/
├── checkpoints/              # 5000 steps마다 저장
│   ├── roarm_ppo_5000_steps.zip
│   ├── roarm_ppo_10000_steps.zip
│   ├── ...
│   └── roarm_ppo_50000_steps.zip
├── final_model/              # 최종 모델
│   └── roarm_ppo_final.zip
├── monitor.monitor.csv       # 학습 통계
└── training_progress.png     # 학습 곡선 이미지
  ↓
GUI 확인 (test_trained_model.py)
```

---

## ⚡ 빠른 참조

| 작업 | 명령어 | 설명 |
|------|--------|------|
| **학습 시작** | `nohup ~/isaacsim/python.sh scripts/simple_train.py &` | Headless, 백그라운드 |
| **학습 확인** | `bash scripts/monitor_training.sh` | 진행 상황 모니터링 |
| **결과 GUI** | `~/isaacsim/python.sh scripts/test_trained_model.py` | 시각적 확인 |
| **느린 재생** | `--render-delay 0.2` 추가 | 동작 자세히 관찰 |
| **체크포인트** | `--model [경로]` | 특정 시점 모델 |

---

## 💡 핵심 포인트

### 학습 (Headless)
- ✅ **빠름**: 화면 렌더링 없음 → 2-3배 빠른 학습
- ✅ **백그라운드**: 다른 작업 가능
- ✅ **자동 저장**: 5000 steps마다 체크포인트
- ✅ **로그 기록**: 언제든지 진행 상황 확인

### 결과 확인 (GUI)
- ✅ **시각적**: 로봇 움직임을 눈으로 확인
- ✅ **분석**: 어디서 실패하는지 파악
- ✅ **비교**: 여러 모델 성능 비교
- ✅ **디버깅**: 문제점 발견

---

## 🎯 실전 예시

### 시나리오 1: 장시간 학습

```bash
# 1. 저녁에 학습 시작
cd ~/roarm_isaac_clean
PYTHONUNBUFFERED=1 nohup ~/isaacsim/python.sh scripts/simple_train.py \
  --timesteps 100000 > logs/training.log 2>&1 &

# 2. 다음 날 아침 결과 확인
bash scripts/monitor_training.sh --once

# 3. GUI로 최종 성능 확인
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --episodes 10
```

### 시나리오 2: 빠른 테스트

```bash
# 1. 짧게 학습 (10K steps)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/simple_train.py \
  --timesteps 10000

# 2. 바로 GUI로 확인
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --episodes 3
```

### 시나리오 3: 체크포인트 비교

```bash
# 각 체크포인트를 GUI로 확인하여 학습 진행 상황 파악
for checkpoint in logs/rl_training/checkpoints/*.zip; do
  echo "테스트: $checkpoint"
  PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
    --model "$checkpoint" \
    --episodes 2
done
```

---

## 🐛 문제 해결

### Q: 학습이 너무 느려요
- **확인**: `nvidia-smi`로 GPU 사용 확인
- **해결**: 다른 GPU 프로세스 종료

### Q: GUI가 열리지 않아요
- **확인**: `echo $DISPLAY`
- **해결**: `xhost +local:`

### Q: 로봇이 떨어져요
- **확인**: Joint drive 설정 확인
- **해결**: `ROBOT_FALLING_FIX.md` 참조

---

## 📖 관련 문서

- `EASY_MODE_TRAINING_GUIDE.md` - 전체 학습 가이드
- `MODEL_VISUALIZATION_GUIDE.md` - 시각화 상세 가이드
- `QUICK_VISUALIZATION.md` - 빠른 참조 가이드
- `ROBOT_FALLING_FIX.md` - 로봇 떨어짐 문제 해결

---

## ✅ 요약

```
학습: Headless (빠름) ⚡
  ↓
모니터링: 텍스트/이미지 📊
  ↓
확인: GUI (시각적) 🎥
```

이것이 **최적의 RL 워크플로우**입니다!
