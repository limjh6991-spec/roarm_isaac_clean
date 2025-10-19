# 학습된 모델 시각화 가이드

**작성일**: 2025-10-19  
**목적**: 학습된 RL 모델을 영상/GUI로 확인

---

## 🎥 3가지 시각화 방법

### 방법 1: GUI로 실시간 확인 (가장 간단) ⭐

학습된 모델이 어떻게 동작하는지 **실시간**으로 확인합니다.

```bash
# 최종 모델 테스트 (5 에피소드)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --episodes 5

# 특정 체크포인트 테스트
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training/checkpoints/roarm_ppo_40000_steps.zip \
  --episodes 3

# 느리게 보기 (0.1초 딜레이)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --render-delay 0.1
```

**출력 예시**:
```
✅ Isaac Sim GUI 모드 초기화 완료

📂 모델 로딩: logs/rl_training/final_model/roarm_ppo_final.zip
✅ 모델 로드 완료

🎬 5개 에피소드 테스트 시작...

📊 Episode 1/5
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ 에피소드 완료:
  보상: -2456.32
  길이: 601 steps

...

📊 테스트 결과 요약
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
총 에피소드: 5
성공: 0/5 (0.0%)

보상 통계:
  평균: -2786.45
  최고: -2401.23
  최저: -3012.56
```

**Isaac Sim 창이 열리면서 로봇 움직임을 실시간으로 볼 수 있습니다!**

---

### 방법 2: 비디오 녹화 (공유용)

학습된 모델의 실행을 **MP4 비디오**로 저장합니다.

```bash
# 기본 녹화 (3 에피소드, 30 FPS)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/record_model_video.py

# 커스텀 설정
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/record_model_video.py \
  --episodes 5 \
  --fps 60 \
  --output logs/videos/my_model_demo.mp4

# 특정 체크포인트 녹화
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/record_model_video.py \
  --model logs/rl_training/checkpoints/roarm_ppo_50000_steps.zip \
  --episodes 3
```

**출력 예시**:
```
✅ Isaac Sim 초기화 완료

📂 모델 로딩: logs/rl_training/final_model/roarm_ppo_final.zip
✅ 모델 로드 완료

🎥 비디오 녹화 설정...
  출력 경로: logs/videos/roarm_20251019_143000.mp4
  FPS: 30
  에피소드: 3

🎬 녹화 시작...

📊 Episode 1/3
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
에피소드 완료:
  보상: -2456.32
  길이: 601 steps

...

✅ 녹화 완료!
  총 프레임: 1803
  영상 길이: 60.1초
  저장 위치: logs/videos/
```

**비디오 재생**:
```bash
# 저장된 비디오 확인
ls -lh logs/videos/

# VLC로 재생
vlc logs/videos/roarm_20251019_143000.mp4

# 또는 기본 플레이어
xdg-open logs/videos/roarm_20251019_143000.mp4
```

---

### 방법 3: 스크린샷 캡처

특정 순간을 **이미지**로 저장합니다.

```bash
# GUI 모드에서 Isaac Sim 실행
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py

# Isaac Sim에서:
# 1. Window → Viewport → Capture Screenshot
# 2. 또는 단축키: Ctrl + Shift + S
```

---

## 📊 시각화 비교

| 방법 | 장점 | 단점 | 용도 |
|------|------|------|------|
| **GUI 실시간** | 즉시 확인 가능 | 녹화 안 됨 | 빠른 디버깅 |
| **비디오 녹화** | 공유 가능, 반복 재생 | 시간 소요 | 발표, 문서화 |
| **스크린샷** | 경량, 빠름 | 동작 확인 불가 | 보고서 |

---

## 🎬 예시: 전체 학습 과정 시각화

학습 진행에 따른 성능 개선을 확인하려면:

```bash
# 1. 초기 모델 (5K steps)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training/checkpoints/roarm_ppo_5000_steps.zip \
  --episodes 3

# 2. 중간 모델 (25K steps)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training/checkpoints/roarm_ppo_25000_steps.zip \
  --episodes 3

# 3. 최종 모델 (50K steps)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training/final_model/roarm_ppo_final.zip \
  --episodes 3
```

---

## 🔍 시각화로 확인할 사항

### 1. 로봇 움직임
- ✅ 부드러운 움직임
- ✅ 물체를 향해 이동
- ❌ 떨림, 불안정한 행동

### 2. 그리퍼 동작
- ✅ 물체 근처에서 닫힘
- ✅ 물체를 잡음
- ❌ 계속 열려 있음

### 3. Task 완수
- ✅ 물체를 목표 위치로 이동
- ✅ 빠른 완료 (< 300 steps)
- ❌ 시간 초과 (600 steps)

---

## 🐛 트러블슈팅

### Q: GUI가 열리지 않음
```bash
# 디스플레이 확인
echo $DISPLAY

# X11 권한 확인
xhost +local:

# 다시 실행
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py
```

### Q: 비디오 녹화가 안 됨
- **원인**: Replicator 설정 문제
- **해결**: GUI 모드로 먼저 확인 후 녹화

### Q: 화면이 너무 빠름
```bash
# 렌더링 딜레이 증가
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --render-delay 0.2  # 0.2초 딜레이
```

---

## 💡 팁

### 더 나은 시각화를 위해

1. **카메라 각도 조정**:
   - GUI에서 마우스 드래그로 시점 변경
   - 로봇과 물체가 잘 보이도록

2. **느린 재생**:
   - `--render-delay 0.1` 또는 `0.2` 사용
   - 세밀한 동작 분석 가능

3. **여러 에피소드 비교**:
   - 성공한 에피소드 vs 실패한 에피소드
   - 차이점 분석

---

## 📁 파일 구조

```
logs/
├── rl_training/
│   ├── final_model/
│   │   └── roarm_ppo_final.zip          # 최종 모델
│   └── checkpoints/
│       ├── roarm_ppo_5000_steps.zip     # 체크포인트
│       ├── roarm_ppo_10000_steps.zip
│       └── ...
└── videos/                               # 녹화된 비디오
    ├── roarm_20251019_143000.mp4
    └── ...
```

---

## 🎯 다음 단계

시각화로 문제를 발견했다면:

1. **로봇이 물체에 닿지 못함**:
   - 보상 함수 조정 (거리 보상 증가)
   - Episode 길이 연장

2. **그리퍼가 작동 안 함**:
   - 그리퍼 액션 범위 확인
   - 그리퍼 보상 추가

3. **움직임이 불안정**:
   - Learning rate 감소
   - 더 긴 학습

---

## 📖 참고

- `EASY_MODE_TRAINING_GUIDE.md` - 학습 가이드
- `scripts/test_trained_model.py` - GUI 테스트 스크립트
- `scripts/record_model_video.py` - 비디오 녹화 스크립트
