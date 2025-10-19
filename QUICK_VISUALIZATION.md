# 🎥 Isaac Sim 영상 확인 완벽 가이드

**질문**: "isaacsim에서 영상으로 확인은 어려워?"  
**답변**: **전혀 어렵지 않습니다!** 3가지 방법이 있습니다.

---

## ✅ 방법 1: GUI로 실시간 확인 (추천) ⭐

가장 **간단하고 빠른** 방법입니다. Isaac Sim 창이 열리면서 로봇 움직임을 실시간으로 볼 수 있습니다.

### 사용법

```bash
# 최종 모델 테스트 (5 에피소드)
cd ~/roarm_isaac_clean
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --episodes 5
```

### 실행 예시

```
✅ Isaac Sim GUI 모드 초기화 완료

📂 모델 로딩: logs/rl_training/final_model/roarm_ppo_final.zip
✅ 모델 로드 완료

🎬 5개 에피소드 테스트 시작...

📊 Episode 1/5
━━━━━━━━━━━━━━━━━━━━━━━━━━
  ⏱️ Timeout (Max steps: 600)

❌ 에피소드 완료:
  보상: -2658.16
  길이: 601 steps

...

📊 테스트 결과 요약
━━━━━━━━━━━━━━━━━━━━━━━━━━
총 에피소드: 5
성공: 0/5 (0.0%)

보상 통계:
  평균: -2786.45
  최고: -2401.23
  최저: -3012.56
```

**Isaac Sim 창에서 로봇 움직임을 실시간으로 볼 수 있습니다!**

### 추가 옵션

```bash
# 특정 체크포인트 테스트
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training/checkpoints/roarm_ppo_40000_steps.zip \
  --episodes 3

# 느리게 보기 (동작 자세히 관찰)
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --render-delay 0.2  # 0.2초 딜레이

# 빠르게 보기
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --render-delay 0.01  # 0.01초 딜레이
```

---

## 🎬 방법 2: 비디오 녹화 (공유용)

학습된 모델의 실행을 **MP4 비디오**로 저장합니다. 발표나 문서화에 유용합니다.

### 사용법

```bash
# 기본 녹화 (3 에피소드, 30 FPS)
cd ~/roarm_isaac_clean
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/record_model_video.py

# 커스텀 설정
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/record_model_video.py \
  --episodes 5 \
  --fps 60 \
  --output logs/videos/my_demo.mp4
```

### 비디오 재생

```bash
# 저장된 비디오 확인
ls -lh logs/videos/

# VLC로 재생
vlc logs/videos/roarm_20251019_143000.mp4

# 또는 기본 플레이어
xdg-open logs/videos/roarm_20251019_143000.mp4
```

---

## 📸 방법 3: 스크린샷

특정 순간을 **이미지**로 저장합니다.

```bash
# GUI 모드에서 Isaac Sim 실행
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py

# Isaac Sim에서 스크린샷:
# Window → Viewport → Capture Screenshot
# 또는 단축키: Ctrl + Shift + S
```

---

## 💡 언제 어떤 방법을 사용할까?

| 상황 | 추천 방법 | 이유 |
|------|-----------|------|
| 빠르게 확인 | GUI 실시간 | 즉시 확인 가능 |
| 문제 분석 | GUI 실시간 (느리게) | 동작 자세히 관찰 |
| 발표/보고서 | 비디오 녹화 | 공유 및 반복 재생 가능 |
| 문서 삽입 | 스크린샷 | 경량, 빠름 |

---

## 🎯 실제 사용 예시

### 학습 진행 상황 확인

```bash
# 1. 초기 모델 (5K steps) - 어떻게 움직이나?
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training/checkpoints/roarm_ppo_5000_steps.zip \
  --episodes 2

# 2. 중간 모델 (25K steps) - 개선되었나?
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training/checkpoints/roarm_ppo_25000_steps.zip \
  --episodes 2

# 3. 최종 모델 (50K steps) - 최종 성능은?
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --episodes 5
```

### 느린 재생으로 문제 분석

```bash
# 로봇 동작을 천천히 관찰
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --episodes 1 \
  --render-delay 0.2

# 이제 GUI에서:
# - 로봇이 물체를 향해 가는가?
# - 그리퍼가 열고 닫히는가?
# - 어느 부분에서 실패하는가?
```

---

## 🐛 트러블슈팅

### Q: GUI가 열리지 않음

```bash
# 디스플레이 확인
echo $DISPLAY

# X11 권한 부여
xhost +local:

# 다시 실행
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py
```

### Q: "headless" 오류

```bash
# test_trained_model.py는 headless=False로 설정되어 있음
# 확인:
grep "headless" scripts/test_trained_model.py

# 출력:
# "headless": False,  # GUI 활성화
```

### Q: 화면이 너무 빠름

```bash
# 렌더링 딜레이 증가
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --render-delay 0.5  # 0.5초 딜레이 (매우 느림)
```

### Q: 모델 파일 없음

```bash
# 사용 가능한 모델 확인
ls -lh logs/rl_training/final_model/
ls -lh logs/rl_training/checkpoints/

# 최신 체크포인트 사용
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training/checkpoints/roarm_ppo_50000_steps.zip
```

---

## 📊 시각화로 확인할 사항

### 1. 로봇 움직임
- ✅ 부드러운 움직임
- ✅ 물체를 향해 이동
- ❌ 떨림, 불안정한 행동
- ❌ 제자리에서 움직이지 않음

### 2. 그리퍼 동작
- ✅ 물체 근처에서 닫힘
- ✅ 물체를 잡음
- ❌ 계속 열려 있음
- ❌ 물체를 놓침

### 3. Task 완수
- ✅ 물체를 목표 위치로 이동
- ✅ 빠른 완료 (< 300 steps)
- ❌ 시간 초과 (600 steps)
- ❌ 물체에 도달하지 못함

---

## 🎓 학습 효과 측정

### GUI로 성공률 확인

```bash
# 10 에피소드 테스트
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
  --episodes 10

# 결과 예시:
# 총 에피소드: 10
# 성공: 2/10 (20.0%)  ← 성공률
# 평균 보상: -2456.32
```

### 체크포인트 비교

```bash
# 각 체크포인트의 성능 비교
for checkpoint in logs/rl_training/checkpoints/*.zip; do
  echo "Testing: $checkpoint"
  PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py \
    --model "$checkpoint" \
    --episodes 5 | grep "평균"
done
```

---

## ✅ 요약

**질문**: "isaacsim에서 영상으로 확인은 어려워?"

**답변**: **전혀 어렵지 않습니다!**

```bash
# 단 한 줄로 GUI 확인:
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py

# Isaac Sim 창이 열리고 로봇 움직임을 실시간으로 볼 수 있습니다!
```

### 핵심 명령어

| 목적 | 명령어 |
|------|--------|
| **실시간 확인** | `~/isaacsim/python.sh scripts/test_trained_model.py` |
| **느리게 보기** | `~/isaacsim/python.sh scripts/test_trained_model.py --render-delay 0.2` |
| **비디오 녹화** | `~/isaacsim/python.sh scripts/record_model_video.py` |
| **특정 모델** | `~/isaacsim/python.sh scripts/test_trained_model.py --model [경로]` |

---

## 📖 더 자세한 정보

- `MODEL_VISUALIZATION_GUIDE.md` - 전체 시각화 가이드
- `EASY_MODE_TRAINING_GUIDE.md` - 학습 가이드
- `scripts/test_trained_model.py` - GUI 테스트 스크립트
- `scripts/record_model_video.py` - 비디오 녹화 스크립트
