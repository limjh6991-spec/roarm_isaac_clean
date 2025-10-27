# 학습 완료 후 동영상 생성 가이드

## 📋 학습 진행 상황

현재 학습이 진행 중입니다:
- **목표**: 100K timesteps
- **현재**: ~30K timesteps (30%)
- **예상 완료 시간**: 약 3-5분 후

## 🎬 동영상 생성 워크플로우

### 1단계: 학습 완료 확인

학습이 완료되면 다음 파일들이 생성됩니다:
```
logs/rl_training_curriculum/
├── final_model/
│   ├── roarm_ppo_dense_final.zip      # 최종 모델
│   └── vecnormalize.pkl                # 정규화 통계
├── checkpoints/
│   ├── roarm_ppo_curriculum_5000_steps.zip
│   ├── roarm_ppo_curriculum_10000_steps.zip
│   └── ...
└── monitor.monitor.csv                 # 에피소드 로그
```

### 2단계: 학습 결과 시각화

```bash
# 학습 그래프 생성
python scripts/rl/plot_training_progress.py
```

출력:
- `logs/rl_training_curriculum/training_progress.png`
- 보상, 에피소드 길이, 학습 시간 추이

### 3단계: 학습된 모델 테스트 및 동영상 녹화

```bash
# GUI 모드로 테스트 (5 에피소드)
~/isaacsim/python.sh scripts/rl/test_and_record_video.py \
  --model logs/rl_training_curriculum/final_model/roarm_ppo_dense_final.zip \
  --vecnorm logs/rl_training_curriculum/final_model/vecnormalize.pkl \
  --episodes 5 \
  --output logs/demo_video.mp4
```

**주의**: Isaac Sim의 Movie Capture 기능 사용 방법:
1. GUI에서 `Window > Movie Capture` 열기
2. "Output Directory" 설정: `logs/`
3. "File Name" 설정: `demo_video`
4. "Format" 선택: `MP4`
5. "Start Recording" 클릭
6. 스크립트 실행
7. 완료 후 "Stop Recording" 클릭

### 4단계: 대안 - 스크린샷 시퀀스로 동영상 생성

Isaac Sim Movie Capture가 안 될 경우:

```bash
# 스크린샷 캡처 (자동)
~/isaacsim/python.sh scripts/rl/capture_screenshots_during_test.py \
  --model logs/rl_training_curriculum/final_model/roarm_ppo_dense_final.zip \
  --vecnorm logs/rl_training_curriculum/final_model/vecnormalize.pkl \
  --episodes 3 \
  --output-dir logs/screenshots
```

그 다음 ffmpeg로 동영상 생성:
```bash
ffmpeg -framerate 30 -pattern_type glob -i 'logs/screenshots/*.png' \
  -c:v libx264 -pix_fmt yuv420p -crf 20 \
  logs/demo_video.mp4
```

---

## 📊 예상 결과

### 학습 메트릭 (100K timesteps 기준)
- **REACH 달성률**: 70-80%
- **GRIP 달성률**: 10-30%
- **LIFT 달성률**: 5-15%
- **Success 달성률**: 0-10%

### 동영상 내용
- 5 에피소드 시연
- 각 에피소드 ~600 steps (10초)
- 총 길이: ~50초 (1분)
- 해상도: 1920x1080
- FPS: 30

---

## 🔧 트러블슈팅

### 문제 1: Movie Capture 확장 없음
**해결**: 스크린샷 시퀀스 방법 사용 (위 4단계)

### 문제 2: 모델 파일 없음
**확인**:
```bash
ls -lh logs/rl_training_curriculum/final_model/
ls -lh logs/rl_training_curriculum/checkpoints/
```

**대안**: 체크포인트 사용
```bash
~/isaacsim/python.sh scripts/rl/test_and_record_video.py \
  --model logs/rl_training_curriculum/checkpoints/roarm_ppo_curriculum_100000_steps.zip
```

### 문제 3: VecNormalize 파일 없음
**증상**: 관측 값이 이상함
**해결**: 반드시 학습 시와 동일한 vecnormalize.pkl 사용!

---

## 📝 자동화 스크립트 (학습 완료 후 실행)

```bash
#!/bin/bash
# complete_workflow.sh

echo "🎉 학습 완료 후 워크플로우 시작"

# 1. 학습 그래프 생성
echo "📊 Step 1: 학습 그래프 생성"
python scripts/rl/plot_training_progress.py

# 2. 스크린샷 캡처
echo "📸 Step 2: 스크린샷 캡처 (3 에피소드)"
~/isaacsim/python.sh scripts/rl/capture_screenshots_during_test.py \
  --model logs/rl_training_curriculum/final_model/roarm_ppo_dense_final.zip \
  --vecnorm logs/rl_training_curriculum/final_model/vecnormalize.pkl \
  --episodes 3 \
  --output-dir logs/screenshots

# 3. 동영상 생성
echo "🎬 Step 3: 동영상 생성"
ffmpeg -y -framerate 30 -pattern_type glob -i 'logs/screenshots/*.png' \
  -c:v libx264 -pix_fmt yuv420p -crf 20 \
  logs/demo_video.mp4

echo "✅ 완료! 동영상: logs/demo_video.mp4"
```

실행:
```bash
chmod +x complete_workflow.sh
./complete_workflow.sh
```

---

## 🎯 최종 확인 사항

학습 완료 후 다음을 확인하세요:

- [ ] 학습 로그 파일 존재
- [ ] 최종 모델 파일 존재
- [ ] VecNormalize 파일 존재
- [ ] REACH 달성률 60% 이상
- [ ] 체크포인트 파일들 저장됨

모든 체크가 완료되면 동영상 생성을 진행하세요!
