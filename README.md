# RoArm-M3 Isaac Sim RL Training

RoArm-M3 로봇팔의 Pick and Place 작업을 위한 강화학습 환경 (Isaac Sim 5.0 + PPO)

## 📊 현재 상태

**Training Phase**: Phase 0 (Easy Mode)  
**Total Steps**: 50K  
**달성 마일스톤**: REACH (+5.0) ✅

### 학습 성과
- ep_rew_mean: -6.01 → -4.21 (30% 개선)
- Dense Reward(실패) → Sparse Reward(안정) → Shaped-Sparse(현재)
- REACH 마일스톤 첫 달성!

## 🎯 보상 시스템 (Shaped-Sparse)

```
근접 (+5)   : EE→큐브 5cm          ✅ 달성!
그립 (+10)  : 유효 그립 3프레임    ❌ 학습 중
리프트 (+15): 큐브 5cm 상승        ❌ 학습 중  
목표 (+20)  : 큐브→타겟 8cm        ❌ 학습 중
Success (+100): 타겟 5cm 5프레임   ❌ 학습 중
Time (-0.01): 효율성 패널티
```

## 🏗️ 프로젝트 구조

```
roarm_isaac_clean/
├── assets/roarm_m3/          # RoArm-M3 URDF, meshes
├── envs/                     # RL 환경 (Shaped-Sparse + Curriculum)
├── scripts/                  # 학습/테스트 스크립트
├── logs/                     # 학습 로그 및 체크포인트
│   └── rl_training_curriculum/
│       └── checkpoints/      # 9개 체크포인트 (5K~50K)
├── docs/                     # 문서
│   └── TRAINING_LOG_20251019.md
└── README.md

```

## 🚀 빠른 시작

### 1. 환경 설정
```bash
# Isaac Sim 5.0 설치 필요
# conda 환경 설정 (선택)
```

### 2. 학습 실행
```bash
cd ~/roarm_isaac_clean
~/isaacsim/python.sh scripts/train_dense_reward.py
```

### 3. GUI 테스트
```bash
~/isaacsim/python.sh scripts/test_trained_model.py \
  --model logs/rl_training_curriculum/checkpoints/roarm_ppo_curriculum_50000_steps.zip \
  --episodes 3
```

## 📈 학습 이력

### Dense Reward (실패 - 폐기)
- 개선 보상 누적으로 정책 붕괴 (ep_rew: +916)
- EV 0.00006, VL 855

### Sparse Reward (안정화)
- 100K steps 완료
- EV 4,717배, VL 6,287배 개선
- 문제: Success 신호 부족

### Shaped-Sparse + Curriculum (현재)
- 50K steps (Phase 0 Easy Mode)
- 첫 마일스톤: REACH (+5) 달성!
- 다음: 200K steps 장기 학습 예정

## 🔧 기술 스택

- **Isaac Sim**: 5.0
- **RL Library**: Stable-Baselines3 (PPO)
- **Python**: 3.11
- **Framework**: omni.isaac.lab (deprecated APIs 사용)

## 📝 주요 파일

- `envs/roarm_pick_place_env.py`: Shaped-Sparse 보상 + Curriculum 환경
- `scripts/train_dense_reward.py`: PPO 학습 스크립트
- `scripts/early_warning_callback.py`: EV/VL 자동 감지 콜백
- `scripts/test_trained_model.py`: GUI 테스트 스크립트

## 🚧 개선 필요 사항

### URDF
1. 그리퍼 형태: 평행 그리퍼 재설계 필요
2. Link 크기/비율: 실제 RoArm M3 사양 반영 필요

### 학습
1. 추가 마일스톤 달성 (GRIP, LIFT, GOAL, SUCCESS)
2. Phase 1 Normal Mode 전환 (성공률 ≥60%)

## 📚 참고 문서

- [Training Log](docs/TRAINING_LOG_20251019.md)
- [RL Best Practices](resources/rl_best_practices.md)

## 👥 기여자

- limjh6991-spec

---

**Last Updated**: 2025-10-19  
**Status**: Phase 0 학습 완료, 장기 학습 준비 중
