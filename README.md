# RoArm-M3 Isaac Sim RL Training

RoArm-M3 로봇팔의 Pick and Place 작업을 위한 강화학습 환경 (Isaac Sim 5.0 + PPO)

## 📊 현재 상태

**Training Phase**: Phase 0 (Easy Mode) - **URDF 수정 필요** 🚨  
**Total Steps**: 100K  
**달성 마일스톤**: REACH (+5.0) 12회 ✅

### 학습 성과
- ep_rew_mean: +916 → -6.01 → -5.66 (안정화)
- EV: 0.00006 → 0.283 (4,717배 개선)
- Dense Reward(실패) → Sparse Reward(안정) → Shaped-Sparse(현재)
- REACH 마일스톤 달성! ✅
- **GRIP 마일스톤 미달성** ❌ → URDF 그리퍼 조인트 문제

### ⚠️ 현재 이슈
**그리퍼 조인트 미작동**
- 로그: "Joints (8): ['joint_1', 'joint_2', 'joint_3']..." (3개만 출력)
- 원인: 그리퍼 조인트가 Fixed 또는 잘못 정의됨
- 해결: URDF 수정 필요 (내일 최우선 작업)
- 참고: `docs/ISSUE_ANALYSIS_20251019.md`

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
- 100K steps (Phase 0 Easy Mode)
- REACH 마일스톤: 12회 달성! ✅
- **이슈 발견**: GRIP 미달성 (URDF 그리퍼 문제)
- **다음**: URDF 수정 후 50K 재학습

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

## 🚧 개선 필요 사항 (우선순위)

### 1. URDF 수정 (최우선!) 🚨
- **그리퍼 조인트**: Fixed → Prismatic 변경
- **Joint limits**: 0~25mm 추가
- **그리퍼 형태**: 평행 그리퍼 재설계
- **Link 크기**: 실제 RoArm M3 사양 반영
- **검증**: Isaac Sim 임포트 테스트

### 2. 학습 재개 (URDF 수정 후)
- 50K steps 테스트 학습
- GRIP 마일스톤 1회 이상 목표
- 성공 시 1M steps 장기 학습

### 3. Phase 1 전환 (최종)
- Normal Mode (큐브 25~35cm, 타겟 25~35cm)
- 성공률 ≥60% 목표

## 📚 참고 문서

- [내일 할 일](docs/TODO_20251020.md) ⭐
- [이슈 분석](docs/ISSUE_ANALYSIS_20251019.md)
- [Training Log](docs/TRAINING_LOG_20251019.md)

## 👥 기여자

- limjh6991-spec

---

**Last Updated**: 2025-10-19  
**Status**: Phase 0 학습 완료, 장기 학습 준비 중
