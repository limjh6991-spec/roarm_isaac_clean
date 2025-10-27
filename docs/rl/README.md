# RoArm M3 강화학습 문서

**목적**: Pick & Place 작업을 위한 강화학습 종합 가이드

---

## 📚 문서 목록

### 1. [RL_TRAINING_PLAN_V2.md](./RL_TRAINING_PLAN_V2.md) ⭐ **시작점**
**목적**: 전체 학습 로드맵 및 Phase별 계획

**내용**:
- Phase 0: 사전 진단 (필수!)
- Phase 1: 초기 학습 (0~50K steps)
- Phase 2: GRIP 디버깅 (50K~100K steps)
- Phase 3: 안정화 (100K~500K steps)
- Phase 4: 최종 달성 (500K~1M steps)
- 문제 해결 플레이북
- 타임라인 (총 ~21시간)

**즉시 실행**:
```bash
# 1. 사전 진단
cd ~/roarm_isaac_clean/scripts/rl
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh diagnose_env.py --full

# 2. Phase 1 시작
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh train_dense_reward.py --timesteps 50000
```

---

### 2. [DIAGNOSTIC_CHECKLIST.md](./DIAGNOSTIC_CHECKLIST.md) ⚠️ **학습 전 필수**
**목적**: 환경 검증 7단계 체크리스트

**진단 항목**:
1. ✅ Observation Check - 큐브 정보가 관측에 포함되는가?
2. ✅ Coordinate Frame Alignment - 좌표계가 일치하는가?
3. ✅ Camera Validation - (Vision 사용 시) 카메라 작동하는가?
4. ✅ Physics & Collision - 물리 충돌이 올바른가?
5. ✅ Reward Signal - 보상이 학습을 유도하는가?
6. ✅ Task Integration - 태스크가 올바르게 등록되었는가?
7. ✅ Runtime Diagnostics - 실시간 데이터가 정상인가?

**사용법**:
```bash
# 전체 진단
python diagnose_env.py --full

# 특정 항목
python diagnose_env.py --check observation
python diagnose_env.py --check reward --verbose
```

---

### 3. [REWARD_DESIGN_GUIDE.md](./REWARD_DESIGN_GUIDE.md) 🎯
**목적**: 효과적인 보상 함수 설계

**핵심 내용**:
- Dense vs Sparse 보상
- 보상 스케일 (권장: -10 ~ +100)
- 1회성 보상 (중복 방지)
- 다단계 보상 (REACH → GRIP → LIFT → GOAL → SUCCESS)
- RoArm M3 보상 구조 상세 코드
- GRIP 미달성 문제 해결 (조건 완화, Contact force)
- 디버깅 팁

**현재 문제**: GRIP 조건 너무 엄격
```python
# ❌ 현재
grasp_valid = (ee_to_cube < 0.08) & (gripper < 0.02) & (height > 0.03)

# ✅ 권장
grasp_valid = (ee_to_cube < 0.10) & (gripper < 0.03) & (height > 0.02)
```

---

### 4. [PPO_HYPERPARAMETERS_CHEATSHEET.md](./PPO_HYPERPARAMETERS_CHEATSHEET.md) ⚙️
**목적**: PPO 파라미터 빠른 참조

**기본 설정** (로봇 조작):
```python
learning_rate = 3e-4
n_steps = 2048
batch_size = 64
n_epochs = 10
gamma = 0.99
clip_range = 0.2
ent_coef = 0.01
```

**문제별 해결책**:
- 학습 불안정 → `clip_range=0.1`, `learning_rate=1e-4`
- 학습 느림 → `n_steps=4096`, `ent_coef=0.05`
- Over-fitting → `ent_coef=0.05`, Domain Randomization

**파라미터 범위 표** 포함

---

### 5. [SUCCESS_CASES.md](./SUCCESS_CASES.md) 🏆
**목적**: 실제 성공한 프로젝트 분석

**케이스 스터디**:
1. **Franka Panda Cube Stack** (IsaacGym)
   - 학습: 5M steps, ~12시간
   - 성공률: 85%
   - 보상: Shaped-Sparse 하이브리드

2. **UR5 Pick and Place** (ROS)
   - Curriculum Learning 3단계
   - Sim-to-Real 성공률: 78%

3. **Shadow Hand** (OpenAI)
   - Asymmetric Actor-Critic
   - ADR (Automatic Domain Randomization)

4. **Berkeley Depth Camera**
   - Vision-based Reward

**RoArm M3 적용 방안** 포함

---

### 6. [RL_RESOURCES_INDEX.md](./RL_RESOURCES_INDEX.md) 📖
**목적**: 종합 참고 자료

**섹션**:
1. 핵심 논문 (PPO, HER, 로봇 조작)
2. Isaac Sim RL 튜토리얼
3. 로봇 조작 성공 사례
4. PPO 알고리즘 가이드
5. 보상 함수 설계
6. 하이퍼파라미터 튜닝
7. 디버깅 체크리스트
8. 실전 팁
9. 추가 참고 자료 (GitHub, 블로그, 비디오)
10. 다음 단계

**주요 링크**:
- IsaacGymEnvs: https://github.com/NVIDIA-Omniverse/IsaacGymEnvs
- Stable-Baselines3: https://github.com/DLR-RM/stable-baselines3
- OpenAI Spinning Up: https://spinningup.openai.com/

---

## 🚀 빠른 시작 가이드

### Step 1: 사전 진단 (필수!)
```bash
cd ~/roarm_isaac_clean/scripts/rl
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh diagnose_env.py --full
```

**예상 결과**:
```
✅ Observation Check: PASSED
✅ Coordinate Frame: PASSED
✅ Physics & Collision: PASSED
⚠️ Reward Signal: PASSED (but GRIP condition strict)
✅ Task Integration: PASSED
✅ Runtime Diagnostics: PASSED

=== RECOMMENDATIONS ===
✅ All checks passed! Ready to train.
   Run: PYTHONUNBUFFERED=1 ~/isaacsim/python.sh train_dense_reward.py --timesteps 50000
```

### Step 2: Phase 1 학습 (50K steps)
```bash
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh train_dense_reward.py --timesteps 50000 2>&1 | tee /tmp/training_phase1.log
```

**목표**:
- REACH 마일스톤 10회 이상
- ep_rew_mean > -5

### Step 3: TensorBoard 모니터링 (별도 터미널)
```bash
tensorboard --logdir=./tensorboard_logs --port=6006
```

**주요 지표**:
- `ep_rew_mean`: 평균 보상 (↑ 목표)
- `ep_len_mean`: 평균 에피소드 길이
- `value_loss`: Critic 손실
- `entropy`: 탐험 정도

### Step 4: Phase 1 평가
```bash
grep "REACH\|GRIP\|LIFT" /tmp/training_phase1.log | tail -20
```

**통과 조건**:
- ✅ REACH 10회 이상 → Phase 2 진행
- ❌ REACH 5회 미만 → 진단 재확인

---

## 📊 현재 상태 (2025-10-20)

### ✅ 완료
- URDF v1.1 개선 (그리퍼 5배 향상, 마찰 2배 증가)
- GUI 테스트 성공 (`test_trained_model.py`)
- RL 문서 5개 작성
- 진단 스크립트 생성 (`diagnose_env.py`)

### 🔄 진행 중
- **다음 작업**: Phase 0 사전 진단 실행

### ⏳ 예정
- Phase 1: 50K 학습 (REACH 목표)
- Phase 2: GRIP 디버깅 (조건 완화)
- Phase 3~4: 안정화 및 최종 달성 (1M steps)

---

## 🐛 알려진 문제

### 1. GRIP 마일스톤 미달성 (100K steps, 0회)
**원인**: grasp_valid 조건 너무 엄격
```python
# 현재
grasp_valid = (
    ee_to_cube_dist < 0.08 and
    gripper_width < 0.02 and
    cube_pos[2] > 0.03
)
```

**해결 계획**: Phase 2에서 조건 완화
```python
# 개선
grasp_valid = (
    ee_to_cube_dist < 0.10 and      # 8cm → 10cm
    gripper_width < 0.03 and         # 2cm → 3cm
    cube_pos[2] > 0.02               # 3cm → 2cm
)
```

### 2. URDF 그리퍼 개선 완료 (2025-10-20)
✅ **해결됨**:
- Prismatic joint 확인
- 접촉 면적 5배 증가
- 마찰 계수 2배 증가 (0.3 → 0.6)
- 그리퍼 힘 60% 증가 (5 → 8 N·m)

---

## 📝 문서 작성 이력

| 날짜 | 문서 | 내용 |
|-----|------|------|
| 2025-10-20 | RL_TRAINING_PLAN_V2.md | 전문가 진단 통합, Phase별 계획 |
| 2025-10-20 | DIAGNOSTIC_CHECKLIST.md | 7단계 진단 체크리스트 |
| 2025-10-20 | REWARD_DESIGN_GUIDE.md | 보상 함수 설계 가이드 |
| 2025-10-20 | PPO_HYPERPARAMETERS_CHEATSHEET.md | PPO 파라미터 치트시트 |
| 2025-10-20 | SUCCESS_CASES.md | 성공 사례 4개 분석 |
| 2025-10-20 | RL_RESOURCES_INDEX.md | 종합 참고 자료 |
| 2025-10-20 | diagnose_env.py | 자동 진단 스크립트 |

---

## 🔗 관련 디렉토리

- **학습 스크립트**: `~/roarm_isaac_clean/scripts/rl/`
- **URDF 파일**: `~/roarm_isaac_clean/assets/roarm_m3/urdf/`
- **모델 저장**: `~/roarm_isaac_clean/scripts/rl/models/`
- **TensorBoard**: `~/roarm_isaac_clean/scripts/rl/tensorboard_logs/`

---

## 💡 팁

### 학습 전
1. ✅ 사전 진단 반드시 실행
2. ✅ URDF v1.1 확인 (그리퍼 개선)
3. ✅ TensorBoard 설정
4. ✅ 백업 디렉토리 생성

### 학습 중
1. 📈 TensorBoard로 실시간 모니터링
2. 💾 50K마다 모델 저장
3. 📊 마일스톤 달성 횟수 확인
4. 🐛 문제 발생 시 진단 재실행

### 학습 후
1. 📹 비디오 녹화 (`--save-video`)
2. 📊 최종 평가 (100 에피소드)
3. 📝 README 업데이트
4. 🎉 성공률 60% 달성 축하!

---

**작성자**: GitHub Copilot  
**최종 업데이트**: 2025-10-20  
**다음 업데이트**: Phase 1 학습 결과 반영

---

## 📧 문의

문제가 발생하거나 질문이 있으면:
1. DIAGNOSTIC_CHECKLIST.md 참조
2. REWARD_DESIGN_GUIDE.md 문제 해결 섹션
3. GitHub Issue 생성
