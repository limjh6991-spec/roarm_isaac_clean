# 강화학습 리소스 인덱스
> 수집 날짜: 2025-10-19  
> 프로젝트: RoArm-M3 Isaac Sim RL

## 📚 로컬 리소스

### 1. 베스트 프랙티스
**파일**: `rl_best_practices.md`  
**내용**:
- 보상 설계 (Reward Shaping)
- PPO 하이퍼파라미터 가이드
- 학습 디버깅 방법
- 환경 설계 원칙
- 일반적인 실수와 해결법

**주요 발견사항**:
- ✅ RoArm-M3 보상 스케일 문제 진단
- ✅ 보상 범위: -50 ~ +50 권장
- ✅ Dense Reward 구성: 3-5개 요소

---

### 2. PPO 알고리즘 완전 가이드
**파일**: `ppo_algorithm_guide.md`  
**내용**:
- PPO 작동 원리 상세
- 수식 및 코드 예제
- 다른 알고리즘과 비교 (SAC, TD3, A2C)
- 하이퍼파라미터 상세 설명
- 실전 팁 및 고급 기법

**핵심 하이퍼파라미터**:
```python
learning_rate: 3e-4 (1e-5 ~ 1e-3)
n_steps: 2048 (512 ~ 8192)
batch_size: 64 (32 ~ 512)
n_epochs: 10 (3 ~ 30)
gamma: 0.99 (0.9 ~ 0.9999)
clip_range: 0.2 (0.1 ~ 0.3)
```

---

### 3. 디버깅 체크리스트
**파일**: `rl_debugging_checklist.md`  
**내용**:
- 문제 증상별 해결 가이드
- 5가지 주요 문제 유형
- 디버깅 도구 사용법
- 학습 전 체크리스트
- 고급 디버깅 기법

**문제 유형**:
1. 학습이 전혀 안 됨 (보상 flat)
2. 학습이 불안정 (보상 진동)
3. 수렴이 느림
4. NaN/Inf 발생
5. 과적합

---

## 🌐 온라인 리소스

### 공식 문서
1. **Stable-Baselines3**: https://stable-baselines3.readthedocs.io/
   - RL Tips: `/guide/rl_tips.html`
   - PPO Docs: `/modules/ppo.html`
   - Examples: `/guide/examples.html`

2. **OpenAI Spinning Up**: https://spinningup.openai.com/
   - Key Concepts: `/spinningup/rl_intro.html`
   - PPO Algorithm: `/algorithms/ppo.html`
   - Exercises: `/spinningup/exercises.html`

3. **Ray RLlib**: https://docs.ray.io/en/latest/rllib/
   - Algorithms: `/rllib-algorithms.html`
   - Environments: `/rllib-env.html`

4. **HuggingFace Deep RL Course**: https://huggingface.co/deep-rl-course
   - Unit 1: Introduction to RL
   - Unit 2: Q-Learning
   - Unit 8: PPO

### 핵심 논문
1. **PPO (2017)**: [Proximal Policy Optimization](https://arxiv.org/abs/1707.06347)
   - 저자: Schulman et al.
   - 핵심: Clipped surrogate objective

2. **GAE (2016)**: [Generalized Advantage Estimation](https://arxiv.org/abs/1506.02438)
   - 저자: Schulman et al.
   - 핵심: Bias-variance tradeoff

3. **TRPO (2015)**: [Trust Region Policy Optimization](https://arxiv.org/abs/1502.05477)
   - 저자: Schulman et al.
   - 핵심: Constrained optimization

4. **Reward Shaping (1999)**: [Policy Invariance Under Reward Transformations](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/NgHaradaRussell-shaping-ICML1999.pdf)
   - 저자: Ng, Harada, Russell
   - 핵심: Potential-based shaping

### 코드 저장소
1. **Stable-Baselines3**: https://github.com/DLR-RM/stable-baselines3
2. **RL Baselines3 Zoo**: https://github.com/DLR-RM/rl-baselines3-zoo
3. **CleanRL**: https://github.com/vwxyzjn/cleanrl
4. **OpenAI Spinning Up**: https://github.com/openai/spinningup

### 튜토리얼
1. **PyTorch RL Tutorial**: https://pytorch.org/tutorials/intermediate/reinforcement_q_learning.html
2. **HuggingFace Lunar Lander**: https://huggingface.co/blog/deep-rl-intro
3. **Stable-Baselines3 Getting Started**: https://stable-baselines3.readthedocs.io/en/master/guide/quickstart.html

---

## 🎯 RoArm-M3 프로젝트 적용

### 현재 상황
- **환경**: RoArm-M3 Pick and Place (Isaac Sim 5.0.0)
- **알고리즘**: PPO (Stable-Baselines3 2.7.0)
- **문제**: 보상 스케일 과도 (수천 단위)

### 수집한 지식 적용
1. **보상 함수 재설계** (`rl_best_practices.md` 참고)
   - 현재: reach 5.0, move 10.0 (과도)
   - 권장: reach 0.5~1.0, move 1.0~2.0

2. **하이퍼파라미터 조정** (`ppo_algorithm_guide.md` 참고)
   - learning_rate: 3e-4 → 1e-4 (안정성)
   - clip_range: 0.2 → 0.1 (안정성)
   - ent_coef: 0.0 → 0.005 (exploration)

3. **디버깅 체계화** (`rl_debugging_checklist.md` 참고)
   - Tensorboard 필수 지표 모니터링
   - VecCheckNan으로 NaN 조기 감지
   - Monitor CSV 분석

---

## 📖 학습 계획

### 1단계: 기초 이론 (완료)
- [x] RL 기본 개념 (MDP, Policy, Value Function)
- [x] PPO 알고리즘 이해
- [x] 보상 설계 원칙

### 2단계: 실전 적용 (진행 중)
- [x] RoArm-M3 환경 구현
- [x] Dense Reward 함수 설계
- [ ] 보상 스케일 조정 및 재학습
- [ ] 하이퍼파라미터 튜닝
- [ ] 성공률 10% 이상 달성

### 3단계: 고급 기법 (예정)
- [ ] Curriculum Learning 도입
- [ ] Multi-task Learning
- [ ] Sim-to-Real Transfer
- [ ] RoArm 실제 하드웨어 배포

---

## 🔗 빠른 참조

### 보상 스케일 문제?
→ `rl_best_practices.md` 섹션 "보상 설계"

### 학습이 안 됨?
→ `rl_debugging_checklist.md` 문제 1

### PPO 파라미터 모르겠음?
→ `ppo_algorithm_guide.md` 섹션 "하이퍼파라미터 상세"

### Tensorboard 뭘 봐야 함?
→ `rl_debugging_checklist.md` 섹션 "디버깅 도구"

---

## 📝 메모

### 2025-10-19
- 온라인 리소스 3개 소스에서 수집 완료
- 로컬에 3개 가이드 문서 작성
- RoArm-M3 보상 스케일 문제 진단
- 다음: 보상 함수 수정 후 재학습

### 핵심 발견사항
1. **보상 스케일이 학습의 90%**: 다른 모든 것보다 중요!
2. **PPO는 안정적**: 기본 설정으로도 잘 작동
3. **디버깅은 Tensorboard**: 반드시 사용해야 함
4. **병렬 환경 필수**: 학습 속도 4-8배 향상

