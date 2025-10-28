# Phase 2 요약: Vision-Based Agricultural RL

**버전**: v1.0  
**날짜**: 2025-10-28

---

## 🎯 **핵심 목표 3가지**

### 1. 시각 기반 강화학습 (Vision-Based RL)
```
현재: Joint 위치 기반 (28-dim vector)
목표: RGB-D 카메라 입력 기반
     Intel RealSense D455 통합
```

### 2. 실제 농업 환경 시뮬레이션
```
현재: 단순 Pick & Place (큐브 + 타겟)
목표: 고추밭 환경 (고추 나무, 빨간/초록 고추)
     실제 수확 시나리오
```

### 3. 이동형 로봇 시스템
```
현재: 고정 베이스 로봇팔
목표: 궤도형 이동 베이스 + 로봇팔
     Mobile Manipulation
```

---

## 🔍 **현재 문제점**

### ❌ **문제 1: URDF 단순성**
```
그리퍼: 단순 프리즘 (잡기 어려움)
카메라: 없음 (시각 입력 불가)
링크: 실제와 불일치
센서: 부족
```
**→ GRIP 0% 원인 중 하나**

### ❌ **문제 2: 환경 구성 경험 부족**
```
경험: 단순 환경만
복잡한 환경: 미경험
Isaac Assets: 로드 실패 경험
```
**→ 고추밭 환경 생성 방법 불명확**

### ❌ **문제 3: 이동 베이스 통합 미경험**
```
현재: 로봇팔만 (고정)
필요: 궤도형 이동체 + 로봇팔
경험: Multi-body 시스템 미경험
```
**→ 복합 제어 방법 불명확**

---

## ✅ **해결 방안 로드맵**

### **Step 1: URDF 개선** (2-3주) ⭐ 최우선
```bash
Week 1: 그리퍼 개선
  - Parallel Jaw 그리퍼 구현
  - 물리 파라미터 최적화
  - Pick & Place 재테스트
  → 목표: GRIP rate > 30%

Week 2: 링크/조인트 디테일
  - 실제 RoArm-M3 스펙 반영
  - DH 파라미터 정확화
  - Inertia/Mass 보정

Week 3: 카메라 통합
  - RealSense D455 URDF 작성
  - 로봇팔에 부착
  - 데이터 수집 테스트
```

### **Step 2: Vision-Based RL** (3-4주)
```bash
Week 4: Observation 구현
  - RGB-D 데이터 파이프라인
  - CNN Encoder 설계
  - 전처리 (84x84, normalize)

Week 5-6: 학습
  - CnnPolicy (Stable-Baselines3)
  - 기존 Pick & Place 환경
  - 100K-500K steps

Week 7: 평가 및 개선
  - 성공률 > 60% 목표
  - Vision vs Joint 비교
```

### **Step 3: 농업 환경** (4-5주)
```bash
Week 8-9: Asset 확보 + 환경 생성
  - 고추/식물 모델 검색
  - 고추밭 Scene 구성
  - USD 최적화

Week 10-11: 고추 수확 Task
  - 보상 함수 설계
  - 빨간 고추만 수확
  - Vision 기반 학습

Week 12: 평가
  - 수확 성공률 측정
  - 감지 정확도 확인
```

### **Step 4: 이동 베이스** (5-6주)
```bash
Week 13-14: 이동체 통합
  - 궤도형 베이스 URDF
  - 로봇팔 장착
  - 제어 인터페이스

Week 15-17: Mobile Manipulation 학습
  - Action space 확장 (10-dim)
  - 이동 + 매니퓰레이션
  - 1M steps

Week 18: 최종 평가
  - 시간당 수확량
  - 성공률 측정
```

**총 예상 기간**: 4-5개월

---

## 🚀 **우선순위 TOP 5**

### 1️⃣ 그리퍼 개선 ⭐⭐⭐⭐⭐
```
이유: 현재 GRIP 0% 문제 해결
영향: 모든 후속 작업의 기초
시간: 1주
난이도: ★★☆☆☆
```

### 2️⃣ 카메라 통합 ⭐⭐⭐⭐☆
```
이유: Vision-based RL 필수
영향: 실제 배포 필수 요소
시간: 1주
난이도: ★★★☆☆
```

### 3️⃣ Vision-based RL ⭐⭐⭐⭐☆
```
이유: 기존 환경에서 먼저 검증
영향: 농업 환경 전 필수
시간: 3-4주
난이도: ★★★★☆
```

### 4️⃣ 농업 환경 ⭐⭐⭐☆☆
```
이유: 실제 시나리오 구현
영향: 최종 목표 달성
시간: 4-5주
난이도: ★★★★☆
```

### 5️⃣ 이동 베이스 ⭐⭐☆☆☆
```
이유: 가장 복잡, 선택적
영향: 확장성 향상
시간: 5-6주
난이도: ★★★★★
```

---

## 🛠️ **즉시 시작 가능 (10M 완료 후)**

### Day 1-2: URDF 분석
```bash
□ 현재 URDF 완전 파싱 및 시각화
□ RoArm-M3 공식 스펙 다운로드
  - https://www.waveshare.com/roarm-m3.htm
□ 비교 분석 문서 작성
□ 개선점 우선순위 결정

# 산출물
docs/URDF_ANALYSIS_REPORT.md
```

### Day 3-5: 그리퍼 개선 시작
```bash
□ Parallel Jaw 그리퍼 URDF 작성
  - 2개 finger links
  - Prismatic joints
  - Collision meshes
□ Isaac Sim 로드 및 테스트
□ 물리 파라미터 튜닝
  - Friction: 1.0-2.0
  - Contact stiffness/damping

# 산출물
assets/roarm_m3/urdf/roarm_m3_gripper_v2.urdf
scripts/test_gripper_physics.py
```

### Week 2: 카메라 조사 및 통합 준비
```bash
□ RealSense D455 URDF 확보
  - GitHub: IntelRealSense/realsense-ros
  - 또는 직접 작성
□ Isaac Sim Camera API 학습
  - RGB/Depth sensor 설정
  - 데이터 캡처 방법
□ 간단한 카메라 테스트

# 산출물
assets/cameras/realsense_d455.urdf
scripts/test_camera_capture.py
```

---

## 📊 **성공 지표 (KPI)**

### Phase 2.1: URDF 개선
```
✅ GRIP rate: 0% → 30%+
✅ 그리퍼 힘: 측정 가능
✅ 카메라 데이터: RGB/Depth 수집 성공
```

### Phase 2.2: Vision-Based RL
```
✅ Pick & Place 성공률: > 60%
✅ Vision vs Joint 비교: 성능 차이 < 10%
✅ 학습 속도: 기존 대비 5-10배 느림 (허용)
```

### Phase 2.3: 농업 환경
```
✅ 고추 감지 정확도: > 90%
✅ 빨간 고추 수확 성공률: > 50%
✅ 초록 고추 회피율: > 95%
```

### Phase 2.4: Mobile Manipulation
```
✅ 이동 + 수확 시연: 성공
✅ 시간당 수확량: 10개+
✅ 에너지 효율: 측정 가능
```

---

## ⚠️ **주요 리스크 및 대응**

### 리스크 1: Asset 확보 실패
```
확률: 30%
영향: 높음
대응: 
  - Plan A: Isaac Nucleus Server
  - Plan B: Sketchfab/TurboSquid
  - Plan C: 직접 모델링 (Blender)
```

### 리스크 2: 학습 속도 저하
```
확률: 50%
영향: 중간
대응:
  - 환경 크기 축소 (5m x 5m)
  - 고추 개수 감소 (10개부터)
  - GPU 최대 활용
  - Frame skip 증가
```

### 리스크 3: Sim-to-Real 격차
```
확률: 80%
영향: 높음
대응:
  - Domain Randomization
  - 실제 데이터 수집 후 Fine-tuning
  - Sim2Real 논문 참고
```

---

## 📚 **필수 학습 자료**

### Papers
```
□ Domain Randomization (Tobin et al., 2017)
□ Learning Dexterous In-Hand Manipulation (OpenAI, 2019)
□ QT-Opt (Kalashnikov et al., 2018)
□ Agricultural Robotics Survey
```

### Tutorials
```
□ Isaac Sim Camera Tutorial
□ Isaac Lab Scene Creation
□ Stable-Baselines3 CnnPolicy
□ USD Scene Graph
```

### Code Examples
```
□ IntelRealSense/realsense-ros
□ NVIDIA Isaac Sim Examples
□ agricultural-robotics (GitHub)
```

---

## 🎯 **현재 상태 (2025-10-28)**

### Phase 1 (기본 RL)
```
✅ RoArm-M3 기본 URDF
✅ Pick & Place 환경
✅ PPO 학습 파이프라인
⏳ 10M steps 학습 진행 중 (746K/10M, 7.46%)
```

### Phase 2 (준비 중)
```
□ URDF 개선
□ 카메라 통합
□ Vision-Based RL
□ 농업 환경
□ 이동 베이스
```

---

## 🔜 **Next Steps (10M 완료 후)**

1. **10M 결과 리뷰** (1일)
   - GRIP 달성 여부 확인
   - 성능 지표 분석
   - 문제점 파악

2. **Phase 2 시작 결정** (1일)
   - 우선순위 최종 확정
   - 리소스 확인
   - 타임라인 조정

3. **URDF 분석 시작** (2-3일)
   - 현재 URDF 상세 분석
   - 실제 스펙 비교
   - 개선 계획 수립

---

**문서 위치**: `docs/PHASE2_VISION_BASED_RL_PLAN.md`  
**작성**: 2025-10-28  
**다음 업데이트**: 10M 학습 완료 후
