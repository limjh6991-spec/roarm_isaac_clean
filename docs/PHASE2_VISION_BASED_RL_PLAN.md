# Phase 2: Vision-Based RL in Agricultural Environment

**작성일**: 2025-10-28  
**목표**: 시각 기반 강화학습 + 실제 고추밭 환경 시뮬레이션

---

## 🎯 **최종 목표**

### 1. 시각 기반 강화학습
- Intel RealSense D455 카메라 통합
- RGB-D 데이터 기반 학습
- 고추 인식 및 수확 자동화

### 2. 실제 농업 환경 시뮬레이션
- 고추밭 환경 생성 (토양, 고추 나무, 고추)
- 궤도형 이동 베이스 + 로봇팔 통합
- 실제 수확 시나리오 학습

---

## 🔍 **현재 상황 분석**

### ✅ **완료된 것**
```
Phase 1 (기본 Pick & Place):
- RoArm-M3 URDF 기본 구성 ✅
- 단순 환경 (큐브, 타겟) ✅
- PPO 기반 강화학습 파이프라인 ✅
- 10M steps 학습 진행 중 (현재 746K) ✅
- 모듈화된 코드 구조 (v3.9.6) ✅
```

### ⚠️ **문제점 (Phase 2 진입 장벽)**

#### 2-1. URDF 단순성
```yaml
현재 상태:
  - 링크: 단순화된 구조 (실제와 불일치)
  - 그리퍼: 기본 프리즘 형태 (잡기 기능 부족)
  - 카메라: 없음 (시각 입력 불가)
  - 센서: 없음 (깊이, 힘 센서 등)

문제:
  ❌ 실제 로봇과 sim-to-real 격차 큼
  ❌ 그리퍼 성능 제한 (GRIP 0% 원인 중 하나)
  ❌ 시각 기반 학습 불가능
  ❌ 물리적 정밀도 부족
```

#### 2-2. 환경 구성 경험 부족
```yaml
현재 상태:
  - 경험: 단순 Pick & Place만
  - Isaac Lab Assets: 로드 실패 경험
  - 복잡한 환경: 미경험

문제:
  ❌ 고추밭 환경 생성 방법 불명확
  ❌ 식물/농작물 모델링 노하우 없음
  ❌ 대규모 환경 최적화 미경험
```

#### 2-3. 이동 베이스 통합
```yaml
현재 상태:
  - 로봇팔만 존재 (고정 베이스)
  - 궤도형 이동체 경험 없음
  - Multi-body 시스템 미경험

문제:
  ❌ 궤도형 베이스 URDF 미확보
  ❌ 로봇팔 + 이동체 결합 방법 불명확
  ❌ 복합 제어 (이동 + 매니퓰레이션) 미경험
```

---

## 📋 **단계별 실행 계획**

---

## **Step 1: URDF 개선 (기초 작업)** ⭐ **최우선**

### 1.1 현재 URDF 분석 및 문제점 파악
```bash
# 작업 목표
- 현재 URDF 구조 완전 분석
- 실제 RoArm-M3 스펙과 비교
- 개선점 리스트업

# 실행 계획 (1일)
□ 현재 URDF 파싱 및 시각화
□ 실제 RoArm-M3 스펙 문서 확보
  - 공식 문서: https://www.waveshare.com/roarm-m3.htm
  - CAD 파일 확보 (가능하면)
□ 비교 분석 문서 작성
□ 개선 우선순위 결정

# 예상 산출물
- docs/URDF_ANALYSIS_REPORT.md
- docs/ROARM_M3_SPEC_COMPARISON.md
```

### 1.2 그리퍼 구조 개선
```bash
# 작업 목표
- 실제 평행 그리퍼(Parallel Jaw) 구현
- 접촉 물리 개선 (Friction, Contact)
- 힘 센서 추가 (선택)

# 실행 계획 (2-3일)
□ Parallel Jaw 그리퍼 URDF 작성
  - 2개 finger link
  - Prismatic joint (평행 이동)
  - 적절한 collision mesh
□ 물리 파라미터 조정
  - Friction: 1.0-2.0
  - Contact stiffness/damping
□ Isaac Sim에서 테스트
  - 큐브 잡기 성공률 확인
  - 접촉력 시각화
□ Pick & Place 환경 재테스트

# 참고 자료
- Isaac Sim Gripper Examples
- USD Parallel Gripper 템플릿
- Pybullet Gripper URDF 예제

# 예상 산출물
- assets/roarm_m3/urdf/roarm_m3_improved_gripper.urdf
- scripts/test_gripper_physics.py
```

### 1.3 링크 및 조인트 디테일 강화
```bash
# 작업 목표
- 실제 RoArm-M3와 동일한 링크 수/크기
- 정확한 DH 파라미터 반영
- Inertia 및 Mass 정확화

# 실행 계획 (2일)
□ 실제 로봇 측정 또는 CAD 데이터 확보
  - Link 길이: L1, L2, L3, L4
  - Joint 범위: ±180° (실제 값)
  - Mass 분포
□ URDF 재작성
  - 정확한 origin/rpy/xyz
  - 정확한 inertia 텐서
□ Isaac Sim Forward Kinematics 검증
□ 동작 범위 테스트

# 예상 산출물
- assets/roarm_m3/urdf/roarm_m3_detailed.urdf
- docs/DH_PARAMETERS.md
```

---

## **Step 2: Intel RealSense D455 통합** 🎥

### 2.1 RealSense D455 URDF 생성
```bash
# 작업 목표
- D455 카메라 URDF 작성
- RGB + Depth 센서 정의
- 물리적 크기/무게 정확 반영

# 실행 계획 (2일)
□ Intel 공식 URDF/CAD 다운로드
  - GitHub: IntelRealSense/realsense-ros
  - 또는 직접 작성
□ Isaac Sim 카메라 센서 설정
  - RGB: 1920x1080
  - Depth: 1280x720
  - FOV: 87° x 58°
□ USD로 변환 및 테스트
□ 데이터 캡처 스크립트 작성

# 참고 자료
- Intel RealSense ROS 패키지
- Isaac Sim Camera Sensor API
- realsense2_description 패키지

# 예상 산출물
- assets/cameras/realsense_d455.urdf
- assets/cameras/realsense_d455.usd
- scripts/test_camera_capture.py
```

### 2.2 RoArm-M3 + D455 통합
```bash
# 작업 목표
- 카메라를 그리퍼 또는 EE에 부착
- 카메라 좌표계 정확히 설정
- 데이터 파이프라인 구축

# 실행 계획 (2일)
□ 부착 위치 결정
  - Option 1: 그리퍼 손가락 사이
  - Option 2: EE 상단
  - Option 3: 별도 링크 추가
□ URDF 통합
  - Fixed joint로 부착
  - 정확한 transform 설정
□ Isaac Sim에서 시각화
  - RGB/Depth 렌더링 확인
  - 좌표계 일치 확인
□ 데이터 수집 테스트

# 예상 산출물
- assets/roarm_m3/urdf/roarm_m3_with_camera.urdf
- scripts/test_camera_integration.py
- docs/CAMERA_CALIBRATION.md
```

### 2.3 Vision-based Observation 구현
```bash
# 작업 목표
- RGB-D 데이터를 관측 공간에 추가
- CNN 기반 Feature Extraction
- 학습 파이프라인 수정

# 실행 계획 (3-4일)
□ 관측 공간 재설계
  - 기존: 28-dim vector
  - 추가: RGB (84x84x3) + Depth (84x84x1)
  - CNN Encoder 설계
□ 환경 수정
  - envs/observation/camera_builder.py 생성
  - RGB-D 전처리 (resize, normalize)
□ PPO 모델 수정
  - CnnPolicy 사용
  - MultiInputPolicy 고려
□ 학습 테스트 (10K steps)

# 참고 자료
- Stable-Baselines3 CnnPolicy
- Isaac Sim Camera API
- Vision-based RL 논문

# 예상 산출물
- envs/observation/camera_builder.py
- scripts/rl/train_vision_based.py
- docs/VISION_BASED_RL_DESIGN.md
```

---

## **Step 3: 농업 환경 구축** 🌱

### 3.1 Isaac Lab/Sim Asset 조사
```bash
# 작업 목표
- 사용 가능한 농업 관련 Asset 확보
- 고추/식물 모델 검색
- 토양/환경 Asset 확보

# 실행 계획 (2일)
□ Isaac Sim Nucleus Server 검색
  - 식물 모델: /Isaac/Environments/Simple_Plants
  - 지형: /Isaac/Environments/Simple_Terrain
  - Props: 나무, 잎, 열매
□ Sketchfab/TurboSquid 검색
  - "pepper plant" "chili plant"
  - USD/OBJ/FBX 형식
□ Asset 변환 및 테스트
  - FBX → USD 변환
  - Physics collider 추가
□ Asset 라이브러리 구축

# 예상 산출물
- assets/agriculture/peppers/
- assets/agriculture/soil/
- docs/ASSET_LIBRARY.md
```

### 3.2 고추밭 환경 생성
```bash
# 작업 목표
- 실제 고추밭과 유사한 환경
- 여러 고추 나무 배치
- 수확 가능한 고추 생성

# 실행 계획 (3-4일)
□ 환경 설계
  - 크기: 10m x 10m (시작)
  - 고추 나무: 20-30개
  - 고추: 빨강(수확 가능), 초록(미성숙)
□ USD Scene 구성
  - Ground plane (토양 텍스처)
  - 고추 나무 배치 (랜덤 위치)
  - 고추 randomization
□ Isaac Lab 환경 클래스 작성
  - envs/agriculture_env.py
  - 고추 위치/색상 randomization
  - Success condition: 빨간 고추 수확
□ 물리 시뮬레이션 최적화
  - Collision 최소화
  - GPU 가속 활용

# 참고 자료
- Isaac Lab Scene Creation Tutorial
- USD Scene Graph
- Randomization API

# 예상 산출물
- assets/scenes/pepper_farm_scene.usd
- envs/agriculture_env.py
- scripts/test_agriculture_env.py
```

### 3.3 고추 감지 및 수확 Task 정의
```bash
# 작업 목표
- Vision 기반 고추 감지
- 빨간 고추 타겟팅
- 수확 동작 학습

# 실행 계획 (2일)
□ 보상 함수 설계
  - 고추 감지: +10
  - 빨간 고추 접근: +20
  - 고추 잡기: +50
  - 고추 따기: +100
  - 잘못된 고추: -20
□ Success condition
  - 빨간 고추만 수확
  - 초록 고추 피하기
□ Observation 설계
  - RGB-D from camera
  - 고추 위치/색상 (GT for baseline)
□ Episode 설정
  - 길이: 1000 steps
  - 리셋: 새로운 고추 위치

# 예상 산출물
- docs/AGRICULTURE_TASK_DESIGN.md
- envs/reward/agriculture_reward.py
```

---

## **Step 4: 궤도형 이동 베이스 통합** 🚜

### 4.1 이동 베이스 검색 및 선정
```bash
# 작업 목표
- 농업용 궤도형 이동체 찾기
- URDF 확보 또는 생성

# 실행 계획 (2-3일)
□ 기존 URDF 검색
  - Clearpath Husky (tracked version)
  - TurtleBot with tracks
  - Custom agricultural robot
□ URDF 수정/생성
  - Track simulation (바퀴로 근사)
  - 적절한 크기/무게
  - Isaac Sim physics 테스트
□ 이동 제어 테스트
  - 속도 제어
  - 회전 제어
  - 지형 주행

# 참고 자료
- Clearpath Robotics URDF
- Isaac Sim Wheeled Robot Examples
- Agricultural Robot Papers

# 예상 산출물
- assets/mobile_base/tracked_base.urdf
- scripts/test_mobile_base.py
```

### 4.2 로봇팔 + 이동 베이스 통합
```bash
# 작업 목표
- RoArm-M3를 이동 베이스에 장착
- 복합 시스템 제어

# 실행 계획 (2일)
□ URDF 통합
  - Fixed joint로 로봇팔 장착
  - 높이/위치 조정
  - 무게 중심 고려
□ 제어 인터페이스 설계
  - 이동: (v_x, v_y, omega)
  - 매니퓰레이션: (joint velocities)
□ Isaac Sim 시뮬레이션
  - 이동 중 안정성 확인
  - 매니퓰레이션 가능 여부
□ 학습 환경 수정
  - Action space 확장 (3 + 7 = 10-dim)
  - Observation 확장 (base pose)

# 예상 산출물
- assets/complete_system/roarm_m3_on_tracked_base.urdf
- envs/mobile_manipulation_env.py
```

### 4.3 Mobile Manipulation 학습
```bash
# 작업 목표
- 이동 + 매니퓰레이션 동시 학습
- 고추밭 내비게이션

# 실행 계획 (5-7일)
□ 환경 설정
  - 넓은 고추밭 (20m x 20m)
  - 여러 고추 나무
  - 랜덤 시작 위치
□ 학습 전략
  - Curriculum: 고정 → 이동
  - Hierarchical RL 고려
  - 이동/매니퓰레이션 분리 학습?
□ PPO 학습 (1M steps)
□ 성능 평가
  - 수확 성공률
  - 이동 효율성
  - 시간당 수확량

# 예상 산출물
- scripts/rl/train_mobile_manipulation.py
- docs/MOBILE_MANIPULATION_RESULTS.md
```

---

## 🔧 **기술적 도전과제 및 해결 방안**

### 도전 1: Vision-based RL의 학습 속도
```yaml
문제:
  - RGB-D 입력 → 학습 느림 (10배 이상)
  - 대용량 네트워크 (CNN)

해결:
  ✅ GPU 가속 필수 (현재 RTX 5090 보유)
  ✅ Image size 축소 (84x84 권장)
  ✅ Frame skip 사용 (4 frames)
  ✅ Latent representation 사전 학습
  ✅ Auxiliary tasks (depth prediction, segmentation)
```

### 도전 2: 복잡한 환경의 시뮬레이션 속도
```yaml
문제:
  - 고추밭 환경 → 많은 객체 (수백 개)
  - 물리 계산 부하 증가
  - FPS 감소 (100 → 10?)

해결:
  ✅ Static meshes 최대 활용 (collision 최소화)
  ✅ LOD (Level of Detail) 적용
  ✅ Occlusion culling
  ✅ GPU physics pipeline 사용
  ✅ 환경 크기 점진적 확장 (5m → 10m → 20m)
```

### 도전 3: Sim-to-Real 격차
```yaml
문제:
  - 시뮬레이션 ≠ 실제
  - Domain gap (외형, 물리, 센서)

해결:
  ✅ Domain Randomization
    - 조명 변화
    - 텍스처 변화
    - 노이즈 추가 (센서, 액추에이터)
  ✅ 실제 로봇 데이터 수집 → Fine-tuning
  ✅ Sim2Real 논문 참고 (Tobin et al., 2017)
```

### 도전 4: Multi-body System 제어
```yaml
문제:
  - 이동 + 매니퓰레이션 → 복잡
  - Action space 커짐 (10-dim)
  - 학습 난이도 상승

해결:
  ✅ Hierarchical RL
    - High-level: 어디로 이동?
    - Low-level: 어떻게 수확?
  ✅ Curriculum Learning
    - Stage 1: 고정 베이스
    - Stage 2: 제한된 이동
    - Stage 3: 완전 자유
  ✅ Modular RL (이동/매니퓰레이션 분리 학습)
```

---

## 📅 **전체 타임라인 (예상)**

### Phase 2.1: URDF 개선 (2-3주)
```
Week 1: 현재 URDF 분석 + 그리퍼 개선
Week 2: 링크/조인트 디테일 강화 + 테스트
Week 3: RealSense D455 통합 + 카메라 테스트
```

### Phase 2.2: Vision-based RL (3-4주)
```
Week 4: Vision observation 구현
Week 5-6: Vision-based 학습 (Pick & Place)
Week 7: 성능 평가 및 개선
```

### Phase 2.3: 농업 환경 구축 (4-5주)
```
Week 8-9: Asset 확보 + 환경 생성
Week 10-11: 고추 수확 Task 구현 + 학습
Week 12: 성능 평가
```

### Phase 2.4: 이동 베이스 통합 (5-6주)
```
Week 13-14: 이동 베이스 확보 + 통합
Week 15-17: Mobile Manipulation 학습
Week 18: 최종 평가 및 문서화
```

**총 예상 기간**: 4-5개월

---

## 📚 **학습 로드맵 (지식 습득)**

### 필수 학습 항목
```
□ Isaac Sim Advanced
  - Camera Sensor API
  - Scene Composition (USD)
  - Physics Optimization
  
□ URDF/USD 심화
  - Sensor 정의
  - Complex kinematics
  - USD Schema
  
□ Vision-based RL
  - CNN Policy (SB3)
  - Image preprocessing
  - Vision papers (DQN, SAC-AE)
  
□ Domain Randomization
  - Light randomization
  - Texture randomization
  - Dynamics randomization
  
□ Agricultural Robotics
  - Fruit detection
  - Grasping strategies
  - Mobile manipulation
```

### 추천 자료
```
📖 Papers:
- "Domain Randomization for Transferring Deep Neural Networks..." (Tobin, 2017)
- "Learning Dexterous In-Hand Manipulation" (OpenAI, 2019)
- "QT-Opt: Scalable Deep Reinforcement Learning..." (Kalashnikov, 2018)

📚 Tutorials:
- Isaac Sim Camera Tutorial
- Isaac Lab Scene Creation
- Stable-Baselines3 CnnPolicy Examples

🔗 Repositories:
- IntelRealSense/realsense-ros
- NVIDIA Isaac Sim Examples
- agricultural-robotics (GitHub)
```

---

## 🎯 **즉시 시작 가능한 작업 (Phase 10M 완료 후)**

### Week 1 Quick Start
```bash
# Day 1: URDF 분석
□ 현재 URDF 완전 파싱
□ RoArm-M3 공식 스펙 다운로드
□ 비교 분석 시작

# Day 2-3: 그리퍼 개선 시작
□ Parallel Jaw 그리퍼 URDF 초안 작성
□ Isaac Sim에서 로드 테스트
□ 물리 파라미터 조정

# Day 4-5: 카메라 조사
□ RealSense D455 URDF 다운로드/작성
□ Isaac Sim Camera API 학습
□ 간단한 카메라 테스트

# 산출물
- docs/URDF_ANALYSIS_REPORT.md
- assets/roarm_m3/urdf/roarm_m3_gripper_v2.urdf
- assets/cameras/realsense_d455.urdf (초안)
```

---

## 💡 **자비스의 추천 전략**

### 우선순위 순서
```
1️⃣ URDF 개선 (그리퍼) ⭐⭐⭐⭐⭐
   → 현재 GRIP 0% 문제 해결
   → 모든 후속 작업의 기초

2️⃣ 카메라 통합 ⭐⭐⭐⭐
   → Vision-based RL의 필수 요소
   → 실제 배포 필수

3️⃣ Vision-based RL ⭐⭐⭐⭐
   → 기존 환경에서 먼저 검증
   → 농업 환경 전 필수

4️⃣ 농업 환경 구축 ⭐⭐⭐
   → Asset 확보가 관건
   → 점진적 확장 (간단 → 복잡)

5️⃣ 이동 베이스 통합 ⭐⭐
   → 가장 복잡
   → 마지막에 추가 권장
```

### 리스크 관리
```
🔴 High Risk:
- Asset 확보 실패 → 대체 방안: 직접 모델링
- 학습 속도 저하 → 대체: 작은 환경부터
- Sim-to-Real 격차 → 대체: Domain Randomization

🟡 Medium Risk:
- URDF 복잡도 증가 → 점진적 개선
- 카메라 통합 어려움 → 공식 예제 참고

🟢 Low Risk:
- 코드 구조 변경 → 현재 모듈화 완료
```

### Success Metrics
```
✅ Phase 2.1 완료:
- GRIP rate > 30% (개선된 그리퍼)
- 카메라 데이터 수집 성공

✅ Phase 2.2 완료:
- Vision-based Pick & Place 성공률 > 60%

✅ Phase 2.3 완료:
- 고추 감지 정확도 > 90%
- 빨간 고추 수확 성공률 > 50%

✅ Phase 2.4 완료:
- Mobile manipulation 시연 성공
- 시간당 10개 이상 수확
```

---

## 📝 **다음 미팅 안건**

1. **10M 학습 결과 리뷰**
   - GRIP 달성 여부
   - 성능 평가
   - 문제점 파악

2. **Phase 2 시작 결정**
   - URDF 개선부터 시작?
   - 예산/리소스 확인
   - 타임라인 조정

3. **기술 스택 확정**
   - Isaac Sim vs Isaac Lab
   - 추가 도구 필요 여부

---

**작성자**: Jarvis AI  
**리뷰 필요**: Phase 10M 완료 후  
**다음 업데이트**: 2025-10-29 (10M 결과 확인 후)
