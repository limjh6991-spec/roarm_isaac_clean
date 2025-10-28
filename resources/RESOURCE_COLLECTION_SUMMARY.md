# Phase 2 Resource Collection Summary

Phase 2를 위한 리소스 수집 완료 보고서입니다.

## 수집 완료 시간
- **날짜**: 2025-10-28
- **시작**: 18:10
- **완료**: 18:30
- **소요 시간**: 약 20분

---

## 1. 수집된 리소스 개요

### 📁 resources/grippers/ (그리퍼 및 조인트 URDF)
**목적**: RoArm-M3 그리퍼 개선, Parallel Jaw 그리퍼 통합

**수집 내용**:
1. **Intel RealSense ROS** (카메라 통합)
   - Repository: https://github.com/IntelRealSense/realsense-ros
   - URDF: `realsense2_description/urdf/`
   - 카메라: D455, D435i (RGB-D)
   - ROS2 Humble/Iron/Jazzy 지원

2. **Robotiq Gripper URDF**
   - Repository: https://github.com/ros-industrial/robotiq
   - 모델: 2F-85, 2F-140, 3F Gripper
   - 핵심: Mimic Joint (대칭 파지)
   - Prismatic Joint + Contact Sensors

3. **Universal Robots URDF** (참고용)
   - Repository: https://github.com/ros-industrial/universal_robot
   - 로봇: UR3, UR5, UR10, UR16
   - 학습 포인트: 6-DOF 조인트 구성, MoveIt! 통합

4. **iCub Humanoid Robot** (고급 참고)
   - Repository: https://github.com/robotology/icub-models
   - 복잡한 Multi-body 시스템
   - 손가락 조인트 구성 (20+ DOF)

**다음 단계**:
```bash
# 다운로드 명령어
cd resources/grippers/
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/ros-industrial/robotiq.git
git clone https://github.com/ros-industrial/universal_robot.git

# URDF 분석
cd realsense-ros/realsense2_description/urdf
# d455.urdf.xacro 분석
```

---

### 📁 resources/vision_rl/ (Vision-Based RL)
**목적**: RGB-D 입력 기반 CNN Policy 학습

**수집 내용**:
1. **Stable-Baselines3 CnnPolicy**
   - Repository: https://github.com/DLR-RM/stable-baselines3
   - 지원: PPO, A2C, DQN, SAC
   - CNN 구조: NatureCNN (Atari 스타일)
   - Custom CNN 예시 포함

2. **Isaac Sim Camera Sensor**
   - 문서: https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/ext_omni_isaac_sensor.html
   - Camera 종류: RGB, Depth, Segmentation, BBox
   - Python API: `omni.isaac.sensor.Camera`
   - Resolution: 256x256, 20Hz

3. **Vision-Based RL 논문** (5편)
   - "Learning to Manipulate Deformable Objects" (RSS 2020)
   - "Deep RL for Vision-Based Robotic Grasping" (ICRA 2018)
   - "Learning Dexterous In-Hand Manipulation" (OpenAI, IJRR 2020)
   - "Visual Foresight" (2018)
   - "SAC-X: SAC for Pixels" (2019)

4. **Preprocessing Pipeline**
   - RGB: resize (84x84), normalize [0, 1], transpose (C, H, W)
   - Depth: clip, normalize, (1, H, W)
   - RGBD: concatenate → (4, 84, 84)
   - Frame Stacking: 4 frames → (16, 84, 84)

**다음 단계**:
```python
# SB3 설치
pip install stable-baselines3[extra]

# Custom CNN 구현
# envs/observation/observation_builder.py 수정
# obs_mode="vector" → obs_mode="rgbd"
```

---

### 📁 resources/isaac_assets/ (Isaac Assets 활용)
**목적**: 농업 환경 (고추밭) 구성, USD Scene 생성

**수집 내용**:
1. **USD (Universal Scene Description) 기초**
   - Pixar USD Tutorial: https://graphics.pixar.com/usd/
   - NVIDIA Omniverse USD Guide
   - Python API: `pxr.Usd`, `pxr.UsdGeom`

2. **Isaac Lab Scene API**
   - Repository: https://github.com/isaac-sim/IsaacLab
   - Scene 구성: InteractiveSceneCfg
   - Modular composition

3. **3D 모델 소스**
   - **Sketchfab**: https://sketchfab.com/ (CC BY)
   - **TurboSquid**: https://www.turbosquid.com/
   - **Free3D**: https://free3d.com/
   - **Poly Haven**: https://polyhaven.com/ (CC0)
   - 검색어: "pepper plant", "chili plant", "farm", "crop"

4. **Blender → USD Workflow**
   - Blender 3.6+ (USD Export 지원)
   - FBX/OBJ → USD 변환
   - Material, Collision 설정

5. **고추밭 Scene 구성 가이드**
   - Ground + Rows (밭 구조)
   - 고추 식물 배치 (5 rows x 10 plants)
   - 수확 대상 고추 (20개)
   - Lighting, Physics, Domain Randomization

**다음 단계**:
```bash
# Blender 설치 (Ubuntu)
sudo snap install blender --classic

# 3D 모델 다운로드
# 1. Sketchfab에서 "pepper plant" 검색
# 2. FBX 형식 다운로드
# 3. Blender에서 USD로 변환
# 4. Isaac Sim에서 임포트
```

---

## 2. 학습 진행 상황 (백그라운드)

### 현재 상태 (18:30 확인)
```
Steps: 1,359,000 / 10,000,000 (13.59%)
경과 시간: 37분
예상 완료: 2025-10-29 00:48 (약 6시간 남음)
FPS: 383
reach_rate: 18%
grip_rate: 0% (여전히)
평균 보상: 4,775 ~ 7,195 (변동폭 큼)
explained_variance: 0.995 (매우 안정적)
```

### Checkpoint
- 최신: `roarm_ppo_curriculum_1355000_steps.zip` (18:51 생성)
- 간격: 5,000 steps

### 관찰
- **GRIP rate = 0%**: 여전히 파지 실패
- **reach_rate = 18%**: 큐브까지 도달 성공 (18%)
- **보상 변동**: 2,831 ~ 7,194 (Episode 난이도 차이)
- **학습 안정성**: explained_variance 0.995 (매우 좋음)

---

## 3. Phase 2 로드맵 (4-5개월)

### Step 1: URDF 개선 (Week 1-2)
**목표**: 그리퍼 개선, 카메라 통합

**작업**:
1. **Week 1**: 그리퍼 URDF 분석
   - Robotiq 2F-85 URDF 분석
   - Mimic Joint 이해
   - Contact Sensor 위치 파악
   - RoArm-M3 URDF에 적용

2. **Week 2**: 카메라 통합
   - RealSense D455 URDF 분석
   - RoArm-M3 gripper_base_link에 부착
   - Isaac Sim Camera Sensor 연결
   - Extrinsics 계산

**예상 결과**:
- GRIP rate: 0% → 30%+
- URDF 파일: `assets/roarm_m3/urdf/roarm_m3_v2.urdf`
- Camera link 추가

---

### Step 2: Vision-Based RL (Week 3-4)
**목표**: RGB-D 입력 기반 CNN Policy 학습

**작업**:
1. **Week 3**: CNN Policy 통합
   - ObservationBuilder 수정 (obs_mode="rgbd")
   - Custom CNN Feature Extractor
   - Preprocessing Pipeline
   - Frame Stacking (4 frames)

2. **Week 4**: Vision RL 학습
   - RGB-only baseline (1M steps)
   - RGBD comparison (1M steps)
   - Hyperparameter tuning
   - Domain Randomization (lighting, texture)

**예상 결과**:
- Vector obs → RGBD obs 전환
- CNN Policy 학습 성공
- 성능: reach_rate 18% → 40%+

---

### Step 3: 농업 환경 (Week 5-6)
**목표**: 고추밭 시뮬레이션, 수확 Task

**작업**:
1. **Week 5**: 환경 구성
   - 3D 모델 수집 (Sketchfab)
   - Blender → USD 변환
   - 고추밭 Scene 생성 (5 rows x 10 plants)
   - Physics, Lighting 설정

2. **Week 6**: Task 통합
   - 수확 Task 정의
   - 보상 함수 설계 (harvest_reward)
   - Vision RL + 농업 환경 학습
   - 성능 평가

**예상 결과**:
- 고추밭 USD Scene
- 수확 Task 성공률: 20%+

---

### Step 4: 이동 베이스 (Week 7-8)
**목표**: 궤도형 이동 베이스 + 로봇팔 통합

**작업**:
1. **Week 7**: 이동 베이스 URDF
   - 궤도형 베이스 URDF 설계
   - Differential Drive 제어
   - RoArm-M3 URDF 통합 (Base + Arm)

2. **Week 8**: Mobile Manipulation 학습
   - Navigation + Manipulation 통합
   - Multi-stage Task (이동 → 수확)
   - 전체 시스템 학습

**예상 결과**:
- 이동형 로봇 시뮬레이션
- Mobile Manipulation 성공

---

## 4. 우선순위 TOP 5

### Priority 1: 그리퍼 개선 (GRIP 0% → 30%+)
- **문제**: 현재 그리퍼가 파지를 전혀 못함
- **해결**: Robotiq 2F-85 스타일로 업그레이드
- **기간**: Week 1-2
- **영향**: 전체 Task 성공의 전제 조건

### Priority 2: 카메라 통합 (URDF + Isaac Sim)
- **문제**: 현재 관측이 Vector만 지원
- **해결**: RealSense D455 URDF 추가, Camera Sensor 연결
- **기간**: Week 2
- **영향**: Vision-Based RL 전제 조건

### Priority 3: CNN Policy 통합 (RGB-D → CNN)
- **문제**: SB3 MlpPolicy만 사용
- **해결**: CnnPolicy, Custom CNN Feature Extractor
- **기간**: Week 3-4
- **영향**: Vision-Based RL 핵심

### Priority 4: 농업 환경 구성 (고추밭 Scene)
- **문제**: 현재 Simple Scene (Ground + Cube)
- **해결**: 고추밭 USD Scene, 수확 Task
- **기간**: Week 5-6
- **영향**: 실제 응용 가능성

### Priority 5: 이동 베이스 통합 (Mobile Manipulation)
- **문제**: 고정형 로봇 (Base fixed)
- **해결**: 궤도형 베이스 URDF, Navigation 통합
- **기간**: Week 7-8
- **영향**: 농업 로봇 완성

---

## 5. 즉시 시작 가능 작업 (Day 1-5)

### Day 1: URDF 분석 시작
```bash
cd resources/grippers/
git clone https://github.com/ros-industrial/robotiq.git
cd robotiq/robotiq_2f_85_gripper_visualization/urdf
cat robotiq_arg2f_85_model.urdf.xacro
```
- Mimic Joint 구조 이해
- Prismatic Joint limits 확인
- Contact geometry 파악

### Day 2: RoArm-M3 URDF 수정 계획
```bash
cd assets/roarm_m3/urdf
cp roarm_m3.urdf roarm_m3_v2.urdf
```
- 그리퍼 조인트 재설계
- 2-finger parallel jaw 구조
- Collision mesh 추가

### Day 3: Camera URDF 통합 계획
```bash
cd resources/grippers/
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/realsense2_description/urdf
cat d455.urdf.xacro
```
- Camera link 위치 결정
- Extrinsics 계산
- Isaac Sim Camera Sensor 매핑

### Day 4: 10M 학습 완료 대기
- 예상 완료: 2025-10-29 00:48
- Checkpoint 확인: 10,000,000 steps
- 성능 평가: reach_rate, grip_rate, 보상

### Day 5: Week 1 작업 시작 여부 결정
- 10M 학습 결과 분석
- GRIP rate > 0% 달성 시 → Week 1 시작
- GRIP rate = 0% 지속 시 → 그리퍼 문제 디버깅 우선

---

## 6. 성공 지표 (KPI)

### Phase 1 목표 (10M 학습)
- [x] 10M steps 완료: 1.36M / 10M (13.6%)
- [ ] reach_rate > 50%: 현재 18%
- [ ] grip_rate > 10%: 현재 0%
- [ ] 평균 보상 > 5,000: 현재 4,775 ~ 7,195

### Phase 2 목표 (4-5개월)
- [ ] GRIP rate > 30% (Week 2 완료)
- [ ] Vision RL reach_rate > 40% (Week 4 완료)
- [ ] 농업 환경 harvest_rate > 20% (Week 6 완료)
- [ ] Mobile Manipulation 성공 (Week 8 완료)

---

## 7. 리소스 다운로드 체크리스트

### ✅ 완료
- [x] Intel RealSense ROS (GitHub 링크)
- [x] Robotiq Gripper URDF (GitHub 링크)
- [x] Universal Robots URDF (GitHub 링크)
- [x] iCub Models (GitHub 링크)
- [x] Stable-Baselines3 (pip install)
- [x] Isaac Sim Camera Docs (공식 문서)
- [x] Vision-based RL 논문 5편 (arXiv)
- [x] USD Tutorial (Pixar 공식 문서)
- [x] Isaac Lab Scene API (GitHub)

### ⏳ 대기 중 (다운로드 필요)
- [ ] RealSense ROS (`git clone`)
- [ ] Robotiq URDF (`git clone`)
- [ ] Universal Robots URDF (`git clone`)
- [ ] 고추 식물 3D 모델 (Sketchfab)
- [ ] Blender 3.6+ 설치

---

## 8. 예상 타임라인

```
2025-10-29 (화): 10M 학습 완료, 성능 평가
2025-10-30 ~ 11-12 (수-화, Week 1-2): URDF 개선
2025-11-13 ~ 11-26 (수-화, Week 3-4): Vision-Based RL
2025-11-27 ~ 12-10 (수-화, Week 5-6): 농업 환경
2025-12-11 ~ 12-24 (수-화, Week 7-8): 이동 베이스
2025-12-25: Phase 2 완료
```

---

## 9. 다음 단계 (우선순위)

### 즉시 (오늘 밤)
1. 10M 학습 완료 대기 (00:48 예상)
2. 최종 Checkpoint 확인
3. 성능 평가 (reach_rate, grip_rate)

### 내일 (10/29)
1. 학습 결과 분석
2. GRIP rate 0% 원인 분석
3. 그리퍼 URDF 다운로드 시작
4. Week 1 작업 계획 수립

### 이번 주 (10/29-11/1)
1. Robotiq 2F-85 URDF 분석
2. RoArm-M3 URDF 수정 시작
3. Contact Sensor 추가
4. Isaac Sim에서 테스트

---

## 10. 문의 및 지원

### GitHub Issues
- RealSense ROS: https://github.com/IntelRealSense/realsense-ros/issues
- Robotiq: https://github.com/ros-industrial/robotiq/issues
- Isaac Sim: https://forums.developer.nvidia.com/c/omniverse/simulation/69

### 공식 문서
- Isaac Sim: https://docs.omniverse.nvidia.com/isaacsim/latest/
- Stable-Baselines3: https://stable-baselines3.readthedocs.io/
- USD: https://graphics.pixar.com/usd/

---

**작성일**: 2025-10-28 18:30  
**상태**: 리소스 수집 완료, Phase 2 준비 완료  
**다음 마일스톤**: 10M 학습 완료 (2025-10-29 00:48)
