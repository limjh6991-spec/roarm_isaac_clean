# RoArm M3 URDF → USD 변환 가이드 (Isaac Sim GUI)

**날짜**: 2025-10-18  
**목적**: 공식 스펙 기반 완벽한 URDF를 Isaac Sim USD로 안전하게 변환

---

## 📋 요약

- **URDF 파일**: `assets/roarm_m3/urdf/roarm_m3_v2_complete.urdf`
- **출력 USD**: `assets/roarm_m3/usd/roarm_m3_v2.usd`
- **특징**: Mesh 파일 없음 (순수 primitive geometry) → 경로 문제 최소화
- **전략**: 디버깅용 프리셋으로 검증 → 운용용 프리셋으로 최적화

---

## ⚠️ 잠재적 문제 체크리스트 (현장 12가지 함정)

### 1. Joint 축(axis) 오해
- **문제**: URDF `<axis xyz="..."/>`가 부모 프레임 기준. Import 후 축이 의도와 다를 수 있음
- **조치**: 각 관절의 Axis(X/Y/Z) 실제 회전 방향 확인

### 2. Continuous → Revolute 변환
- **문제**: `continuous` joint가 제한 있는 `revolute`로 변환될 수 있음
- **조치**: `hasLimits=false` 또는 ±π 이상으로 설정

### 3. Fixed Joint Merge 부작용
- **문제**: Merge 시 센서/툴 프레임 사라질 수 있음
- **조치**: 초기 디버그는 Merge OFF, 안정 후 ON

### 4. Inertia(질량/관성) 품질
- **문제**: 비현실적 관성(0 또는 비대칭) → 폭주/진동
- **조치**: Import 시 "Compute Inertia from Collision" 고려, mass > 0 확인

### 5. Massless/Collisionless 링크
- **문제**: 시각만 있고 충돌/질량 없으면 관절 구속 비정상
- **조치**: 최소한의 collision shape와 mass 보장

### 6. Collision 두께/간격(Contact Offset)
- **문제**: 얇은 형상 → 관통/진동
- **조치**: Contact Offset ≈ 1-3mm, Rest Offset ≈ 0-1mm

### 7. Self-Collision 폭주
- **문제**: 전부 켜면 인접 링크 간 불필요 접촉 → 떨림
- **조치**: 초기 OFF, 엔드이펙터만 선택적 ON

### 8. 초기 자세(Drive Target) 불일치
- **문제**: 초기 target이 충돌 상태면 시작부터 튐
- **조치**: Drive Target=0 등 중립 포즈로 시작

### 9. 월드 축/단위
- **문제**: Z-up, m 단위 불일치 → 링크 90° 틀어짐
- **조치**: Stage Up Axis=Z, MetersPerUnit=1.0 확인

### 10. 중첩 Articulation Root
- **문제**: 상위 Xform에 루트 중첩 → 구동 안됨
- **조치**: 최상위 로봇 prim 하나에만 ArticulationRootAPI

### 11. Joint/Link Name 중복
- **문제**: 이름 충돌 → 임포터가 리네임/스킵
- **조치**: URDF에서 전부 unique 보장 (✅ 현재 URDF 확인 완료)

### 12. Drive 타입 과도/부족 제어
- **문제**: 과도한 Stiffness/Damping → 떨림
- **조치**: 디버그는 Velocity 기반이 안정적, 점진 튜닝

---

## 🔧 Import 프리셋

### A. **디버깅용** (1차 시도: 프레임 보존, 원인 파악 우선)

```yaml
Fix Base Link: TRUE                    # 초기 낙하 방지
Import Inertia Tensor: TRUE             # URDF 값 우선 (이상 시 "Compute from Collision")
Merge Fixed Joints: FALSE               # 프레임 추적·센서 포즈 확인용
Self Collision: FALSE                   # 초기엔 끄기
Create Physics Scene: TRUE
Default Prim: TRUE

Joint Drive:
  Type: Velocity                        # Position보다 안정적
  Stiffness: 2000                       # 보수적 시작 (10000의 1/5)
  Damping: 200                          # 보수적 시작 (1000의 1/5)

Stage Settings:
  Up Axis: Z
  Meters Per Unit: 1.0
```

**목적**: 
- 프레임 계층 구조 확인
- 축 방향 검증
- 관성/질량 문제 파악
- 떨림/폭주 원인 추적

---

### B. **운용용** (2차 시도: 성능/간소화 우선)

```yaml
Fix Base Link: TRUE                    # 데모용 고정 (주행 복합 시 FALSE)
Import Inertia Tensor: TRUE             # 검증 통과한 값
Merge Fixed Joints: TRUE                # 성능 향상 (4개 fixed joint 병합)
Self Collision: FALSE                   # 필요 시 엔드이펙터만 부분 ON
Create Physics Scene: TRUE
Default Prim: TRUE

Joint Drive:
  Type: Position                        # 목표 작업에 맞춤 (학습 시 Effort)
  Stiffness: 5000                       # 디버깅 후 점진 증가
  Damping: 500                          # 떨림 없는 수준
  
Stage Settings:
  Up Axis: Z
  Meters Per Unit: 1.0
```

**목적**:
- 최종 시연/학습용
- 프레임 간소화 (world, tcp 등 병합)
- 성능 최적화

---

## 📝 단계별 실행 절차

### **Phase 1: 디버깅용 Import (필수)**

#### Step 1: Isaac Sim 실행
```bash
cd ~/roarm_isaac_clean
source ~/isaacsim-venv/bin/activate
isaacsim
```

#### Step 2: URDF Import (디버깅 프리셋)
1. **File → Import → URDF**
2. **파일 선택**: `assets/roarm_m3/urdf/roarm_m3_v2_complete.urdf`
3. **Import Config 설정** (좌측 패널):
   ```
   ✓ Fix Base Link: TRUE
   ✓ Import Inertia Tensor: TRUE
   ✓ Merge Fixed Joints: FALSE        ⭐ 디버깅용
   ✓ Self Collision: FALSE
   ✓ Create Physics Scene: TRUE
   ✓ Default Prim: TRUE
   
   Joint Drive Settings:
   ✓ Drive Type: Velocity              ⭐ 안정적
   ✓ Stiffness: 2000                   ⭐ 보수적
   ✓ Damping: 200                      ⭐ 보수적
   ```
4. **Import** 버튼 클릭

#### Step 3: 시각적 검증
1. **Viewport 확인**:
   - 로봇이 원점에 제대로 배치되었는지
   - 링크들이 정상적으로 보이는지
   
2. **Stage 패널 확인** (좌측):
   ```
   World
   └─ roarm_m3 (ArticulationRootAPI)
       ├─ base_link
       ├─ link_1
       ├─ link_2 (shoulder)
       ├─ link_3 (elbow)
       ├─ link_4 (wrist1)
       ├─ link_5 (wrist2)
       ├─ gripper_base
       ├─ gripper_left_finger
       ├─ gripper_right_finger
       └─ tcp
   ```
   - 총 11개 링크 확인 (Merge OFF 상태)

3. **Property 패널 확인** (우측):
   - 각 link 선택 → Mass API → `mass > 0` 확인
   - 각 joint 선택 → PhysicsJoint → Axis 방향 확인
     - joint_1 (base): axis=(0,0,1) Z축 회전
     - joint_2 (shoulder): axis=(0,1,0) Y축 회전
     - joint_3 (elbow): axis=(0,1,0) Y축 회전
     - joint_4 (wrist1): axis=(0,1,0) Y축 회전
     - joint_5 (wrist2): axis=(1,0,0) X축 회전
     - joint_6 (gripper): axis=(0,0,1) Z축 회전

#### Step 4: Physics 테스트
1. **Play 버튼 클릭** (Viewport 상단)
2. **확인 사항**:
   - ❌ 로봇이 바닥으로 떨어지지 않는가? (Fix Base 동작 확인)
   - ❌ Joint가 폭주하거나 떨리지 않는가?
   - ❌ 링크가 관통하거나 비정상적으로 접촉하지 않는가?
   
3. **문제 발생 시**:
   - **떨림/진동**: Damping 증가 (200 → 500)
   - **너무 느림**: Stiffness 증가 (2000 → 3000)
   - **폭주**: Drive Type을 Effort로 변경, limit 재확인
   - **관통**: Contact Offset 증가 (Property → PhysicsCollision)

#### Step 5: Joint 수동 제어 테스트
1. **Stop 버튼** 클릭
2. **Stage에서 joint 선택** (예: joint_1)
3. **Property → PhysicsDriveAPI**:
   - `targetPosition` 값 변경 (예: 0 → 0.5)
4. **Play** 후 관절이 부드럽게 움직이는지 확인

#### Step 6: 디버깅용 USD 저장
```
File → Save As
경로: assets/roarm_m3/usd/roarm_m3_v2_debug.usd
```
**용도**: 문제 발생 시 분석용 백업

---

### **Phase 2: 운용용 Import (최적화)**

#### Step 1: 새 Stage 시작
```
File → New
```

#### Step 2: URDF Re-import (운용 프리셋)
1. **File → Import → URDF**
2. **동일 파일**: `assets/roarm_m3/urdf/roarm_m3_v2_complete.urdf`
3. **Import Config 변경**:
   ```
   ✓ Fix Base Link: TRUE
   ✓ Import Inertia Tensor: TRUE
   ✓ Merge Fixed Joints: TRUE         ⭐ 운용용 (4개 fixed joint 병합)
   ✓ Self Collision: FALSE
   ✓ Create Physics Scene: TRUE
   ✓ Default Prim: TRUE
   
   Joint Drive Settings:
   ✓ Drive Type: Position              ⭐ 목표 작업용
   ✓ Stiffness: 5000                   ⭐ 디버깅 후 조정값
   ✓ Damping: 500                      ⭐ 안정화된 값
   ```
4. **Import**

#### Step 3: 구조 확인
- **Stage 패널**:
  ```
  World
  └─ roarm_m3 (ArticulationRootAPI)
      ├─ base_link (world 병합됨)
      ├─ link_1
      ├─ link_2
      ├─ link_3
      ├─ link_4
      ├─ link_5
      └─ gripper_base (tcp, fingers 병합됨)
  ```
  - 약 7-8개 링크로 간소화 (정확한 개수는 병합 규칙에 따라 변동)

#### Step 4: 최종 검증
1. **Play** → 동작 확인
2. **Joint 제어 테스트**
3. **성능 확인**: FPS, PhysX 업데이트 시간

#### Step 5: 최종 USD 저장
```
File → Save As
경로: assets/roarm_m3/usd/roarm_m3_v2.usd
```

---

## ✅ 성공 기준

### 디버깅 단계 (Phase 1)
- [ ] Import 완료 (에러 없음)
- [ ] 11개 링크 모두 Stage에 존재
- [ ] 각 링크 mass > 0
- [ ] Play 시 로봇이 바닥에 고정 (낙하 안함)
- [ ] Joint 떨림/폭주 없음
- [ ] 수동 Joint 제어 정상 동작

### 운용 단계 (Phase 2)
- [ ] Fixed joint 병합 완료 (7-8개 링크)
- [ ] Play 시 안정적 동작
- [ ] Position Drive로 정밀 제어 가능
- [ ] FPS 60 이상 유지
- [ ] `roarm_m3_v2.usd` 저장 완료

---

## 🔍 문제 해결 (Troubleshooting)

### 문제 1: Joint가 떨린다
**원인**: Stiffness가 너무 높거나 Damping이 부족  
**해결**:
```python
# Property → PhysicsDriveAPI
stiffness: 5000 → 2000 (낮추기)
damping: 200 → 500 (높이기)
```

### 문제 2: Joint가 느리게 움직인다
**원인**: Stiffness가 너무 낮거나 Damping이 과다  
**해결**:
```python
stiffness: 2000 → 5000 (높이기)
damping: 500 → 300 (낮추기)
```

### 문제 3: 로봇이 바닥으로 떨어진다
**원인**: Fix Base가 적용 안됨  
**해결**:
1. Stage에서 `roarm_m3` prim 선택
2. Property → ArticulationRootAPI
3. `fixedBase = true` 확인
4. 안되면 base_link에 직접 `FixedJoint` 추가 (world 연결)

### 문제 4: Continuous joint가 제한됨
**원인**: Import 시 자동 변환  
**해결**:
1. Stage에서 해당 joint 선택 (joint_1, joint_5)
2. Property → RevoluteJoint
3. Lower Limit: -10.0 (넉넉하게)
4. Upper Limit: +10.0
5. 또는 `hasLimits = false` 체크

### 문제 5: 축 방향이 이상하다
**원인**: URDF axis와 USD axis 불일치  
**해결**:
1. URDF에서 `<axis xyz="..."/>` 확인
2. USD Property → PhysicsRevoluteJoint → axis 수동 조정
3. 또는 URDF 수정 후 재 Import

### 문제 6: 관성 이상 (폭주/진동)
**원인**: URDF inertia 값 비현실적  
**해결**:
1. Re-import with "Compute Inertia from Collision" 옵션
2. 또는 각 link의 MassAPI에서 수동 조정:
   ```python
   mass: 0.3
   inertia diagonal: (0.001, 0.001, 0.0005)  # 대칭·양수
   ```

---

## 📊 예상 결과

### URDF 구조 (입력)
```
11 links, 10 joints
- world (fixed)
- base_link
- link_1 (continuous, Z축)
- link_2 (revolute, Y축, dual-drive)
- link_3 (revolute, Y축)
- link_4 (revolute, Y축)
- link_5 (continuous, X축)
- gripper_base (revolute, Z축)
- gripper_left_finger (fixed)
- gripper_right_finger (fixed)
- tcp (fixed)
```

### USD 구조 (출력 - Merge OFF)
```
11 links, 10 joints (동일)
ArticulationRootAPI: /World/roarm_m3
PhysicsScene: /World/physicsScene
```

### USD 구조 (출력 - Merge ON)
```
7-8 links, 6 joints
- world + base_link 병합
- gripper_base + fingers + tcp 병합
ArticulationRootAPI: /World/roarm_m3
```

---

## 🚀 다음 단계

Phase 1 & 2 완료 후:

1. **USD 검증**:
   ```bash
   # 터미널에서
   usdview assets/roarm_m3/usd/roarm_m3_v2.usd
   ```

2. **Python 스크립트 테스트**:
   ```python
   from isaacsim.core.api import World
   world = World()
   world.scene.add_default_ground_plane()
   robot = world.scene.add(
       ArticulationView(
           prim_paths_expr="/World/roarm_m3",
           name="roarm_m3"
       )
   )
   world.reset()
   ```

3. **RL 환경 구축**: 
   - 간단한 pick-and-place 태스크
   - PPO 학습 시작

---

**작성**: GitHub Copilot (Jarvis)  
**검증**: Phase 1 → Phase 2 순차 진행 필수  
**소요 시간**: Phase 1: 10-15분, Phase 2: 5-10분
