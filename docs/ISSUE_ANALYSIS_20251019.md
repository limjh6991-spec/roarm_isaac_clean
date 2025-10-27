# 이슈 분석 보고서 - 2025년 10월 19일

## �� 핵심 문제: 그리퍼 미작동

### 증상
- **100K steps 학습 결과**
  - REACH 마일스톤: 12회 달성 ✅
  - GRIP 마일스톤: 0회 ❌
  - LIFT 마일스톤: 0회 ❌
  - GOAL 마일스톤: 0회 ❌
  - SUCCESS: 0회 ❌
  
- **ep_rew_mean**: -5.66 (정체)

### 근본 원인

#### 1. URDF 그리퍼 조인트 문제
```
로그 출력: "Joints (8): ['joint_1', 'joint_2', 'joint_3']..."
문제: 8개 조인트인데 3개만 출력됨
원인: 그리퍼 조인트가 제대로 정의되지 않았거나 Fixed 타입
```

#### 2. 현재 URDF 추정 구조
```
joint_1, joint_2, joint_3: 로봇팔 관절 (정상 작동)
joint_4, joint_5, joint_6: ??? (미확인)
gripper_left, gripper_right: Fixed 또는 누락
```

#### 3. 필요한 구조
```
joint_1~6: 로봇팔 6 DoF
gripper_left_joint: Prismatic (0~25mm)
gripper_right_joint: Prismatic (0~25mm, 반대 방향)
```

---

## 📊 학습 진행 타임라인

### Phase 0: Dense Reward (실패)
- 50K steps
- ep_rew_mean: +916 (폭발)
- EV: 0.00006 (붕괴)
- **원인**: 개선 보상 누적

### Phase 1: Sparse Reward (안정화)
- 100K steps
- ep_rew_mean: -6.01 (안정)
- EV: 0.283 (4,717배 개선)
- **성과**: 학습 안정화

### Phase 2: Shaped-Sparse + Curriculum (현재)
- 100K steps
- ep_rew_mean: -5.66 (소폭 개선)
- REACH: 12회 달성
- **문제**: GRIP 이후 진전 없음

---

## 🔍 디버깅 로그 분석

### 로봇 초기화 로그
```
✅ 로봇 임포트 성공!
  📍 Prim path: /roarm_m3_multiprim/root_joint
  ⏳ Articulation 초기화 중...
  ✅ Joints (8): ['joint_1', 'joint_2', 'joint_3']...  ← 문제!
  ⏳ Joint drive 설정 중...
  ✅ Joint drive 설정 완료! (완화된 값)
```

**문제점:**
- 8개 조인트 선언했지만 3개만 출력
- 나머지 5개는 어디로?
- 그리퍼 조인트 초기화 실패 가능성

### 환경 정보
```
📊 환경 정보:
  - Observation dim: 25
  - Action dim: 8  ← 그리퍼 포함
  - Max steps: 600
```

**모순:**
- Action dim은 8 (그리퍼 포함)
- 하지만 그리퍼 조인트 미작동
- **추론**: Action은 받지만 실제 제어 안됨

---

## 💡 해결 방안

### 즉시 조치 (내일 오전)
1. **URDF 파일 분석**
   ```bash
   grep -E "joint.*gripper|joint_[0-9]" assets/roarm_m3/urdf/roarm_m3_multiprim.urdf
   ```

2. **그리퍼 조인트 타입 확인**
   - Fixed → Prismatic으로 변경
   - Joint limits 추가
   - Axis 방향 설정

3. **검증**
   ```bash
   ~/isaacsim/python.sh scripts/verify_urdf.py
   ```

### 중기 조치 (내일 오후)
1. **환경 코드 수정**
   - 그리퍼 조인트 인덱스 확인
   - grasp_valid 조건 재검토

2. **50K 테스트 학습**
   - GRIP 마일스톤 1회 이상 목표

3. **GUI 테스트**
   - 그리퍼 개폐 동작 확인

---

## 📈 예상 성과

### URDF 수정 성공 시
```
50K steps 후:
- REACH: 지속 달성
- GRIP: 10+ 회 ✅
- LIFT: 1~5회
- ep_rew_mean: -4.0 이상
```

### 장기 학습 계획
```
1M steps (약 30분):
- 모든 마일스톤 달성
- Phase 1 Normal Mode 전환
- 최종 Success 달성
```

---

## ⚠️ 학습된 교훈

### 잘한 점
1. ✅ Dense → Sparse 전환으로 안정화
2. ✅ Shaped-Sparse + Curriculum 설계
3. ✅ 체계적인 로깅 및 모니터링

### 개선 필요
1. ❌ **초기 URDF 검증 부족**
   - 그리퍼 동작 테스트 안 함
   - 조인트 개수만 확인

2. ❌ **환경 검증 불충분**
   - Action이 실제 제어되는지 확인 안 함
   - Contact sensor 미구현

3. ❌ **너무 빠른 장시간 학습**
   - 50K로 충분히 검증 가능했음
   - 10M 계획은 시기상조

---

## 🎯 다음 단계

### 우선순위 1: URDF 수정 (내일 필수)
- [ ] 그리퍼 조인트 Prismatic 타입 설정
- [ ] Joint limits 추가 (0~25mm)
- [ ] Isaac Sim 임포트 검증

### 우선순위 2: 짧은 검증 (내일 오후)
- [ ] 50K steps 테스트 학습
- [ ] GRIP 마일스톤 1회 이상 달성 확인

### 우선순위 3: 장기 계획 (다음 주)
- [ ] 1M steps 학습
- [ ] Phase 1 전환
- [ ] 최종 성공률 50% 이상

---

**작성일**: 2025-10-19  
**작성자**: RL Engineer  
**다음 리뷰**: 2025-10-20 18:00
