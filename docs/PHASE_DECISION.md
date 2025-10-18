# Phase 1 vs Phase 2 결정 가이드

## 현재 상황 (2025-10-18)

### ✅ Phase 1 완료 상태
- [x] URDF (primitive cylinder) 생성
- [x] Isaac Sim import 성공
- [ ] Physics/Joint 테스트 진행 중

### 🤔 Phase 2 진행 여부 결정

## Option A: Phase 1만 완료 후 RL 환경 구축 (추천)

**예상 소요 시간**: 1-2시간
**장점**:
- 빠른 프로토타이핑
- Physics 검증에 집중
- RL 학습 바로 시작 가능

**진행 순서**:
1. Isaac Sim Physics 테스트 (10분)
2. USD 저장 (5분)
3. Python API로 Joint 제어 스크립트 작성 (30분)
4. RL 환경 구축 (1시간)
5. 간단한 학습 테스트 (30분)

**Phase 2는 언제?**
- RL 학습 성공 후
- 데모/발표 준비할 때
- 논문 작성 시

---

## Option B: Phase 2까지 완료 (실제 mesh)

**예상 소요 시간**: 2-3시간
**장점**:
- 실제 로봇 외형
- 데모용 비주얼 완성
- Vision-based RL 가능

**진행 순서**:
1. Isaac Sim Physics 테스트 (10분)
2. FreeCAD STL 분리 작업 (30-60분)
3. URDF 업데이트 (20분)
4. Isaac Sim 재import 및 테스트 (20분)
5. USD 저장 (5분)
6. RL 환경 구축 (1시간)

---

## 💡 추천: Option A

**이유**:
1. 현재 목표는 "Isaac Sim에서 RoArm-M3 동작시키기"
2. RL 학습은 primitive로도 충분
3. 시간 절약 (Phase 2는 1시간 이상 소요)
4. 필요시 나중에 Phase 2 진행 가능

**Phase 2가 꼭 필요한 경우만 진행**:
- Vision-based RL (카메라 이미지 사용)
- 논문/발표용 시뮬레이션 영상
- 복잡한 그리퍼 작업 (정밀한 collision)

---

## 🎯 결정

- [ ] **Option A**: Phase 1만 완료 → RL 환경 바로 구축
- [ ] **Option B**: Phase 2까지 완료 → 실제 mesh 사용

**선택 후 진행하세요!**
