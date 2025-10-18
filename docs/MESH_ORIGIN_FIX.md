# Mesh Origin 조정 가이드

## 문제:
FreeCAD에서 export한 STL 파일들의 중심점이 (0,0,0)에 있지 않아서
Isaac Sim에 import했을 때 파트들이 흩어져 보입니다.

## 해결 방법 1: Primitive로 돌아가기 (추천) ⭐

**이유:**
- Mesh 매칭이 매우 어려움 (10,000개 파트 중 올바른 파트 찾기)
- Origin 조정에 시간이 많이 소요됨
- Primitive geometry로도 Physics/RL 학습 가능

**진행:**
1. 이전 primitive URDF 사용
2. Physics/Joint 테스트 완료
3. RL 환경 구축 시작
4. 필요시 나중에 mesh 작업

## 해결 방법 2: STL 파일 중심점 수정

**MeshLab 사용:**
```bash
# MeshLab 설치
sudo apt install meshlab

# 각 STL 파일 열기
meshlab assets/roarm_m3/meshes/visual/base_link.stl

# Filters → Normals, Curvatures and Orientation → Transform: Move, Translate, Center
# "Center on Scene BBox" 선택 → Apply
# File → Export Mesh → 같은 이름으로 저장
```

각 6개 파일에 반복.

## 해결 방법 3: URDF origin 수동 조정

**시행착오 방법:**
1. Isaac Sim에서 각 link의 위치 확인
2. URDF에서 visual origin xyz 값 조정
3. 다시 import
4. 반복...

매우 시간이 많이 걸림 (1-2시간).

## 🎯 추천:

**Phase 1 (Primitive)로 돌아가서:**
1. Physics/Joint 동작 확인
2. USD 저장
3. RL 환경 구축 시작
4. **Phase 2는 나중에** (발표/데모 필요시)

**Mesh는 선택사항이지 필수가 아닙니다!**
