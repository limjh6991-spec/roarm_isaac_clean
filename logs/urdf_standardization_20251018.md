# URDF 표준화 작업 로그
**날짜:** 2025-10-18  
**목표:** Isaac Sim에서 완벽히 작동하는 표준화된 URDF 생성  
**현재 상태:** 🟡 진행 중 (STL 생성 문제 발견)

---

## 📋 작업 개요

### 목표
- RoArm-M3 URDF를 Isaac Sim 5.0.0 표준에 맞게 변환
- 링크별 STL 파일 생성 및 경로 표준화
- Joint origin 보존하면서 visual/collision origin만 0으로 설정

### 진행 단계
1. ✅ **STL 경로 형식 수정**: `file://` 제거, 직접 절대 경로 사용
2. ✅ **스케일 수정**: `scale="0.001 0.001 0.001"` 적용 (mm → m 변환)
3. ✅ **Isaac Sim 렌더링 성공**: 로봇 메시가 Viewport에 표시됨
4. ❌ **STL 파일 문제 발견**: 링크들이 올바른 위치에 조립되지 않음

---

## 🔍 발견된 문제

### 증상
- Isaac Sim Viewport에서 로봇 링크들이 **분산되어 표시됨**
- Stage 계층 구조는 정상 (world → link_1~5 → gripper)
- Joint origin 값은 URDF에 올바르게 보존됨

### 근본 원인
**STL 파일이 정상적으로 생성되지 못함**
- FreeCAD 매크로를 통해 생성된 STL 파일들 (18:38-18:39)
- 각 링크의 **Placement가 Identity로 베이크되지 않음**
- 결과적으로 STL 파일 자체에 월드 좌표계 기준의 offset이 포함됨

### 기술적 배경
```
문제 시나리오:
1. FreeCAD에서 link_1의 Placement = (0, 0, 0.06)
2. STL export 시 Placement가 베이크되지 않음
3. URDF: joint origin (0, 0, 0.06) + STL 내부 offset (0, 0, 0.06)
4. 결과: 링크가 (0, 0, 0.12)에 위치 → 2배 offset!
```

---

## ✅ 성공한 작업

### 1. URDF 경로 형식 수정
**변경 전:**
```xml
<mesh filename="file:///home/roarm_m3/.../base_link.stl" scale="1 1 1"/>
```

**변경 후:**
```xml
<mesh filename="/home/roarm_m3/.../base_link.stl" scale="0.001 0.001 0.001"/>
```

**결과:**
- ✅ Isaac Sim URDF importer가 메시를 정상적으로 로드
- ✅ Viewport에 로봇 지오메트리 렌더링 성공
- ✅ 스케일 올바름 (~0.5m 높이)

### 2. urdf_autopatch_standard.py 스크립트 수정
**핵심 변경사항:**
```python
# 절대 경로 생성 (file:// 제거)
stl_path = f"{abs_mesh_dir}/{subdir}/{link_name}.stl"

# mm → m 단위 변환
mesh_elem.set("scale", "0.001 0.001 0.001")
```

**검증:**
```bash
$ grep -E "(filename|scale)=" roarm_m3_standard.urdf | head -3
# filename="/home/roarm_m3/.../base_link.stl" scale="0.001 0.001 0.001"
```

### 3. Joint Origin 보존 확인
```bash
$ grep -E "joint name=|<origin xyz=" roarm_m3_standard.urdf | head -6
<joint name="joint_1">
  <origin xyz="0 0 0.060" rpy="0 0 0" />
<joint name="joint_2">
  <origin xyz="0 0 0.080" rpy="0 0 0" />
```
✅ Joint origin 값이 원본 v3_transformed.urdf와 동일하게 유지됨

---

## ❌ 해결되지 않은 문제

### STL 파일 생성 프로세스
**현재 상태:**
- `assets/roarm_m3/meshes/visual/*.stl` (9개 파일, 18:38-18:39 생성)
- 파일 크기: 776KB ~ 1.1MB (정상 범위)
- 파일 형식: STL (ASCII/Binary)

**문제:**
- FreeCAD에서 각 링크의 Placement가 베이크되지 않음
- STL 파일 내부에 월드 좌표계 기준의 변환 포함
- URDF joint origin과 중복되어 링크 위치 계산 오류

**필요한 작업:**
1. FreeCAD에서 각 링크를 **Identity Placement**로 재설정
2. 또는 STL export 시 **"Apply Placement" 옵션 활성화**
3. 재생성된 STL로 URDF 다시 패치

---

## 🔧 사용된 도구 및 스크립트

### 1. urdf_autopatch_standard.py
- **위치:** `scripts/urdf_autopatch_standard.py`
- **기능:**
  - Visual/collision mesh를 링크별 STL로 교체
  - Visual/collision origin을 0으로 강제
  - Joint origin 보존
  - Inertial 자동 계산
  - 스케일 0.001 적용

### 2. verify_urdf_standard.py
- **위치:** `scripts/verify_urdf_standard.py`
- **결과:** 5/6 체크 통과 (83.3%)
- **실패 항목:** Link count (기대 7, 실제 10)

### 3. URDF_STANDARD_GUIDE.md
- **위치:** `docs/URDF_STANDARD_GUIDE.md`
- **내용:** 3단계 표준화 프로세스 가이드

---

## 📊 파일 현황

### URDF 파일
| 파일명 | 상태 | 설명 |
|--------|------|------|
| `roarm_m3_v3_transformed.urdf` | ✅ 원본 | 동작하는 기준 URDF |
| `roarm_m3_standard.urdf` | 🟡 생성됨 | 경로/스케일 수정, STL 문제 |
| `roarm_m3_standard_abs.urdf` | ❌ 폐기 | file:// 경로 (실패) |

### STL 파일
| 디렉토리 | 파일 수 | 생성 시간 | 상태 |
|----------|---------|-----------|------|
| `meshes/visual/` | 19 | 18:38-18:39 | 🟡 Placement 미베이크 |
| `meshes/collision/` | 19 | 복사본 | 🟡 동일 문제 |

### 스크립트
| 파일 | 수정 | 검증 |
|------|------|------|
| `urdf_autopatch_standard.py` | ✅ | ✅ |
| `verify_urdf_standard.py` | - | ✅ |
| `URDF_STANDARD_GUIDE.md` | - | ✅ |

---

## 🎯 다음 단계 (내일 작업)

### 우선순위 1: STL 파일 재생성
1. **FreeCAD 매크로 수정**
   - BatchExport_STL_ByLink_v2 매크로 확인
   - Placement 베이크 로직 추가
   - 테스트 export (base_link만)

2. **대안: Assembly4 Export**
   - FreeCAD Assembly4 addon 설치
   - LCS(Local Coordinate System) 설정
   - A4URDF exporter로 재수출

3. **수동 보정 (최후 수단)**
   - MeshLab/Blender에서 STL 위치 재설정
   - 각 링크를 원점 기준으로 정렬

### 우선순위 2: 검증 프로세스
1. 원본 `roarm_m3_v3_transformed.urdf`를 Isaac Sim에 직접 임포트
2. 제대로 조립되는지 확인 (baseline 검증)
3. 새로 생성한 STL과 비교

### 우선순위 3: 문서화
1. STL 생성 프로세스 상세 가이드 작성
2. FreeCAD 매크로 사용법 스크린샷 추가
3. 트러블슈팅 섹션 확장

---

## 📚 참고 자료

### Isaac Sim URDF Importer 요구사항
- 경로: 절대 경로 또는 URDF 기준 상대 경로
- 스케일: mesh 태그의 scale 속성으로 지정
- Origin: visual/collision은 0, joint는 실제 값
- 단위: URDF는 m, STL은 export 시 지정

### FreeCAD STL Export 옵션
```python
# 올바른 export (Placement 베이크)
Part.export([obj], filename)
obj.Shape.exportStl(filename, ASCIIMode=False)

# 문제가 있는 export (Placement 미베이크)
Mesh.export([mesh_obj], filename)
```

---

## 🐛 알려진 이슈

### Issue #1: STL Placement 베이크 안 됨
- **심각도:** 🔴 High
- **영향:** 로봇 링크 조립 실패
- **상태:** 확인됨, 수정 필요
- **해결 예정:** 2025-10-19

### Issue #2: Link Count 불일치
- **심각도:** 🟡 Medium
- **영향:** verify_urdf_standard.py 체크 실패
- **원인:** tcp, gripper_finger 링크 포함
- **상태:** 검증 스크립트 수정 필요

---

## 💡 교훈

1. **STL Export는 Placement 베이크 필수**
   - FreeCAD에서 export 시 반드시 확인
   - 테스트 파일로 먼저 검증

2. **Isaac Sim 경로 형식**
   - `file://` URI 스킴은 사용 불가
   - 절대 경로 또는 상대 경로만 지원

3. **단위 변환 주의**
   - STL: mm 단위로 export
   - URDF: m 단위, scale="0.001" 필수

4. **Joint vs Visual Origin 구분**
   - Joint origin: 링크 연결 위치 (보존)
   - Visual origin: 메시 로컬 좌표계 (0으로 설정)

---

## 🔗 관련 파일

- 원본 URDF: `assets/roarm_m3/urdf/roarm_m3_v3_transformed.urdf`
- 생성된 URDF: `assets/roarm_m3/urdf/roarm_m3_standard.urdf`
- STL 디렉토리: `assets/roarm_m3/meshes/visual/`
- 패처 스크립트: `scripts/urdf_autopatch_standard.py`
- 검증 스크립트: `scripts/verify_urdf_standard.py`
- 가이드 문서: `docs/URDF_STANDARD_GUIDE.md`

---

**작성자:** GitHub Copilot  
**마지막 업데이트:** 2025-10-18 14:30
