# RoArm M3 STEP → STL 변환 가이드 (FreeCAD GUI)

## 🎯 목표
RoArm-M3.step 파일을 개별 링크별 STL로 분리

## 📋 필요 파일
- **입력**: `resources/roarm_m3/RoArm-M3.step` (24MB)
- **출력**: `assets/roarm_m3/meshes/visual/*.stl` (링크별 7-9개 파일)

---

## 🔧 FreeCAD GUI 사용법

### Step 1: FreeCAD 실행
```bash
freecad
```

### Step 2: STEP 파일 열기
1. **File → Open**
2. 파일 선택: `resources/roarm_m3/RoArm-M3.step`
3. **OK** 클릭
4. 로딩 대기 (1-2분)

### Step 3: 파트 구조 확인
왼쪽 **Model 트리**에서 파트 목록 확인:
```
RoArm-M3
├─ Base (베이스)
├─ Link1 (회전축)
├─ Link2 (어깨/Shoulder)
├─ Link3 (팔꿈치/Elbow)
├─ Link4 (손목1)
├─ Link5 (손목2)
├─ Gripper_Base (그리퍼 베이스)
├─ Gripper_Left (그리퍼 왼쪽 핑거)
├─ Gripper_Right (그리퍼 오른쪽 핑거)
└─ (기타 볼트/너트 등)
```

### Step 4: 개별 파트 STL 저장

**각 주요 파트에 대해 반복**:

#### 4.1 파트 선택
- Model 트리에서 파트 **하나** 선택 (예: Base)
- 3D 뷰에서 강조 표시 확인

#### 4.2 Export
1. **File → Export**
2. 파일 형식: **STL mesh (*.stl)** 선택
3. 저장 위치: `assets/roarm_m3/meshes/visual/`
4. 파일명 규칙:
   ```
   base_link.stl
   link_1.stl
   link_2.stl
   link_3.stl
   link_4.stl
   link_5.stl
   gripper_base.stl
   gripper_left_finger.stl
   gripper_right_finger.stl
   ```
5. **Save** 클릭

#### 4.3 Export 옵션
- **ASCII vs Binary**: Binary 선택 (파일 크기 작음)
- **Deviation**: 0.1 (기본값, 정밀도)
- **OK** 클릭

### Step 5: 전체 파트 Export 반복
Step 4를 **각 링크**에 대해 반복 (7-9회)

---

## 📊 예상 결과

```bash
assets/roarm_m3/meshes/visual/
├─ base_link.stl           (~10-20 MB)
├─ link_1.stl              (~5-10 MB)
├─ link_2.stl              (~15-25 MB, dual-drive)
├─ link_3.stl              (~10-15 MB)
├─ link_4.stl              (~5-10 MB)
├─ link_5.stl              (~3-5 MB)
├─ gripper_base.stl        (~5-10 MB)
├─ gripper_left_finger.stl (~1-3 MB)
└─ gripper_right_finger.stl (~1-3 MB)
```

**총 크기**: ~60-120 MB (압축 전)

---

## ✅ 완료 후 확인

```bash
cd ~/roarm_isaac_clean
ls -lh assets/roarm_m3/meshes/visual/
```

**체크리스트**:
- [ ] 9개 STL 파일 생성됨
- [ ] 각 파일이 1MB 이상 (비어있지 않음)
- [ ] 파일명이 URDF 규칙과 일치

---

## 🚨 주의사항

### 파트명이 다를 경우
STEP 파일의 파트명이 예상과 다를 수 있습니다:
- **실제 파트명 확인** 후 기록
- URDF 파일에서 동일한 이름 사용

### 불필요한 파트 제외
- 볼트, 너트, 와셔 등 작은 부품은 스킵
- 주요 구조 파트만 Export (7-9개)

### 파일 크기가 너무 클 경우
나중에 Mesh 간소화 단계에서 최적화:
```bash
# MeshLab으로 간소화 (collision용)
meshlab roarm_m3_complete.stl
Filters → Remeshing → Simplification → Quadric Edge Collapse
Target: 10,000 faces (visual은 100,000)
```

---

## 🔄 대안: Python 스크립트로 자동 분리

만약 FreeCAD GUI가 불편하면 Python 스크립트 작성 가능:
```python
# scripts/split_step_parts.py
import cadquery as cq

# STEP 파일 로드
assembly = cq.importers.importStep("resources/roarm_m3/RoArm-M3.step")

# 각 파트 분리 및 저장 (assembly 구조 분석 필요)
for part in assembly.parts:
    cq.exporters.export(part, f"assets/roarm_m3/meshes/visual/{part.name}.stl")
```

단, STEP 파일의 assembly 구조를 먼저 파악해야 함.

---

## ⏭️ 다음 단계

STL 파일 생성 완료 후:
1. **Collision mesh 생성**: Visual mesh를 간소화 (10,000 faces)
2. **URDF 업데이트**: Primitive geometry → mesh 경로로 변경
3. **Isaac Sim Import**: Mesh 포함 URDF 테스트
4. **USD 저장**: 최종 USD 파일 생성

---

**작성**: 2025-10-18  
**소요 시간**: 30분 (FreeCAD GUI) 또는 1시간 (Python 자동화)
