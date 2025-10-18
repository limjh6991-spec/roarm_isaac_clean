# RoArm-M3 URDF 표준화 가이드
# 작성일: 2025-10-18
# 최종 업데이트: 2025-10-18 14:30
# 목표: Isaac Sim에서 완벽히 작동하는 URDF 생성
# 현재 상태: 🟡 STL Placement 베이크 문제 해결 필요

## ⚠️ 알려진 문제 (2025-10-18)

**STL 파일 Placement 미베이크 문제**
- **증상**: Isaac Sim에서 로봇 링크들이 올바른 위치에 조립되지 않고 분산되어 표시됨
- **원인**: FreeCAD 매크로로 생성된 STL 파일에 Placement가 베이크되지 않음
- **영향**: URDF joint origin과 STL 내부 offset 중복으로 위치 계산 오류
- **상태**: 확인됨, 해결 작업 필요
- **참고**: `logs/urdf_standardization_20251018.md` 참조

---

## 📋 체크리스트 (10분 완주)

- [ ] STL 링크별 베이크 (FreeCAD 매크로)
- [ ] URDF 자동 패치 (스크립트)
- [ ] Isaac Sim 검증 임포트
- [ ] 최종 저장 및 문서화

---

## Step 1: 링크별 STL 생성 (5분)

### 1-1. 환경 변수 설정
```bash
export ROARM_EXPORT_DIR=/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/meshes
export ROARM_FCSTD=/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/meshes/visual/roarm_m3.FCStd
```

### 1-2. FreeCAD GUI 실행
```bash
/snap/bin/freecad "$ROARM_FCSTD" &
```

### 1-3. 매크로 실행
1. **Macro → Macros** 메뉴 열기
2. **BatchExport_STL_ByLink_v2** 선택
3. **Execute** 버튼 클릭
4. 진행 상황 확인:
   - 콘솔에 "Exporting base_link..." 등 메시지 출력
   - `$ROARM_EXPORT_DIR/visual/` 폴더에 STL 생성

### 1-4. 결과 검증
```bash
ls -lh assets/roarm_m3/meshes/visual/*.stl
ls -lh assets/roarm_m3/meshes/collision/*.stl
```

**예상 출력 (9개 파일):**
```
base_link.stl
link_1.stl ~ link_5.stl
gripper_base.stl
gripper_left_finger.stl
gripper_right_finger.stl
```

---

## Step 2: URDF 자동 패치 (2분)

### 2-1. 패처 실행
```bash
python scripts/urdf_autopatch_standard.py \
    --input assets/roarm_m3/urdf/roarm_m3_v3_transformed.urdf \
    --output assets/roarm_m3/urdf/roarm_m3_standard.urdf \
    --mesh-dir meshes
```

### 2-2. 패치 내용 확인
패처가 자동으로 처리하는 것:
- ✅ 모든 `<visual>/<collision>`을 링크별 STL로 교체
- ✅ `<origin xyz="0 0 0" rpy="0 0 0">` 강제
- ✅ `scale="1 1 1"` 강제 (STL에 이미 베이크됨)
- ✅ `<inertial>` 자동 계산 (충돌 프리미티브 기반)

### 2-3. 결과 검증
```bash
# origin이 모두 0인지 확인
grep "<origin" assets/roarm_m3/urdf/roarm_m3_standard.urdf | head -15

# scale이 모두 1인지 확인
grep "scale=" assets/roarm_m3/urdf/roarm_m3_standard.urdf

# 링크 수 확인 (7개여야 함)
grep "<link name=" assets/roarm_m3/urdf/roarm_m3_standard.urdf
```

---

## Step 3: Isaac Sim 검증 임포트 (3분)

### 3-1. Isaac Sim 실행
```bash
~/isaac-sim.sh &
```

### 3-2. URDF 임포트
1. **File → Import → URDF**
2. 선택: `assets/roarm_m3/urdf/roarm_m3_standard.urdf`
3. Import Options:
   - ✅ **Fix Base**: TRUE
   - ⚠️ **Merge Fixed Joints**: FALSE (1차 검증)
   - ❌ **Self Collision**: FALSE
   - ✅ **Import Inertia Tensor**: TRUE

### 3-3. 디버그 체크리스트

**시각적 확인:**
- [ ] 모든 링크가 보임 (base + 5개 링크 + 3개 gripper)
- [ ] 링크가 겹치지 않고 올바른 위치에 배치
- [ ] 스케일이 현실적 (전체 높이 ~50cm)

**조인트 확인:**
- [ ] ArticulationRoot 1개 (base_link)
- [ ] Revolute/Prismatic 조인트 6개
- [ ] 각 조인트 슬라이더 조작 시 링크가 1:1로 따라옴

**바운딩 박스 확인:**
```python
# Isaac Sim Python Console에서
import omni.usd
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/roarm_m3/base_link")
bbox = prim.GetAttribute("xformOp:transform").Get()
print(bbox)  # 스케일 mm→m 변환 OK인지 확인
```

### 3-4. 문제 발생 시

**증상 1: 링크가 이상한 위치에 배치**
→ STL에 Placement 베이크 안 됨
→ Step 1 다시 실행 (FreeCAD 매크로 재실행)

**증상 2: 링크가 너무 작거나 큼**
→ scale 중복/누락
→ URDF에서 `scale="1 1 1"` 확인

**증상 3: 일부 링크만 보임**
→ STL 파일 누락
→ `ls -lh meshes/visual/` 확인

---

## Step 4: 최종 저장 및 검증 (선택)

### 4-1. Merge Fixed Joints (선택)
Step 3-3에서 모든 체크 통과 시:
1. URDF 재임포트
2. **Merge Fixed Joints: TRUE** 설정
3. 재검증

### 4-2. USD 저장
```python
# Isaac Sim에서
import omni.usd
stage = omni.usd.get_context().get_stage()
stage.GetRootLayer().Export("assets/roarm_m3/usd/roarm_m3_articulated.usda")
```

### 4-3. 문서화
```bash
# README에 추가
echo "## URDF 표준화 완료
- 입력: roarm_m3_v3_transformed.urdf
- 출력: roarm_m3_standard.urdf
- 검증: Isaac Sim 5.0.0-rc.45 임포트 성공
- 날짜: $(date +%Y-%m-%d)
" >> docs/urdf_standard_log.md
```

---

## 🔧 대안: 방법 A (FreeCAD Assembly4 재수출)

**Step 3에서 실패 시** (링크 오프셋 여전히 존재, 스케일 여전히 틀림):

### A-1. FreeCAD Assembly4 설치
```bash
# FreeCAD → Tools → Addon Manager
# 검색: Assembly4 → Install
```

### A-2. LCS(Local Coordinate System) 배치
1. 각 링크에 **LCS** 추가 (Part → Local Coordinate System)
2. LCS를 조인트 회전축 또는 링크 중심에 배치
3. 모든 파트의 Placement를 LCS 기준으로 "베이크"

### A-3. A4URDF Export
1. **File → Export → A4URDF**
2. 설정:
   - Unit: **Meter**
   - Up Axis: **Z**
   - Scale: **1.0** (베이크했으니까)
3. Export

### A-4. Isaac Sim 재검증
Step 3-2부터 재실행

---

## 📊 성공 기준

| 항목 | 기준 | 검증 방법 |
|------|------|-----------|
| 링크 수 | 7개 (world 제외) | `grep "<link name=" \| wc -l` |
| Visual Origin | 모두 [0,0,0] | `grep visual -A2 \| grep origin` |
| Scale | 모두 1 1 1 | `grep scale=` |
| 스케일 크기 | 전체 높이 ~0.5m | Isaac Sim bbox 확인 |
| 조인트 작동 | 슬라이더↔링크 1:1 | GUI 조작 |
| 충돌 메시 | 9개 STL 존재 | `ls collision/*.stl \| wc -l` |

---

## ⚠️ 주의사항

1. **FreeCAD 매크로 실행 시 GUI 필수**
   - `--console` 모드는 REPL로 전환되어 실패
   - 반드시 GUI에서 Macro 메뉴 사용

2. **STL 베이크 확인**
   ```bash
   # STL 내부에 변환이 베이크됐는지 확인
   python scripts/check_stl_transform.py meshes/visual/base_link.stl
   ```

3. **Isaac Sim 임포트 옵션**
   - 첫 검증: Merge=FALSE (개별 링크 확인)
   - 최종: Merge=TRUE (성능 최적화)

4. **경로 일관성**
   - URDF `filename="../meshes/visual/..."` (상대경로)
   - Isaac Sim는 URDF 파일 위치 기준으로 해석

---

## 🎯 예상 소요 시간

- Step 1 (STL 생성): **5분**
- Step 2 (패치): **2분**
- Step 3 (검증): **3분**
- **총 10분** ✅

실패 시 방법 A: **2~4시간** (모델링 재작업)
