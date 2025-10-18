# FreeCAD STL Export 체크리스트

## 🎯 찾아야 할 주요 파트 (중국어 이름)

### 1. Base Link (베이스)
**찾을 이름**: 
- `舵机型材底座-舵机安装座 v1` (서보 마운트 베이스)
- 또는 `BASE` 

**Export 이름**: `base_link.stl`

---

### 2. Link 1 (회전 축)
**찾을 이름**:
- `LEG-HOLDER v1` 또는 `LEG-HOLDER v002`
- 또는 베이스 바로 위 회전 부분

**Export 이름**: `link_1.stl`

---

### 3. Link 2 (어깨/Shoulder, 긴 팔)
**찾을 이름**:
- `AL-ELBOW-B v1` (알루미늄 팔꿈치 연결부)
- `碳纤维管，内14外16-70 v2` (탄소섬유 튜브)
- 또는 LEG-HOLDER v003~v006 중 하나

**Export 이름**: `link_2.stl`

---

### 4. Link 3 (팔꿈치/Elbow)
**찾을 이름**:
- `AL-ELBOW-B` 관련 부분
- 또는 탄소섬유 튜브 다음 연결부

**Export 이름**: `link_3.stl`

---

### 5. Link 4 (손목 1)
**찾을 이름**:
- `手腕a v2 v1` (손목 a)

**Export 이름**: `link_4.stl`

---

### 6. Link 5 (손목 2)
**찾을 이름**:
- `手腕b v1` (손목 b)

**Export 이름**: `link_5.stl`

---

### 7. Gripper Base (그리퍼 베이스)
**찾을 이름**:
- 그리퍼 관련 부품 (손목 끝 부분)
- 이름에 "gripper", "claw", "夹爪" 포함

**Export 이름**: `gripper_base.stl`

---

### 8. Gripper Left Finger (왼쪽 핑거)
**찾을 이름**:
- 그리퍼 왼쪽 파츠

**Export 이름**: `gripper_left_finger.stl`

---

### 9. Gripper Right Finger (오른쪽 핑거)
**찾을 이름**:
- 그리퍼 오른쪽 파츠

**Export 이름**: `gripper_right_finger.stl`

---

## 📝 Export 절차 (각 파트마다 반복)

### 1. 파트 선택
- FreeCAD 왼쪽 Model 트리에서 파트 클릭
- 3D 뷰에서 강조되는지 확인

### 2. Export
1. **File → Export** (또는 Ctrl+E)
2. **파일 형식**: `STL mesh (*.stl)` 선택
3. **저장 위치**: `/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/meshes/visual/`
4. **파일명**: 위 체크리스트의 "Export 이름" 사용
5. **Export 옵션**:
   - Format: **Binary** (파일 크기 작음)
   - Deviation: **0.1** (기본값)
6. **Save** 클릭

### 3. 다음 파트로 이동
- 체크리스트에서 체크 ✅
- 다음 파트 선택
- 1번부터 반복

---

## ⚠️ 주의사항

### 여러 파트가 조합된 경우
- **Ctrl** 키를 누르고 여러 파트 선택
- 함께 Export

### 파트 이름이 다른 경우
- 3D 뷰에서 **어느 부분인지 확인**
- 베이스부터 그리퍼까지 순서대로 진행

### 파트를 못 찾는 경우
- 일단 찾은 파트만 Export
- 나중에 URDF에서 primitive로 대체

---

## ✅ 완료 체크리스트

- [ ] base_link.stl
- [ ] link_1.stl
- [ ] link_2.stl
- [ ] link_3.stl
- [ ] link_4.stl
- [ ] link_5.stl
- [ ] gripper_base.stl
- [ ] gripper_left_finger.stl
- [ ] gripper_right_finger.stl

**최소 5개 이상 Export되면 진행 가능!**

---

## 🔍 파트 찾기 팁

### 방법 1: 이름 검색
FreeCAD 왼쪽 트리에서:
- **Ctrl+F** (검색)
- `手腕` (손목), `AL-ELBOW`, `LEG-HOLDER` 검색

### 방법 2: 3D 뷰에서 클릭
- 3D 뷰에서 로봇 부분 클릭
- 왼쪽 트리에서 자동 선택됨

### 방법 3: 계층 구조 확인
- `RoArm-M3-S ASM-TYPE-0(SP) v5` 확장
- 하위 부품들 확인

---

**작업을 시작하세요! 진행 상황을 알려주시면 도와드리겠습니다.** 🚀
