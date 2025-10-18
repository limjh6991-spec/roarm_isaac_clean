# FreeCAD 자동 STL 추출 가이드

RoArm-M3 STEP 파일에서 각 링크의 STL 파일을 자동으로 추출하는 방법입니다.

## 📂 파일 구조

```
scripts/
├── freecad_auto_export_stl.py     # Python 스크립트 (CLI/독립 실행)
├── freecad_macro_export_stl.FCMacro  # FreeCAD 매크로 (GUI 실행)
└── freecad_list_objects.FCMacro    # 객체 이름 확인용 매크로
```

---

## 🎯 방법 1: FreeCAD Macro (추천)

### 단계 1: FreeCAD에서 파일 열기

```bash
freecad assets/roarm_m3/meshes/visual/roarm_m3.FCStd
```

또는 FreeCAD GUI에서:
- File → Open → `roarm_m3.FCStd` 선택

### 단계 2: 매크로 실행

**A. Macro 메뉴 사용:**
1. `Macro` → `Macros...` 클릭
2. `User macros location` 버튼 클릭
3. `scripts/freecad_macro_export_stl.FCMacro` 파일 복사
4. 다시 `Macro` → `Macros...`
5. `freecad_macro_export_stl` 선택 → `Execute` 클릭

**B. Execute Macro 메뉴 사용:**
1. `Tools` → `Execute Macro` 클릭
2. `scripts/freecad_macro_export_stl.FCMacro` 선택
3. `Open` 클릭

### 단계 3: 결과 확인

```bash
ls -lh assets/roarm_m3/meshes/visual/*.stl
```

출력:
- `base_link.stl`
- `link_1.stl` ~ `link_5.stl`
- `gripper_base.stl`
- `gripper_left_finger.stl`
- `gripper_right_finger.stl`

---

## 🎯 방법 2: Python CLI (고급)

### 전제 조건

FreeCAD Python API를 Python에서 직접 사용하려면:

```bash
# FreeCAD Python 경로 확인
which freecadcmd

# 환경변수 설정
export PYTHONPATH=/usr/lib/freecad-python3/lib:$PYTHONPATH
```

### 실행

```bash
# FCStd 파일에서 추출 (빠름)
python scripts/freecad_auto_export_stl.py

# STEP 파일에서 직접 추출 (느림, 10분+)
python scripts/freecad_auto_export_stl.py step

# 객체 목록 나열 (디버깅)
python scripts/freecad_auto_export_stl.py list
```

---

## 🔍 디버깅: 객체 이름 찾기

부품을 찾을 수 없다면 먼저 객체 이름을 확인하세요:

### 매크로 실행

1. FreeCAD에서 파일 열기
2. `Macro` → `Macros...` → `freecad_list_objects.FCMacro` 실행

### 출력 예시

```
📋 FreeCAD 문서 객체 목록: roarm_m3
   총 10,342개 객체

No.   Label                                    Has Shape
----------------------------------------------------------------------
1     舵机型材底座                              ✓
2     旋转底座                                  ✓
3     大臂                                      ✓
...

🔍 RoArm 관련 부품 검색 결과
----------------------------------------------------------------------
🔎 '底座' 검색:
  • 舵机型材底座                              ✓
  • 旋转底座                                  ✓

🔎 '大臂' 검색:
  • 大臂                                      ✓
...
```

---

## 🛠️ 부품 이름 매핑

스크립트에서 사용하는 중국어 → 영어 매핑:

| 영어 링크명 | 중국어 이름 | 설명 |
|------------|------------|------|
| `base_link` | 舵机型材底座 | 서보 베이스 |
| `link_1` | 旋转底座 | 회전 베이스 (joint_1) |
| `link_2` | 大臂 | 큰 팔 / 숄더 (joint_2) |
| `link_3` | 小臂 | 작은 팔 / 엘보우 (joint_3) |
| `link_4` | 手腕a | 손목 a (joint_4) |
| `link_5` | 手腕b | 손목 b (joint_5) |
| `gripper_base` | 夹爪底座 | 그리퍼 베이스 |
| `gripper_left_finger` | 夹爪左 | 왼쪽 그리퍼 |
| `gripper_right_finger` | 夹爪右 | 오른쪽 그리퍼 |

### 이름이 다르다면?

`freecad_macro_export_stl.FCMacro` 파일의 `LINK_PARTS` 딕셔너리를 수정:

```python
LINK_PARTS = {
    "base_link": "실제_중국어_이름",  # 여기를 수정
    ...
}
```

---

## ⚙️ STL 품질 설정

### Mesh Deviation (메시 편차)

더 정밀한 STL이 필요하면 `mesh_deviation` 값 조정:

```python
def export_to_stl(obj, filename, mesh_deviation=0.1):  # ← 이 값
```

- **0.1** (기본): 균형 잡힌 품질
- **0.05**: 더 정밀, 파일 크기 증가
- **0.2**: 빠르고 작지만 덜 정밀

---

## 📊 출력 예시

```
==============================================================
🤖 RoArm-M3 STL 자동 추출 시작
==============================================================

📂 문서: roarm_m3
   객체 수: 10,342
📁 출력: /home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/meshes/visual

[1/9] base_link 검색 중...
  ✓ 발견: 舵机型材底座
  ✅ base_link.stl: 2,010 삼각형, 99.2 KB

[2/9] link_1 검색 중...
  ✓ 발견: 旋转底座
  ✅ link_1.stl: 752 삼각형, 37.1 KB

[3/9] link_2 검색 중...
  ✓ 발견: 大臂
  ✅ link_2.stl: 2,186 삼각형, 107.8 KB

...

==============================================================
✅ 추출 완료: 9/9개 성공
==============================================================
```

---

## ❌ 문제 해결

### 1. "활성 문서가 없습니다"

**원인**: FreeCAD에서 파일을 열지 않았음

**해결**:
```bash
freecad assets/roarm_m3/meshes/visual/roarm_m3.FCStd
```

### 2. "부품을 찾을 수 없습니다"

**원인**: 중국어 이름이 정확하지 않음

**해결**:
1. `freecad_list_objects.FCMacro` 실행
2. 실제 부품 이름 확인
3. `LINK_PARTS` 딕셔너리 수정

### 3. "FreeCAD API를 찾을 수 없습니다"

**원인**: Python 스크립트 실행 시 FreeCAD 모듈 없음

**해결**:
```bash
# FreeCAD CLI 사용
freecadcmd -c scripts/freecad_auto_export_stl.py

# 또는 GUI 매크로 사용
```

### 4. STL이 중심에서 벗어남

**원인**: FreeCAD 좌표계가 조립품 기준

**해결**:
```bash
# STL 중심점 자동 수정 스크립트 실행
python scripts/center_stl_files.py
```

---

## 🚀 완전 자동화 워크플로우

```bash
# 1. FreeCAD에서 FCStd 저장 (최초 1회)
freecad resources/roarm_m3/RoArm-M3.step
# → File → Save As → roarm_m3.FCStd

# 2. FreeCAD 종료 후 자동 추출
freecadcmd -c scripts/freecad_auto_export_stl.py

# 3. STL 중심점 보정
python scripts/center_stl_files.py

# 4. URDF 검증
python scripts/validate_urdf.py

# 5. Isaac Sim import
~/isaac-sim.sh &
# → File → Import → URDF
```

---

## 📝 체크리스트

추출 전:
- [ ] FreeCAD 설치 확인: `freecad --version`
- [ ] FCStd 파일 존재: `assets/roarm_m3/meshes/visual/roarm_m3.FCStd`
- [ ] 출력 디렉토리 권한 확인

추출 후:
- [ ] 9개 STL 파일 생성 확인
- [ ] 파일 크기 확인 (각 14KB ~ 146KB)
- [ ] STL 중심점 보정 실행
- [ ] URDF import 테스트

---

## 🔗 관련 문서

- `docs/FREECAD_EXPORT_CHECKLIST.md` - 수동 추출 가이드
- `docs/MESH_ORIGIN_FIX.md` - 중심점 문제 해결
- `scripts/center_stl_files.py` - STL 보정 스크립트
- `scripts/analyze_stl_center.py` - STL 분석 스크립트

---

## 💡 팁

### 빠른 테스트
단일 부품만 추출하려면 매크로 수정:
```python
LINK_PARTS = {
    "base_link": "舵机型材底座",  # 이것만 남기고 나머지 주석 처리
}
```

### 배치 처리
여러 버전의 STEP 파일을 처리하려면 쉘 스크립트:
```bash
for step_file in resources/roarm_m3/*.step; do
    echo "Processing $step_file..."
    freecadcmd -c scripts/freecad_auto_export_stl.py "$step_file"
done
```

### Python Console에서 실행
FreeCAD GUI 내부:
1. `View` → `Panels` → `Python console` 활성화
2. 콘솔에서 실행:
```python
exec(open("scripts/freecad_macro_export_stl.FCMacro").read())
```
