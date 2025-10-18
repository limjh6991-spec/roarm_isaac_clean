# RoArm M3 + Isaac Sim 5.0 - Production Ready

**생성일**: 2025년 10월 15일  
**최종 업데이트**: 2025년 10월 18일  
**상태**: ✅ **프리플라이트 전체 PASS** + 🧠 **온톨로지 시스템 구축** + 🔧 **URDF 표준화 진행 중**  
**목적**: RoArm M3 로봇팔을 Isaac Sim 5.0에서 처음부터 올바르게 설정하고 강화학습 수행

---

## 🎉 현재 상태: DevOps 인프라 + 지식 관리 시스템 + URDF 표준화 작업

### ✅ 완료된 작업

- [x] **URDF 표준화 프로세스 구축** (2025-10-18) 🆕
  - Isaac Sim 5.0 호환 경로 형식 확립 (file:// → 절대 경로)
  - mm→m 단위 변환 스케일 적용 (scale="0.001")
  - urdf_autopatch_standard.py 자동화 스크립트 개발
  - Joint origin 보존 로직 구현
  - 🟡 **진행 중**: STL 파일 Placement 베이크 문제 해결 필요

- [x] **온톨로지 기반 지식 관리 시스템** (2025-10-18)
  - 문제-솔루션 자동 추적
  - SPARQL 질의로 빠른 정보 검색
  - 재발 문제 자동 감지
  - pxr 환경 문제 완전 문서화

- [x] **프리플라이트 시스템 구축** (2025-10-17)
  - 3단계 자동 검증: System Check, Isaac Extensions, USD Integrity
  - 타임아웃 처리, JSON 로그 아카이브, 색상 출력
  - pip 설치 방식 Isaac Sim 환경 완벽 지원
  - Isaac 번들 pxr 사용 (pip install usd-core 금지)
  
- [x] **USD 무결성 검사 강화** (2025-10-17)
  - Stage metadata 검증 (metersPerUnit, upAxis)
  - PhysX Articulation 검증
  - Rigid Body/Mass/Collision 검증
  - Joint DriveAPI 검증
  - 경고/실패 구분 및 자동 진단

- [x] **Isaac Python 환경 자동 설정**
  - pxr 모듈 경로 자동 탐색
  - PYTHONPATH/LD_LIBRARY_PATH 자동 설정
  - venv와 표준 설치 모두 지원

---

## 📁 프로젝트 구조

```
roarm_isaac_clean/
├── README.md                    # 이 파일
├── docs/                        # 문서 모음
│   ├── LESSONS_LEARNED.md      # 이전 프로젝트에서 배운 교훈
│   ├── SETUP_GUIDE.md          # 단계별 설정 가이드
│   ├── DEVOPS_GUIDE.md         # DevOps 인프라 가이드 (NEW)
│   └── TROUBLESHOOTING.md      # 문제 해결 가이드
├── devops/                      # DevOps 인프라
│   ├── preflight_all.sh        # 마스터 프리플라이트 (타임아웃, JSON 로깅)
│   ├── preflight/              # 개별 프리플라이트 검사
│   │   ├── check_system.sh            # GPU, 드라이버, Vulkan, Python
│   │   ├── check_isaac_extensions.py  # Isaac 확장 로드 검사
│   │   └── check_usd_integrity.sh     # USD 스키마 무결성 검사
│   ├── isaac_python.sh         # pxr 환경 설정 래퍼
│   ├── setup_isaac_python_env.sh      # 환경 변수 자동 설정
│   └── diagnose_python_env.sh  # 환경 진단 도구
├── ontology/                    # 온톨로지 시스템 (NEW)
│   ├── roarm_domain.ttl        # 코어 온톨로지 (도메인 개념)
│   ├── instances/              # 실제 문제/솔루션 데이터
│   │   └── pxr_environment_problem.ttl
│   ├── queries/                # SPARQL 질의 모음
│   │   └── diagnostics.sparql
│   └── README.md               # 온톨로지 사용 가이드
├── scripts/                     # Python 스크립트
│   ├── convert_urdf_to_usd.py  # URDF → USD 변환
│   ├── ontology/               # 온톨로지 도구 (NEW)
│   │   └── query_ontology.py  # SPARQL 질의 실행
│   ├── setup/                  # 초기 설정 스크립트
│   └── usd/                    # USD 관련 유틸리티
│       └── verify_usd_quick.py # USD 빠른 검증
├── resources/                   # 수집한 자료 모음
│   ├── isaac_sim/              # Isaac Sim 5.0 관련 자료
│   ├── roarm_m3/               # RoArm M3 관련 자료
│   ├── community/              # 커뮤니티 자료
│   └── RESOURCE_INDEX.md       # 자료 검색 인덱스
├── scripts/                     # Python 스크립트
│   ├── convert_urdf_to_usd.py  # URDF → USD 변환
│   ├── setup/                  # 초기 설정 스크립트
│   └── usd/                    # USD 관련 유틸리티
│       └── verify_usd_quick.py # USD 빠른 검증 (NEW)
├── assets/                      # 로봇 모델 및 에셋
│   └── roarm_m3/
│       ├── urdf/               # URDF 파일
│       ├── meshes/             # STL 메시 파일
│       └── usd/                # USD 파일 (변환 결과)
├── configs/                     # 설정 파일
│   ├── robot_config.yaml       # 로봇 설정
│   └── training_config.yaml    # 학습 설정
├── logs/                        # 로그 디렉토리 (NEW)
│   ├── preflight/              # 프리플라이트 로그 + JSON 아카이브
│   └── isaac/                  # Isaac Sim 실행 로그
└── tests/                       # 테스트 코드
    └── test_robot_load.py      # 로봇 로딩 테스트
```

---

## 📚 문서 가이드

### ⚠️ CRITICAL: 필독 문서

1. **`docs/PXR_ENVIRONMENT_GUIDE.md`** 🔴 **최우선 필독**
   - pxr (USD Python 바인딩) 환경 설정 완전 가이드
   - pip install usd-core 금지 이유 상세 설명
   - PYTHONPATH + LD_LIBRARY_PATH 설정 방법
   - 트러블슈팅: ModuleNotFoundError, ImportError 해결
   - **USD 관련 모든 작업 전 반드시 읽어야 함**

### 필독 문서 (작업 시작 전)
2. **`docs/DEVOPS_GUIDE.md`** ⭐⭐⭐⭐⭐
   - DevOps 인프라 완전 가이드
   - **pxr 모듈 환경 설정 섹션 포함** (⚠️ CRITICAL)
   - 프리플라이트 시스템 상세 설명
   - 로그 관리, 트러블슈팅

3. **`docs/LESSONS_LEARNED.md`** ⭐⭐⭐⭐⭐
   - 이전 프로젝트에서 배운 교훈
   - **핵심 문제 4가지**: USD CollisionAPI, 원격 GUI, API 변경, **pxr 환경 설정**
   - 반드시 읽어야 하는 최우선 문서

4. **`docs/ISSUES_AND_SOLUTIONS.md`** ⭐⭐⭐⭐
   - 8개 이슈 상세 분석 (P0-P3 우선순위)
   - 각 이슈의 해결 방안 (코드 포함)
   - 문제 발생 시 참조

5. **`docs/ENVIRONMENT_STATUS.md`** ⭐⭐⭐
   - 현재 시스템 환경 상태
   - 하드웨어/소프트웨어 스펙
   - 확인 명령어 모음

6. **`docs/PROJECT_RESTART_SUMMARY.md`** ⭐⭐⭐⭐⭐
   - 프로젝트 재시작 종합 보고서
   - 단계별 액션 플랜
   - 타임라인 및 체크리스트

### 참고 문서 (필요시)
- `~/codex_mcp/README.md` - 이전 프로젝트 개요
- `~/codex_mcp/docs/STATUS.md` - 이전 프로젝트 최신 상태
- `~/codex_mcp/docs/comprehensive_analysis_2025-10-15.md` - 종합 분석

---

## 🚀 빠른 시작

### 1. 프리플라이트 검사 실행
```bash
# 전체 프리플라이트 (시스템, 확장, USD 무결성)
bash devops/preflight_all.sh

# 개별 검사
bash devops/preflight/check_system.sh
python devops/preflight/check_isaac_extensions.py
bash devops/preflight/check_usd_integrity.sh assets/roarm_m3/usd/roarm_m3.usd
```

**성공 시 출력**:
```
========================================================================
Total: 3  PASS:3  FAIL:0  SKIP:0
ALL PREFLIGHT CHECKS PASSED!
System ready for Isaac Sim development.
```

### 2. URDF → USD 변환
```bash
source ~/isaacsim-venv/bin/activate
python scripts/convert_urdf_to_usd.py
```

### 3. USD 빠른 검증
```bash
python scripts/usd/verify_usd_quick.py assets/roarm_m3/usd/roarm_m3.usd
```

### 4. 온톨로지 지식 검색 🆕
```bash
# rdflib, networkx, matplotlib 설치 (최초 1회)
pip install rdflib networkx matplotlib

# 프로젝트 상태 조회
python scripts/ontology/query_ontology.py --query project_status

# pxr 문제 솔루션 찾기
python scripts/ontology/query_ontology.py --query pxr_solutions

# 사용 가능한 질의 목록
python scripts/ontology/query_ontology.py --list-queries

# 지식 그래프 시각화 (PNG 생성)
python scripts/ontology/visualize_graph.py --focus pxr_environment_problem --output pxr_graph.png
python scripts/ontology/visualize_graph.py --output full_graph.png

# 브라우저로 시각화 보기 (추천!) 🆕
python scripts/ontology/view_ontology.py
# → http://localhost:8000 자동으로 열림
# → VS Code Simple Browser에서도 확인 가능
```

**생성된 시각화**:
- 📊 **pxr_subgraph.png**: pxr 문제 중심 서브그래프 (17노드, 29엣지)
- 📊 **full_ontology_graph.png**: 전체 지식 그래프 (128노드, 127엣지)

**VS Code에서 이미지 보기**:
```bash
# VS Code에서 파일 탐색기 열기 → docs/ontology/ 폴더
# pxr_subgraph.png 또는 full_ontology_graph.png 클릭
# 또는 터미널에서:
code docs/ontology/pxr_subgraph.png
code docs/ontology/full_ontology_graph.png
```

### 5. GUI로 USD 확인
```bash
# (작업 예정 - GUI 래퍼 스크립트 추가 예정)
```

---

## 🧠 온톨로지 시스템 (NEW)

### 개요

**온톨로지(Ontology)**는 프로젝트의 지식을 구조화하여 관리하는 시스템입니다.

**핵심 가치**:
- ✅ 문제 발생 시 기존 솔루션 즉시 검색 (10분 → 10초)
- ✅ 재발 문제 자동 감지 및 과거 해결책 제공
- ✅ 문제-솔루션-문서 관계 명시적 연결
- ✅ SPARQL 질의로 복잡한 지식 탐색

### 빠른 사용

```bash
# pxr 환경 문제 솔루션 찾기
python scripts/ontology/query_ontology.py --query pxr_solutions

# 재발 문제 확인
python scripts/ontology/query_ontology.py --query recurring_problems

# 프로젝트 전체 상태
python scripts/ontology/query_ontology.py --query project_status
```

### 예제 출력

```
✅ pxr 문제 해결책:

1. isaac_python.sh 래퍼 스크립트
   성공률: 100%
   경로: /home/roarm_m3/roarm_isaac_clean/devops/isaac_python.sh

2. 수동 PYTHONPATH/LD_LIBRARY_PATH 설정
   성공률: 80%
```

**자세한 내용**: `ontology/README.md` 참조

---

## 📊 프리플라이트 시스템

### 마스터 프리플라이트
- **경로**: `devops/preflight_all.sh`
- **기능**:
  - 3단계 검사를 순차 실행 (System → Extensions → USD)
  - 각 단계별 타임아웃 설정 (30s/120s/45s)
  - 실패해도 계속 진행 (모든 이슈 수집)
  - JSON 로그 자동 아카이브 (`logs/preflight/*.json`)
  - 색상 코드 출력 (PASS=녹색, FAIL=빨강, SKIP=노랑)

### 개별 프리플라이트 검사

#### 1. System Check (`devops/preflight/check_system.sh`)
- NVIDIA 드라이버 (≥550)
- GPU 메모리
- Vulkan 지원
- Python 버전 (3.10 또는 3.11)
- 환경 변수 (CUDA_VISIBLE_DEVICES, VK_ICD_FILENAMES)

#### 2. Isaac Extensions (`devops/preflight/check_isaac_extensions.py`)
- SimulationApp headless 부팅 (5초 이내)
- 필수 확장 로드 확인:
  - `isaacsim.asset.importer.urdf`
  - `isaacsim.core.api`
  - `omni.isaac.core`
  - `omni.physx`
  - `omni.usd`

#### 3. USD Integrity (`devops/preflight/check_usd_integrity.sh`)
- Isaac 번들 pxr 자동 탐색 및 사용
- Stage metadata (metersPerUnit=1.0, upAxis=Z)
- PhysX Articulation Root (정확히 1개, 중첩 금지)
- Rigid Body MassAPI 존재
- Collision 메시 존재
- Joint DriveAPI 바인딩

---

## 🎯 다음 단계

### Phase 3: GUI 테스트 및 시각적 검증
- [ ] GUI 래퍼 스크립트 작성 (`devops/run_isaac_supervised.sh`)
- [ ] USD 파일 시각적 검증 (STL 메시, 조인트 동작)
- [ ] Collision 메시 표시 확인
- [ ] 물리 시뮬레이션 테스트

### Phase 4: URDF → USD 변환 개선
- [ ] Collision 메시 자동 생성
- [ ] JointDriveAPI 자동 적용
- [ ] MassAPI 자동 계산
- [ ] 변환 스크립트 리팩토링

### Phase 5: 강화학습 파이프라인
- [ ] Gymnasium 환경 구현
- [ ] Observation/Action space 정의
- [ ] Reward 함수 설계
- [ ] PPO 학습 실행

---

## ⚠️ 주의사항

### ❌ 피해야 할 것
1. 검증 없는 USD 파일 사용
2. 구형 API (`omni.isaac.core.*`) 사용
3. X11 포워딩으로 GUI 렌더링 시도
4. 땜질식 문제 해결
5. 단계 건너뛰기

### ✅ 반드시 해야 할 것
1. USD 검증 스크립트 항상 실행
2. 최신 API (`isaacsim.core.*`) 사용
3. WebRTC 또는 Headless 모드 사용
4. 각 단계 완료 후 다음 단계 진행
5. 모든 결정과 문제 문서화

---

## 📞 도움 요청

**Phase 2 (자료 수집) 진행 전 확인 요청 부탁드립니다.**

현재 상태:
- ✅ Phase 1 완료 (환경 분석)
- ⏳ Phase 2 준비 완료 (자료 수집 대기)

다음 단계를 진행하시겠습니까?

---

## 📋 현재 상태

### Phase 1: 환경 분석 완료 ✅

- [x] 프로젝트 폴더 생성
- [x] 이전 프로젝트(codex_mcp) 분석 완료
- [x] 주요 이슈 및 교훈 문서화
- [x] 환경 상태 파악 및 기록
- [x] 해결 방안 정리

**생성된 문서:**
- `docs/LESSONS_LEARNED.md` - 이전 프로젝트에서 배운 교훈
- `docs/ENVIRONMENT_STATUS.md` - 현재 환경 상태 및 설정
- `docs/ISSUES_AND_SOLUTIONS.md` - 주요 이슈 및 해결 방안

### Phase 2: 자료 수집 완료 ✅

- [x] Isaac Sim 5.0 공식 문서 조사
- [x] RoArm M3 공식 자료 수집 (Wiki, 스펙, 다운로드)
- [x] 커뮤니티 자료 검색 (Forums, Reddit, GitHub)
- [x] REFERENCES.md 작성 완료

**수집된 자료:**
- `resources/roarm_m3/waveshare_wiki_summary.md` - RoArm M3 Wiki 요약
- `resources/isaac_sim/documentation_links.md` - Isaac Sim 문서 링크
- `resources/community/isaac_sim_resources.md` - 커뮤니티 자료
- `docs/REFERENCES.md` - 전체 참고 자료 정리

**주요 발견:**
- ✅ RoArm M3는 ROS2 지원 (URDF 존재 가능성 높음)
- ✅ STEP 3D 모델, 2D 치수도 다운로드 가능
- ✅ LeRobot 통합 지원
- ⚠️ Isaac Sim 5.0 공식 문서 URL 변경 (로컬 확인 필요)

### 다음 단계: Phase 3 USD 파일 생성 ⏳
- [ ] 디렉토리 구조 구축
- [ ] 이전 프로젝트 분석 문서 작성
- [ ] Isaac Sim 5.0 자료 수집
- [ ] RoArm M3 자료 수집
- [ ] 기본 설정 가이드 작성

---

## 🚀 다음 단계

### Phase 1: 준비 및 자료 수집 (오늘)
1. 이전 프로젝트(codex_mcp) 분석 및 교훈 문서화
2. Isaac Sim 5.0 공식 문서 수집
3. RoArm M3 공식 자료 수집
4. 커뮤니티 자료 수집 (Reddit, GitHub)
5. Resources 폴더 구축 및 인덱싱

### Phase 2: 기초 설정 (내일)
1. URDF 검증 및 수정
2. USD 변환 (올바른 방법)
3. Isaac Sim에서 로봇 로딩 테스트
4. 기본 물리 시뮬레이션 확인

### Phase 3: 강화학습 환경 구축
1. Gym 환경 구현
2. 보상 함수 설계
3. PPO 학습 파이프라인 구축

### Phase 4: 원격 렌더링 및 배포
1. WebRTC 스트리밍 설정
2. 학습된 정책 테스트
3. 문서화 및 정리

---

## 📚 참고 자료

### Isaac Sim 5.0
- 공식 문서: https://docs.omniverse.nvidia.com/isaacsim/latest/
- API 레퍼런스: (수집 예정)
- 마이그레이션 가이드: (수집 예정)

### RoArm M3
- 공식 사이트: (수집 예정)
- GitHub: (수집 예정)
- URDF: (수집 예정)

### 커뮤니티
- Reddit r/IsaacSim: (수집 예정)
- GitHub Discussions: (수집 예정)

---

## ⚠️ 이전 프로젝트(codex_mcp)에서 배운 교훈

자세한 내용은 `docs/LESSONS_LEARNED.md` 참조

**핵심 문제:**
1. CollisionAPI 누락 → PhysX 크래시
2. ArticulationAPI 설정 오류 → 로봇 인식 실패
3. Fixed Joint vs Kinematic Body 혼동
4. URDF → USD 변환 과정의 불완전성

**피해야 할 것:**
- 검증 없이 USD 변환
- Physics API 설정 건너뛰기
- 테스트 없이 복잡한 기능 추가
- 문서화 부족

---

## 💡 핵심 원칙

1. **단계별 검증**: 각 단계마다 철저히 테스트
2. **문서 우선**: 모든 결정과 문제를 문서화
3. **공식 문서 참조**: 추측하지 말고 공식 문서 확인
4. **간단하게 시작**: 복잡한 기능은 나중에 추가

---


**작성**: 2025-10-15  
**업데이트**: 진행 상황에 따라 지속 업데이트
