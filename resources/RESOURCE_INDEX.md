# Resources 디렉토리 인덱스

**작성일**: 2025-10-18  
**목적**: 프로젝트 관련 자료의 빠른 검색 및 참조

---

## 📂 디렉토리 구조

```
resources/
├── isaac_sim/              # Isaac Sim 5.0 관련 자료
│   └── documentation_links.md
├── roarm_m3/               # RoArm M3 로봇팔 관련 자료
│   └── waveshare_wiki_summary.md
├── community/              # 커뮤니티 및 외부 자료
│   └── isaac_sim_resources.md
└── RESOURCE_INDEX.md       # 이 파일
```

---

## 📖 자료 목록

### Isaac Sim 5.0 (isaac_sim/)

#### documentation_links.md
**내용**:
- 공식 문서 링크 모음 (일부 404 오류 - 경로 변경됨)
- 로컬 문서 확인 방법
- 필요한 정보 항목 정리

**주요 항목**:
- Fixed-base Articulation 설정 방법
- URDF Import 옵션
- Physics Scene 권장 파라미터

**상태**: 🟡 일부 링크 업데이트 필요

**관련 문서**: `docs/DEVOPS_GUIDE.md`, `docs/SETUP_GUIDE.md`

---

### RoArm M3 (roarm_m3/)

#### waveshare_wiki_summary.md
**내용**:
- Waveshare Wiki에서 수집한 RoArm M3 스펙
- 하드웨어 사양, 통신 프로토콜
- 제조사 제공 예제 코드 요약

**주요 항목**:
- DOF (Degrees of Freedom): 6축
- 작업 반경 (Workspace)
- 관절 각도 범위
- 통신 인터페이스 (Serial, USB)

**상태**: ✅ 최신 정보 유지 중

**관련 문서**: `assets/roarm_m3/urdf/README.md` (작성 예정)

---

### Community (community/)

#### isaac_sim_resources.md
**내용**:
- 커뮤니티 포럼, GitHub 이슈 링크
- 유용한 외부 튜토리얼
- Stack Overflow Q&A 모음

**주요 항목**:
- NVIDIA Forums: Isaac Sim 카테고리
- Reddit: r/IsaacSim, r/reinforcementlearning
- GitHub Issues: 자주 묻는 질문 (FAQ)

**상태**: ✅ 활성 유지 중

**관련 문서**: `docs/TROUBLESHOOTING.md`

---

## 🔍 주제별 검색 가이드

### pxr 모듈 / USD 환경 설정
- 📄 **주 문서**: `docs/PXR_ENVIRONMENT_GUIDE.md` 🔴 최우선
- 📄 **보조**: `docs/DEVOPS_GUIDE.md` → "⚠️ CRITICAL: pxr 모듈 환경 설정" 섹션
- 📄 **교훈**: `docs/LESSONS_LEARNED.md` → "문제 4: pip 설치 시 pxr 모듈 환경 설정"
- 🔗 **외부**: `resources/isaac_sim/documentation_links.md` → USD Python API

### URDF → USD 변환
- 📄 **가이드**: `docs/SETUP_GUIDE.md` (작성 예정)
- 📄 **이슈**: `docs/ISSUES_AND_SOLUTIONS.md` → P0 이슈
- 🔗 **로봇 스펙**: `resources/roarm_m3/waveshare_wiki_summary.md`
- 🔗 **커뮤니티**: `resources/community/isaac_sim_resources.md`

### Physics & Articulation
- 📄 **이슈**: `docs/ISSUES_AND_SOLUTIONS.md` → P0, P1 이슈
- 📄 **교훈**: `docs/LESSONS_LEARNED.md` → "문제 1: USD CollisionAPI 누락"
- 🔗 **공식 문서**: `resources/isaac_sim/documentation_links.md` → Physics 설정

### 원격 GUI / 시각화
- 📄 **교훈**: `docs/LESSONS_LEARNED.md` → "문제 2: 원격 GUI 렌더링 실패"
- 📄 **환경**: `docs/ENVIRONMENT_STATUS.md` → 하드웨어 스펙
- 🔗 **커뮤니티**: `resources/community/isaac_sim_resources.md` → WebRTC 가이드

### DevOps / 프리플라이트
- 📄 **메인**: `docs/DEVOPS_GUIDE.md`
- 📄 **교훈**: `docs/LESSONS_LEARNED.md` → "문제 4: pxr 모듈 환경 설정"
- 🔧 **스크립트**: `devops/preflight/` 디렉토리

---

## 📝 자료 추가 가이드

새로운 자료를 추가할 때:

1. **적절한 디렉토리 선택**
   - Isaac Sim 공식/API → `isaac_sim/`
   - RoArm M3 하드웨어/스펙 → `roarm_m3/`
   - 외부 커뮤니티/튜토리얼 → `community/`

2. **파일명 규칙**
   - 소문자, 언더스코어 구분
   - 설명적이고 명확한 이름
   - 예: `physics_scene_parameters.md`, `urdf_import_guide.md`

3. **이 인덱스 업데이트**
   - 새 파일 추가 시 이 파일 수정
   - 주제별 검색 가이드에 링크 추가

4. **메타데이터 포함**
   - 작성일/수집일
   - 출처 (URL, 문서명)
   - 마지막 확인일 (외부 링크의 경우)

---

## ⚠️ 주의 사항

### 외부 링크 유효성
- Isaac Sim 공식 문서 URL이 자주 변경됨
- 404 오류 발생 시 `isaac_sim/documentation_links.md` 업데이트
- 가능하면 로컬 문서 또는 스크린샷 보관

### 버전 호환성
- Isaac Sim 버전 변경 시 자료 재검증 필요
- 특정 버전 전용 정보는 파일명에 버전 명시
  - 예: `isaac_sim_5.0_api_changes.md`

### 민감 정보
- API 키, 토큰 등 민감 정보 절대 저장 금지
- `.gitignore`에 추가된 패턴 준수

---

## 🔗 관련 문서

- 📄 **프로젝트 구조**: `README.md` → "📁 프로젝트 구조" 섹션
- 📄 **문서 가이드**: `README.md` → "📚 문서 가이드" 섹션
- 📄 **DevOps 가이드**: `docs/DEVOPS_GUIDE.md`

---

**마지막 업데이트**: 2025-10-18  
**다음 업데이트 예정**: 새 자료 추가 시
