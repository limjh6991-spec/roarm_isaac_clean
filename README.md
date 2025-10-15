# RoArm M3 + Isaac Sim 5.0 - Clean Start

**생성일**: 2025년 10월 15일  
**목적**: RoArm M3 로봇팔을 Isaac Sim 5.0에서 처음부터 올바르게 설정하고 강화학습 수행

---

## 📁 프로젝트 구조

```
roarm_isaac_clean/
├── README.md                    # 이 파일
├── docs/                        # 문서 모음
│   ├── LESSONS_LEARNED.md      # 이전 프로젝트(codex_mcp)에서 배운 교훈
│   ├── SETUP_GUIDE.md          # 단계별 설정 가이드
│   └── TROUBLESHOOTING.md      # 문제 해결 가이드
├── resources/                   # 수집한 자료 모음
│   ├── isaac_sim/              # Isaac Sim 5.0 관련 자료
│   ├── roarm_m3/               # RoArm M3 관련 자료
│   ├── community/              # 커뮤니티 자료 (Reddit, GitHub)
│   └── RESOURCE_INDEX.md       # 자료 검색 인덱스
├── scripts/                     # Python 스크립트
│   ├── setup/                  # 초기 설정 스크립트
│   └── utils/                  # 유틸리티 함수
├── assets/                      # 로봇 모델 및 에셋
│   └── roarm_m3/
│       ├── urdf/               # URDF 파일
│       ├── meshes/             # 메시 파일
│       └── usd/                # USD 파일
├── configs/                     # 설정 파일
│   ├── robot_config.yaml       # 로봇 설정
│   └── training_config.yaml    # 학습 설정
└── tests/                       # 테스트 코드
    └── test_robot_load.py      # 로봇 로딩 테스트
```

---

## 📚 문서 가이드

### 필독 문서 (작업 시작 전)
1. **`docs/LESSONS_LEARNED.md`** ⭐⭐⭐⭐⭐
   - 이전 프로젝트에서 배운 교훈
   - 핵심 문제 3가지 및 해결 방안
   - 반드시 읽어야 하는 최우선 문서

2. **`docs/ISSUES_AND_SOLUTIONS.md`** ⭐⭐⭐⭐
   - 8개 이슈 상세 분석 (P0-P3 우선순위)
   - 각 이슈의 해결 방안 (코드 포함)
   - 문제 발생 시 참조

3. **`docs/ENVIRONMENT_STATUS.md`** ⭐⭐⭐
   - 현재 시스템 환경 상태
   - 하드웨어/소프트웨어 스펙
   - 확인 명령어 모음

4. **`docs/PROJECT_RESTART_SUMMARY.md`** ⭐⭐⭐⭐⭐
   - 프로젝트 재시작 종합 보고서
   - 단계별 액션 플랜
   - 타임라인 및 체크리스트

### 참고 문서 (필요시)
- `~/codex_mcp/README.md` - 이전 프로젝트 개요
- `~/codex_mcp/docs/STATUS.md` - 이전 프로젝트 최신 상태
- `~/codex_mcp/docs/comprehensive_analysis_2025-10-15.md` - 종합 분석

---

## 🎯 다음 단계: Phase 2 자료 수집

### 목표
Isaac Sim 5.0과 RoArm M3 관련 자료를 수집하여 올바른 방법론 확립

### 작업 항목
1. **Isaac Sim 5.0 공식 문서**
   - Articulation API 가이드
   - URDF Import best practices
   - Fixed-base robot 설정 방법
   - Physics 설정 튜토리얼

2. **RoArm M3 자료**
   - 공식 URDF 파일
   - 제조사 스펙
   - 커뮤니티 예제

3. **커뮤니티 자료**
   - Reddit, NVIDIA Forums
   - GitHub Issues
   - 관련 튜토리얼

### 완료 기준
- [ ] Isaac Sim 5.0 fixed-base 설정 방법 이해
- [ ] RoArm M3 URDF 파일 확보
- [ ] 최소 2개 참고 예제 확보
- [ ] `docs/REFERENCES.md` 작성 완료

**예상 소요 시간**: 1-2시간

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
