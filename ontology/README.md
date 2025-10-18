# RoArm M3 온톨로지 시스템

**버전**: v0.1  
**생성일**: 2025-10-18

---

## 🎯 온톨로지 시스템이란?

**온톨로지(Ontology)**는 프로젝트의 지식을 구조화하여 관리하는 시스템입니다.

### 핵심 기능

1. **문제 자동 추적**: 발생한 문제를 체계적으로 기록
2. **솔루션 연결**: 문제-해결책 관계를 명시적으로 연결
3. **지식 검색**: SPARQL 질의로 빠른 정보 검색
4. **시각화**: 지식 그래프를 웹 UI로 탐색

### 왜 필요한가?

| Before (문서만) | After (온톨로지) |
|----------------|------------------|
| 문제 검색: 10분+ | 문제 검색: 10초 |
| 수동 문서 탐색 | 자동 관계 추적 |
| 재발 문제 반복 | 기존 솔루션 즉시 제공 |

---

## 📁 디렉토리 구조

```
ontology/
├── roarm_domain.ttl              # 코어 온톨로지 (도메인 개념)
├── instances/                    # 실제 데이터
│   ├── pxr_environment_problem.ttl   # pxr 환경 문제
│   └── ...
├── queries/                      # SPARQL 질의 모음
│   └── diagnostics.sparql
└── README.md                     # 이 파일
```

---

## 🚀 빠른 시작

### 1. 의존성 설치

```bash
# rdflib 설치 (Python RDF 라이브러리)
pip install rdflib
```

### 2. 첫 번째 질의 실행

```bash
# 프로젝트 상태 조회
python scripts/ontology/query_ontology.py --query project_status

# pxr 관련 모든 정보
python scripts/ontology/query_ontology.py --query all_about_pxr

# CRITICAL 미해결 문제
python scripts/ontology/query_ontology.py --query critical_open_problems
```

### 3. 질의 목록 보기

```bash
python scripts/ontology/query_ontology.py --list-queries
```

---

## 📊 사용 예시

### 예시 1: pxr 문제 솔루션 찾기

```bash
python scripts/ontology/query_ontology.py --query pxr_solutions
```

**출력**:
```
✅ pxr 문제 해결책:

1. isaac_python.sh 래퍼 스크립트
   성공률: 100%
   경로: /home/roarm_m3/roarm_isaac_clean/devops/isaac_python.sh

2. 수동 PYTHONPATH/LD_LIBRARY_PATH 설정
   성공률: 80%
```

### 예시 2: 재발 문제 확인

```bash
python scripts/ontology/query_ontology.py --query recurring_problems
```

**출력**:
```
🔁 재발 문제:
  • pxr 모듈 환경 설정 문제 (3회, CRITICAL, SOLVED)
```

### 예시 3: 프로젝트 전체 상태

```bash
python scripts/ontology/query_ontology.py --query project_status
```

**출력**:
```
📊 프로젝트 상태 요약:
  총 문제: 1
  CRITICAL: 1
  해결: 1
  미해결: 0
```

---

## 🔍 SPARQL 질의 작성

### 기본 문법

```sparql
PREFIX : <http://roarm.ai/ontology#>

SELECT ?problem ?name ?severity
WHERE {
  ?problem a :Problem ;
           :name ?name ;
           :severity "CRITICAL" ;
           :status "OPEN" .
}
```

### 커스텀 질의 실행

```bash
# 커스텀 .sparql 파일 작성 후 실행
python scripts/ontology/query_ontology.py \
  --custom-query ontology/queries/my_query.sparql
```

---

## 📚 온톨로지 구조

### 주요 클래스

| 클래스 | 설명 |
|--------|------|
| `:Problem` | 프로젝트 문제 |
| `:Solution` | 해결책 |
| `:Resource` | 문서, 스크립트 등 리소스 |
| `:PreflightCheck` | 프리플라이트 검사 |
| `:Environment` | 개발 환경 |

### 주요 관계

| 관계 | 설명 |
|------|------|
| `:hasSolution` | 문제 → 솔루션 |
| `:documents` | 문서 → 엔티티 |
| `:implements` | 스크립트 → 솔루션 |
| `:requires` | 엔티티 → 의존성 |
| `:relatedTo` | 엔티티 ↔ 엔티티 |

### 주요 속성

| 속성 | 타입 | 설명 |
|------|------|------|
| `:severity` | String | CRITICAL, HIGH, MEDIUM, LOW |
| `:status` | String | OPEN, SOLVED, RECURRING |
| `:occurrenceCount` | Integer | 재발 횟수 |
| `:successRate` | Float | 솔루션 성공률 (0.0~1.0) |

---

## 🛠️ 도구

### 1. query_ontology.py

SPARQL 질의 실행 도구

```bash
python scripts/ontology/query_ontology.py --query <query_name>
```

**미리 정의된 질의**:
- `critical_open_problems`: CRITICAL 미해결 문제
- `all_about_pxr`: pxr 관련 모든 정보
- `pxr_solutions`: pxr 문제 솔루션
- `recurring_problems`: 재발 문제
- `project_status`: 프로젝트 상태 요약

### 2. add_problem.py (예정)

신규 문제 등록 도구

```bash
python scripts/ontology/add_problem.py \
  --type EnvironmentProblem \
  --name "새 문제" \
  --severity CRITICAL \
  --symptom "에러 메시지"
```

### 3. visualize_graph.py (예정)

지식 그래프 시각화 도구

```bash
python scripts/ontology/visualize_graph.py \
  --output graph.png
```

---

## 📖 예제 데이터: pxr 환경 문제

`instances/pxr_environment_problem.ttl` 참조

### 핵심 정보

- **문제 ID**: PROB-001
- **이름**: pxr 모듈 환경 설정 문제
- **심각도**: CRITICAL
- **재발 횟수**: 3회
- **상태**: SOLVED
- **해결 시간**: 9.5시간

### 솔루션

1. **isaac_python.sh 래퍼** (성공률 100%)
2. **수동 환경 변수 설정** (성공률 80%)

### 관련 문서

- `docs/PXR_ENVIRONMENT_GUIDE.md` (⚠️ CRITICAL)
- `docs/DEVOPS_GUIDE.md` (pxr 섹션)
- `docs/LESSONS_LEARNED.md` (문제 4)

---

## 🔮 향후 계획

### Phase 1 (현재)

- ✅ 코어 온톨로지 정의
- ✅ pxr 문제 마이그레이션
- ✅ SPARQL 질의 도구

### Phase 2 (다음)

- ⬜ 프리플라이트 통합
- ⬜ 자동 문제 등록
- ⬜ 솔루션 제안 시스템

### Phase 3 (미래)

- ⬜ 웹 UI 대시보드
- ⬜ 지식 그래프 시각화
- ⬜ AI Agent 통합 (Copilot/Claude)

---

## 🔗 관련 문서

- 📄 **시스템 설계**: `docs/ontology/ONTOLOGY_SYSTEM_DESIGN.md`
- 📄 **pxr 가이드**: `docs/PXR_ENVIRONMENT_GUIDE.md`
- 📄 **DevOps 가이드**: `docs/DEVOPS_GUIDE.md`

---

**작성**: GitHub Copilot  
**프로젝트**: roarm_isaac_clean  
**버전**: v0.1
