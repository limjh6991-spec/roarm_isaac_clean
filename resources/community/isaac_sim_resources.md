# Isaac Sim 커뮤니티 자료

**수집일**: 2025년 10월 15일

## Reddit

### r/IsaacSim
- **URL**: https://www.reddit.com/r/IsaacSim/
- **용도**: 사용자 질문, 문제 해결, 예제 공유
- **활동도**: 중간

### r/reinforcementlearning
- **URL**: https://www.reddit.com/r/reinforcementlearning/
- **용도**: RL 알고리즘, 시뮬레이션 환경
- **관련성**: Isaac Sim + RL 조합

## NVIDIA Forums

### Isaac Sim 공식 포럼
- **URL**: https://forums.developer.nvidia.com/c/isaac-sim/
- **용도**: 공식 지원, 버그 리포트, 기능 요청
- **활동도**: 높음 (NVIDIA 직원 응답)

### 주요 카테고리
- Installation & Setup
- Python API
- USD & Assets
- Physics Simulation
- Reinforcement Learning

## GitHub

### NVIDIA-Omniverse/IsaacSim-dockerfiles
- **URL**: https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles
- **용도**: Docker 설치, Issues, Discussions
- **유용한 섹션**:
  - Issues: 문제 해결 사례
  - Discussions: 사용자 질문 및 답변

### 검색 키워드
- "fixed base articulation"
- "URDF import collision"
- "ArticulationRootAPI"
- "physics:fixedBase"
- "isaac sim 5.0"

## YouTube

### NVIDIA Developer
- **채널**: NVIDIA Developer
- **내용**: 공식 튜토리얼, 웨비나
- **검색**: "Isaac Sim 5.0 tutorial"

### 커뮤니티 튜토리얼
- 검색어: "Isaac Sim robot arm RL"
- 검색어: "Isaac Sim URDF import"
- 검색어: "Isaac Sim reinforcement learning"

## 예제 프로젝트

### 유사 로봇팔 프로젝트
1. **Franka Panda**
   - Isaac Sim 내장 예제
   - Fixed-base 로봇팔
   - 참고 가치 높음

2. **UR10/UR5**
   - Universal Robots
   - 산업용 로봇팔
   - URDF 공개

3. **Fetch Robot**
   - 모바일 매니퓰레이터
   - ROS/Isaac Sim 통합 예제

## 학습 자료

### 공식 튜토리얼 (로컬)
- Isaac Sim 설치 후 `Examples` 메뉴
- Standalone Python 스크립트 예제
- Omniverse Launcher → Isaac Sim → Learn

### 추천 학습 순서
1. Hello World (기본 시뮬레이션)
2. USD Stage 조작
3. Articulation 로딩 및 제어
4. Physics 설정
5. RL 환경 구축

## 유용한 검색 쿼리

### Google
```
site:forums.developer.nvidia.com isaac sim fixed base articulation
site:github.com isaac sim urdf collision api
site:reddit.com isaac sim reinforcement learning
```

### Stack Overflow
- Tag: [isaac-sim]
- Tag: [omniverse]
- Tag: [usd]

## 이전 프로젝트 참조

### codex_mcp 프로젝트
- 경로: `~/codex_mcp/`
- 참고 파일:
  - `src/envs/isaac_roarm_env.py` - 환경 구현
  - `scripts/verify_usd_roarm_m3.py` - USD 검증
  - `scripts/add_collision_api_clean.py` - CollisionAPI 추가
  - `docs/comprehensive_analysis_2025-10-15.md` - 종합 분석

## 다음 단계

- [ ] NVIDIA Forums에서 "fixed base" 검색
- [ ] Reddit에서 Isaac Sim 5.0 관련 포스트 확인
- [ ] GitHub Issues에서 유사 문제 찾기
- [ ] YouTube에서 최신 튜토리얼 확인

---

**Status**: 주요 커뮤니티 자료 소스 정리 완료
