# 참고 자료 모음

**작성일**: 2025년 10월 15일  
**목적**: Phase 2 자료 수집 결과 정리

---

## 📊 수집 완료 현황

- ✅ **RoArm M3 공식 자료** - Wiki, 스펙, 다운로드 링크
- ✅ **Isaac Sim 문서 경로** - 로컬 환경 확인 방법
- ✅ **커뮤니티 자료 소스** - Forums, Reddit, GitHub
- ✅ **이전 프로젝트 분석** - codex_mcp 참조 자료

---

## 🤖 RoArm M3 자료

### 공식 문서
| 항목 | 링크 | 용도 |
|------|------|------|
| **공식 Wiki** | https://www.waveshare.com/wiki/RoArm-M3 | 전체 스펙, 사용법, 튜토리얼 |
| **제품 페이지** | https://www.waveshare.com/roarm-m3.htm | 제품 정보, 구매 |
| **기술 지원** | https://service.waveshare.com/ | 티켓 시스템 |
| **GitHub** | https://github.com/waveshareteam | 오픈소스 코드 |

### 다운로드 파일
| 파일 | 링크 | 내용 |
|------|------|------|
| **3D STEP 모델** | [RoArm-M3_STEP.zip](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_STEP.zip) | CAD 파일 |
| **2D 치수도** | [RoArm-M3_2Dsize.zip](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_2Dsize.zip) | 도면 |
| **오픈소스 프로그램** | [RoArm-M3_example.zip](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_example-250108.zip) | ESP32 펌웨어, 예제 |
| **Python Demo** | [RoArm-M3_Python.zip](https://files.waveshare.com/wiki/RoArm-M3/RoArm-M3_Python.zip) | Python 제어 예제 |

### 주요 스펙 (요약)
```
DOF: 5+1 (5개 관절 + 그리퍼)
작업 공간: 직경 1m
유효 하중: 0.2kg @ 0.5m
제어: JSON 명령, HTTP/Serial/ROS2
서보: ST3215 (S) / ST3235 (Pro)
정확도: 0.088° (12-bit 엔코더)
전원: DC 7-12.6V (권장 12V 5A)
```

**상세**: `resources/roarm_m3/waveshare_wiki_summary.md`

---

## 🎮 Isaac Sim 5.0 자료

### 공식 문서 (URL 변경됨 - 확인 필요)
| 항목 | 상태 | 비고 |
|------|------|------|
| docs.omniverse.nvidia.com/isaacsim/latest/ | ❌ 404 | URL 변경됨 |
| docs.omniverse.nvidia.com/py/isaacsim/ | ❌ 404 | URL 변경됨 |
| developer.nvidia.com/isaac-sim | ✅ 확인 필요 | 메인 페이지 |

### 로컬 문서 확인 방법
```bash
# Isaac Sim venv 활성화
source ~/isaacsim-venv/bin/activate

# 설치 경로 확인
python -c "import isaacsim; print(isaacsim.__file__)"

# USD 파일 예제 찾기
find ~/isaacsim-venv -name "*.usd" | head -20

# Python API Help
python -c "from pxr import UsdPhysics; help(UsdPhysics.ArticulationRootAPI)"
```

### 주요 API (Isaac Sim 5.0)
```python
# 최신 API
from isaacsim import SimulationApp
from isaacsim.core.prims import SingleArticulation

# USD Physics
from pxr import Usd, UsdPhysics, Sdf

# Deprecated (사용 금지)
# from omni.isaac.core.articulations import Articulation  # ❌
```

### Fixed Base 설정 (추정)
```python
from pxr import Usd, UsdPhysics, Sdf

stage = Usd.Stage.Open("robot.usd")
robot_prim = stage.GetPrimAtPath("/World/robot")

# ArticulationRootAPI 적용
UsdPhysics.ArticulationRootAPI.Apply(robot_prim)

# Fixed base 설정
robot_prim.CreateAttribute(
    "physics:fixedBase", 
    Sdf.ValueTypeNames.Bool
).Set(True)

stage.Save()
```

**상세**: `resources/isaac_sim/documentation_links.md`

---

## 🌐 커뮤니티 자료

### NVIDIA 공식
| 플랫폼 | URL | 용도 |
|--------|-----|------|
| **Forums** | https://forums.developer.nvidia.com/c/isaac-sim/ | 공식 지원, 버그 리포트 |
| **GitHub** | https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles | Issues, Discussions |
| **YouTube** | NVIDIA Developer 채널 | 공식 튜토리얼 |

### 커뮤니티
| 플랫폼 | URL | 용도 |
|--------|-----|------|
| **Reddit** | https://www.reddit.com/r/IsaacSim/ | 사용자 질문, 해결책 |
| **Reddit** | https://www.reddit.com/r/reinforcementlearning/ | RL + 시뮬레이션 |
| **Stack Overflow** | Tag: [isaac-sim], [omniverse] | Q&A |

### 유용한 검색 쿼리
```
site:forums.developer.nvidia.com isaac sim fixed base articulation
site:github.com isaac sim urdf collision api
site:reddit.com isaac sim reinforcement learning
"isaac sim 5.0" fixed base
ArticulationRootAPI physics:fixedBase
```

**상세**: `resources/community/isaac_sim_resources.md`

---

## 📚 이전 프로젝트 (codex_mcp)

### 주요 문서
| 문서 | 경로 | 내용 |
|------|------|------|
| **종합 분석** | `~/codex_mcp/docs/comprehensive_analysis_2025-10-15.md` | 전체 이슈 분석 |
| **현재 상태** | `~/codex_mcp/docs/STATUS.md` | 최신 진행 상황 |
| **아키텍처** | `~/codex_mcp/docs/ARCH_DECISION_DUAL_ENV.md` | Dual Environment |
| **Sim2Real** | `~/codex_mcp/docs/SIM2REAL_GUIDE.md` | 시뮬-실제 전이 |

### 참고 코드
| 파일 | 경로 | 용도 |
|------|------|------|
| **환경 구현** | `~/codex_mcp/src/envs/isaac_roarm_env.py` | RL 환경 |
| **USD 검증** | `~/codex_mcp/scripts/verify_usd_roarm_m3.py` | 검증 스크립트 |
| **CollisionAPI** | `~/codex_mcp/scripts/add_collision_api_clean.py` | CollisionAPI 추가 |
| **학습 스크립트** | `~/codex_mcp/training/train_ppo.py` | PPO 학습 |

### 주요 발견사항
- ⚡ **USD CollisionAPI 누락** → PhysX 크래시
- 🏗️ **Base Link 고정 실패** → 로봇 회전/튕김
- 🖥️ **원격 GUI 렌더링 실패** → WebRTC 필요

**상세**: `docs/LESSONS_LEARNED.md`

---

## 🔍 예제 프로젝트

### Isaac Sim 내장 로봇팔
1. **Franka Panda**
   - Fixed-base 로봇팔
   - Isaac Sim 예제 포함
   - 참고 가치 ⭐⭐⭐⭐⭐

2. **UR10/UR5**
   - Universal Robots
   - 산업용 로봇팔
   - URDF 공개

3. **Fetch Robot**
   - 모바일 매니퓰레이터
   - ROS/Isaac Sim 통합

### LeRobot 프로젝트
- **URL**: https://github.com/huggingface/lerobot
- **특징**: RoArm M3 지원
- **내용**: 사전 학습 모델, 데이터셋, 시뮬레이션

---

## 📖 학습 순서 (권장)

### 1단계: 기본 개념
- [ ] USD Stage 이해
- [ ] Prim, Attribute, Relationship
- [ ] ArticulationAPI 개념

### 2단계: 로봇 로딩
- [ ] URDF → USD 변환
- [ ] CollisionAPI 추가
- [ ] Visual/Collision Geometry

### 3단계: Physics 설정
- [ ] PhysX Scene 구성
- [ ] ArticulationRootAPI 적용
- [ ] Fixed Base 설정

### 4단계: 제어
- [ ] Joint Position 제어
- [ ] Joint Velocity 제어
- [ ] Torque 제어

### 5단계: RL 환경
- [ ] Observation Space
- [ ] Action Space
- [ ] Reward Function
- [ ] Stable-Baselines3 통합

---

## 🎯 Phase 2 완료 기준

- [x] RoArm M3 Wiki 요약 완료
- [x] STEP 파일, 치수도 다운로드 링크 확보
- [x] ROS2 튜토리얼 존재 확인
- [x] Isaac Sim 로컬 문서 확인 방법 파악
- [x] Fixed Base 설정 방법 추정 코드 작성
- [x] 커뮤니티 자료 소스 정리
- [x] REFERENCES.md 작성 완료

---

## 🚀 다음 단계 (Phase 3)

### 목표: USD 파일 올바른 생성

**작업 항목:**
1. RoArm M3 오픈소스 프로그램 다운로드
2. ROS2 관련 파일에서 URDF 확인
3. URDF 검증 (collision, visual, joint)
4. Isaac Sim으로 USD 변환
5. CollisionAPI 추가 스크립트 작성
6. Fixed Base 설정
7. 검증 스크립트 실행

**예상 소요 시간**: 2-4시간

---

## 📝 메모

### 발견한 중요 정보
1. ✅ RoArm M3는 ROS2 지원 → URDF 파일 존재 가능성 높음
2. ✅ STEP 3D 모델 제공 → 필요시 수동 USD 작성 가능
3. ✅ LeRobot 통합 → 사전 학습 모델 활용 가능
4. ⚠️ Isaac Sim 5.0 공식 문서 URL 변경됨 → 로컬 확인 필요
5. ✅ 이전 프로젝트 코드 참조 가능 (codex_mcp)

### 주의사항
- ⚠️ CollisionAPI는 모든 링크에 필수
- ⚠️ Fixed Base는 ArticulationRootAPI + physics:fixedBase
- ⚠️ 구형 API (omni.isaac.core.*) 사용 금지
- ⚠️ URDF → USD 변환 후 즉시 검증 필수

---

**작성 완료**: 2025년 10월 15일  
**Status**: ✅ Phase 2 (자료 수집) 완료  
**Next**: Phase 3 (USD 파일 생성) 준비
