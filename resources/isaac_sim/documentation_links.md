# Isaac Sim 5.0 문서 링크 모음

**수집일**: 2025년 10월 15일
**버전**: Isaac Sim 5.0

## 주요 문서 경로 (확인 필요)

### 공식 문서 (404 발생 - URL 변경됨)
- ❌ https://docs.omniverse.nvidia.com/isaacsim/latest/
- ❌ https://docs.omniverse.nvidia.com/py/isaacsim/index.html

### 대안 문서 경로 (시도 필요)
1. **Omniverse 메인**: https://docs.omniverse.nvidia.com/
2. **Isaac Sim GitHub**: https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles
3. **NVIDIA Developer**: https://developer.nvidia.com/isaac-sim

## 로컬 문서 (설치된 환경에서 확인)

### Python 패키지 문서
```bash
source ~/isaacsim-venv/bin/activate
python -c "import isaacsim; help(isaacsim)"
python -c "from isaacsim.core.prims import SingleArticulation; help(SingleArticulation)"
```

### USD 파일 예제
```bash
# Isaac Sim 설치 경로 확인
python -c "import isaacsim; print(isaacsim.__file__)"

# 예제 USD 파일 위치
find ~/isaacsim-venv -name "*.usd" | head -20
```

## 필요한 정보

### 1. Fixed-base Articulation 설정
- ArticulationRootAPI 사용법
- physics:fixedBase 속성 설정
- 예제 코드

### 2. URDF Import
- isaac-sim urdf 명령어 옵션
- CollisionAPI 자동 추가 방법
- Visual mesh 경로 설정

### 3. Physics 설정
- PhysX Scene 설정
- Gravity, Solver, Timestep
- 권장 파라미터

### 4. API 마이그레이션 (4.x → 5.0)
- omni.isaac.core.* → isaacsim.core.*
- Deprecated API 목록
- 마이그레이션 가이드

## 대안 자료 소스

### 1. 이전 프로젝트 코드
```bash
# codex_mcp 프로젝트 참조
~/codex_mcp/src/envs/isaac_roarm_env.py
~/codex_mcp/scripts/verify_usd_roarm_m3.py
```

### 2. Python Help
```python
# 설치된 패키지에서 직접 확인
from isaacsim import SimulationApp
help(SimulationApp)

from isaacsim.core.prims import SingleArticulation
help(SingleArticulation)

from pxr import UsdPhysics
help(UsdPhysics.ArticulationRootAPI)
```

### 3. 예제 코드 찾기
```bash
find ~/isaacsim-venv -name "*.py" -type f | xargs grep -l "SingleArticulation" | head -10
find ~/isaacsim-venv -name "*.py" -type f | xargs grep -l "ArticulationRootAPI" | head -10
```

## 다음 단계

1. ✅ Isaac Sim venv 활성화
2. ✅ Python help로 API 문서 확인
3. ✅ 예제 코드 검색 및 분석
4. ✅ 작동하는 코드 패턴 추출
5. ✅ 최소 작동 예제 작성

## 참고: 이전 프로젝트 분석 결과

### 사용된 API (Deprecated)
```python
from omni.isaac.core.articulations import Articulation  # ❌ 구형
```

### 권장 API (Isaac Sim 5.0)
```python
from isaacsim.core.prims import SingleArticulation  # ✅ 최신
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

## 커뮤니티 자료

### Reddit
- r/IsaacSim: 사용자 질문 및 해결책
- r/reinforcementlearning: RL + 시뮬레이션

### NVIDIA Forums
- https://forums.developer.nvidia.com/c/isaac-sim/
- 공식 지원 및 버그 리포트

### GitHub
- Issues: https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles/issues
- Discussions: 사용자 토론

---

**Status**: 공식 문서 URL 확인 필요. 로컬 문서 및 코드 분석으로 대체 진행.
