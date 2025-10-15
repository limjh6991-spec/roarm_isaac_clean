# 이전 프로젝트 주요 이슈 및 해결 방안

**작성일**: 2025년 10월 15일  
**출처**: ~/codex_mcp 프로젝트 분석  
**목적**: 동일한 문제 재발 방지

---

## 🔴 Critical Issues (즉시 해결 필요)

### Issue #1: USD CollisionAPI 누락 → PhysX 세그폴트

**우선순위**: ⚡ **최우선** (P0)  
**영향도**: 🔴 **치명적** (시뮬레이션 불가능)

#### 상세 설명
```
증상:
- Isaac Sim GUI에서 타임라인 재생 버튼 클릭 시 즉시 크래시
- Headless 모드에서 타임라인 자동 재생 시 세그폴트
- PhysX 초기화 실패

에러 메시지:
[WARNING] Invalid PhysX transform detected for /World/roarm/roarm/link_*
[ERROR] Illegal BroadPhaseUpdateData
[FATAL] Segmentation fault (core dumped)
```

#### 근본 원인
```python
# USD 검증 결과
CollisionAPI missing on: /World/roarm/base_link
CollisionAPI missing on: /World/roarm/link_base
CollisionAPI missing on: /World/roarm/link_shoulder
CollisionAPI missing on: /World/roarm/link_elbow
CollisionAPI missing on: /World/roarm/link_wrist1
CollisionAPI missing on: /World/roarm/link_wrist2
CollisionAPI missing on: /World/roarm/link_gripper

총 7개 링크 모두 CollisionAPI 없음
→ PhysX BroadPhase가 충돌 형상 초기화 실패
→ 시뮬레이션 시작 불가능
```

#### 발생 경로
```
URDF 파일 (roarm_m3.urdf)
    ↓
isaac-sim urdf 변환 도구
    ↓
USD 파일 생성 (roarm_m3.generated.usd)
    ↓ ❌ CollisionAPI 누락
PhysX 초기화 시도
    ↓
❌ 세그폴트
```

#### 해결 방안

**방법 1: Isaac Sim GUI에서 수동 추가 (권장)**
```
1. Isaac Sim GUI 실행
2. File → Open → <USD 파일>
3. Stage 창에서 각 링크 선택
4. 우클릭 → Add → Physics → Collision Box (또는 Collision Mesh)
5. Property 창에서 크기 조정
6. 모든 링크 반복 (7개)
7. File → Save As → 새 파일명
```

**방법 2: Python 스크립트로 자동 추가**
```python
from pxr import Usd, UsdGeom, UsdPhysics

stage = Usd.Stage.Open("input.usd")
links = [
    "base_link", "link_base", "link_shoulder", 
    "link_elbow", "link_wrist1", "link_wrist2", "link_gripper"
]

for link_name in links:
    prim = stage.GetPrimAtPath(f"/World/roarm/{link_name}")
    
    # Collision Box 추가
    collision_prim = stage.DefinePrim(
        f"/World/roarm/{link_name}/collision",
        "Cube"
    )
    UsdPhysics.CollisionAPI.Apply(collision_prim)
    
    # 크기 설정 (실제 링크 크기에 맞게 조정 필요)
    UsdGeom.Cube(collision_prim).CreateSizeAttr(1.0)

stage.GetRootLayer().Save()
stage.Export("output.usd")
```

**방법 3: URDF 수정 후 재변환**
```xml
<!-- URDF에 collision 태그 추가 -->
<link name="base_link">
  <collision>
    <geometry>
      <box size="0.1 0.1 0.05"/>
    </geometry>
  </collision>
  <visual>
    ...
  </visual>
</link>
```

#### 검증 방법
```bash
# USD 구조 검증 스크립트 실행
python scripts/verify_usd_structure.py --usd <output.usd>

# 예상 출력:
# ✅ CollisionAPI found on /World/roarm/base_link
# ✅ CollisionAPI found on /World/roarm/link_base
# ...
# ✅ All 7 links have CollisionAPI
```

#### 예방 조치
- ✅ USD 변환 후 즉시 검증 스크립트 실행
- ✅ CI/CD에 검증 단계 포함
- ✅ 변환 프로세스 문서화

---

### Issue #2: 로봇 Base Link 고정 실패

**우선순위**: 🔴 **높음** (P1)  
**영향도**: 🟡 **중간** (제어 불가능)

#### 상세 설명
```
증상:
- 재생 시작하면 로봇이 회전하며 튕겨나감
- 중력에 의해 바닥으로 낙하
- 제어 명령이 적용되지 않음

시도한 해결책:
1. Kinematic Body로 base_link 설정
   → 에러: "Articulations with kinematic bodies are not supported"
   
2. Fixed Joint로 World에 고정
   → 에러: "Cannot create a joint between static bodies"
   
3. Ground Plane 추가
   → 낙하는 방지했으나 회전 문제 지속
```

#### 근본 원인
- Isaac Sim 5.0의 Articulation 시스템에서 fixed-base robot 설정 방법 미숙지
- URDF → USD 변환 시 base link 고정 정보 손실
- ArticulationRootAPI 적용 방법 불명확

#### 올바른 해결 방안 (Isaac Sim 5.0)

**방법 1: ArticulationRootAPI + physics:fixedBase 속성**
```python
from pxr import Usd, UsdPhysics
from isaacsim.core.prims import SingleArticulation

# USD에서 설정
stage = Usd.Stage.Open("robot.usd")
robot_prim = stage.GetPrimAtPath("/World/robot")

# ArticulationRootAPI 적용
articulation_api = UsdPhysics.ArticulationRootAPI.Apply(robot_prim)

# Fixed base 설정
robot_prim.CreateAttribute("physics:fixedBase", Sdf.ValueTypeNames.Bool).Set(True)

stage.Save()
```

**방법 2: SingleArticulation API 사용**
```python
from isaacsim.core.prims import SingleArticulation

# 런타임에서 설정
articulation = SingleArticulation(
    prim_path="/World/robot",
    position=np.array([0, 0, 1.0])
)

# Fixed base 설정 (API 확인 필요)
articulation.set_fixed_base(True)
```

#### 검증 방법
```python
# Base link가 고정되었는지 확인
def verify_fixed_base(prim_path):
    from pxr import Usd, UsdPhysics
    
    stage = Usd.Stage.Open("robot.usd")
    robot_prim = stage.GetPrimAtPath(prim_path)
    
    # ArticulationRootAPI 존재 확인
    if not UsdPhysics.ArticulationRootAPI(robot_prim):
        print("❌ ArticulationRootAPI not applied")
        return False
    
    # fixedBase 속성 확인
    fixed_base_attr = robot_prim.GetAttribute("physics:fixedBase")
    if not fixed_base_attr or not fixed_base_attr.Get():
        print("❌ physics:fixedBase not set to True")
        return False
    
    print("✅ Fixed base correctly configured")
    return True
```

#### 필요한 자료 조사
- [ ] Isaac Sim 5.0 공식 문서: Fixed-base Articulation 설정
- [ ] 예제 코드: Franka Panda, UR10 등 고정형 로봇팔
- [ ] API 레퍼런스: `SingleArticulation.set_fixed_base()` 존재 여부

---

### Issue #3: 원격 GUI 렌더링 실패

**우선순위**: 🟡 **중간** (P2)  
**영향도**: 🟡 **중간** (시각 확인 불가)

#### 상세 설명
```
증상:
- SSH X11 포워딩으로 Isaac Sim GUI 실행
- 창은 뜨지만 내부가 검은 화면/회색 화면
- 콘솔 로그는 정상 진행

시도한 해결책:
1. VirtualGL 설치 → Ubuntu 24.04 의존성 충돌
2. 환경변수 조정 (KIT_USE_EGL=0 등) → 실패
3. X11 포워딩 옵션 조정 → 실패
```

#### 근본 원인
- X11 포워딩은 고성능 3D 렌더링(OpenGL/Vulkan) 지원 제한적
- Isaac Sim Omniverse UI는 GPU 직접 렌더링 요구
- 네트워크 대역폭 한계

#### 해결 방안

**방법 1: WebRTC 라이브 스트리밍 (권장)**
```python
# Isaac Sim 설정
CONFIG = {
    "livestream": {
        "enabled": True,
        "port": 8211,
        "encoder": "h264",  # 또는 "h265"
        "bitrate_kbps": 20000
    }
}

from isaacsim import SimulationApp
app = SimulationApp(CONFIG)

# 브라우저에서 접속: https://<서버IP>:8211/streaming/webrtc-client
```

**방법 2: NICE DCV (AWS 권장)**
```bash
# 설치
wget https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu2404-x86_64.tgz
tar -xvzf nice-dcv-ubuntu2404-x86_64.tgz
cd nice-dcv-*
sudo apt-get install ./nice-dcv-server_*.deb

# 세션 시작
dcv create-session --type=virtual my-session

# 클라이언트에서 접속: https://<서버IP>:8443
```

**방법 3: Headless + 메트릭/로그 (실용적)**
```python
# Headless 모드로 실행
CONFIG = {"headless": True}
app = SimulationApp(CONFIG)

# 메트릭 수집
metrics = {
    "episode_reward": [],
    "joint_positions": [],
    "collision_events": []
}

# 주기적으로 스크린샷/영상 저장
from omni.kit.capture import Capture
capture = Capture()
capture.screenshot("output.png")
```

#### 우선순위 권장
1. **단기**: Headless + 로그/메트릭 (즉시 가능)
2. **중기**: WebRTC 스트리밍 설정 (1일 소요)
3. **장기**: NICE DCV 설치 (안정적, 2일 소요)

---

## 🟡 High Priority Issues

### Issue #4: 구형 API 사용 (omni.isaac.core.*)

**우선순위**: 🟡 **중간** (P2)  
**영향도**: 🟢 **낮음** (기능 동작, 종료 시 에러)

#### 문제
```python
# 현재 코드 (codex_mcp)
from omni.isaac.core.articulations import Articulation  # ❌ Deprecated

self._articulation = Articulation(
    prim_path=self._prim_path,
    name="roarm"
)

# 에러 (종료 시)
AttributeError: 'Articulation' object has no attribute '_callbacks'
```

#### 해결
```python
# Isaac Sim 5.0 권장 API
from isaacsim.core.prims import SingleArticulation  # ✅ Latest

self._articulation = SingleArticulation(
    prim_path=self._prim_path
    # name 파라미터 제거 (자동 추론)
)
```

#### 마이그레이션 가이드
```
omni.isaac.core.articulations.Articulation
  → isaacsim.core.prims.SingleArticulation

omni.isaac.core.world.World
  → isaacsim.core.simulation.SimulationContext

omni.isaac.core.objects.*
  → isaacsim.core.prims.*
```

---

### Issue #5: Visual Mesh 누락

**우선순위**: 🟡 **중간** (P2)  
**영향도**: 🟢 **낮음** (기능 동작, 시각만 단순)

#### 문제
- 로봇이 단순한 막대/박스 형태로 표시
- 실제 CAD 모델의 외관이 보이지 않음

#### 원인
- URDF의 `<visual>` 태그에 mesh 경로 설정 누락 또는 잘못됨
- USD 변환 시 mesh 파일 경로 해석 실패

#### 해결
```xml
<!-- URDF 수정 -->
<visual>
  <geometry>
    <mesh filename="package://roarm_m3_description/meshes/base_link.stl"/>
  </geometry>
</visual>
```

```bash
# USD 변환 시 mesh 경로 확인
isaac-sim urdf \
  --input roarm_m3.urdf \
  --output roarm_m3.usd \
  --merge-fixed-joints \
  --import-inertia-tensor
```

---

### Issue #6: 학습 환경과 재생 환경 불일치

**우선순위**: 🟡 **중간** (P2)  
**영향도**: 🟡 **중간** (성능 저하)

#### 문제
```
학습:
- 10,000 스텝 완료
- 정책 저장 성공

재생:
- 500 스텝 실행
- 누적 보상: -2644.434 (매우 낮음)
- 목표 달성률: 0%
```

#### 원인 추정
1. **Physics 설정 불일치**
   - 학습 시: Headless 모드, 특정 확장 비활성화
   - 재생 시: GUI 모드, 모든 확장 활성화
   
2. **Observation Space 불일치**
   - 조인트 순서 변경
   - 스케일링 차이
   - 추가/누락 관측값

3. **Domain Randomization**
   - 학습: 랜덤라이제이션 적용
   - 재생: 고정 파라미터 사용

#### 해결
```python
# 1. 동일한 USD 파일 사용
TRAIN_USD = "assets/roarm_m3.usd"
EVAL_USD = "assets/roarm_m3.usd"  # 동일!

# 2. Physics 설정 명시적 기록
physics_config = {
    "gravity": [0, 0, -9.81],
    "solver_type": "TGS",
    "timestep": 1/120.0
}
save_config("physics_config.yaml", physics_config)

# 3. Observation 버전 관리
OBS_VERSION = "v1.0"
obs_schema = {
    "version": OBS_VERSION,
    "joint_names": ["joint1", "joint2", ...],
    "normalization": {"mean": [...], "std": [...]}
}
save_config("obs_schema.yaml", obs_schema)

# 4. Domain Randomization 비활성화 (재생 시)
env = IsaacRoArmEnv(
    domain_randomizer=None  # 재생 시 None
)
```

---

## 🟢 Medium Priority Issues

### Issue #7: IPC 레이턴시 미측정

**우선순위**: 🟢 **낮음** (P3)  
**영향도**: 🟡 **중간** (성능 최적화)

#### 상황
- 정책 추론 레이턴시만 측정됨
- Isaac Sim step + 전체 루프 레이턴시 미측정
- 목표: p95 < 60ms

#### 해결
```python
import time
import numpy as np

latencies = []

while running:
    start = time.perf_counter()
    
    # 1. Isaac Sim step
    obs = env.get_observation()
    
    # 2. IPC 전송
    obs_json = serialize(obs)
    
    # 3. 정책 추론
    action = policy.predict(obs)
    
    # 4. IPC 수신
    action_json = deserialize(action)
    
    # 5. Isaac Sim 적용
    env.apply_action(action)
    
    end = time.perf_counter()
    latencies.append((end - start) * 1000)  # ms
    
    # 통계 출력
    if len(latencies) >= 100:
        print(f"p50: {np.percentile(latencies, 50):.2f}ms")
        print(f"p95: {np.percentile(latencies, 95):.2f}ms")
        latencies = []
```

---

### Issue #8: TODO/FIXME 주석 65개

**우선순위**: 🟢 **낮음** (P3)  
**영향도**: 🟢 **낮음** (코드 품질)

#### 통계
```
총 65개
- TODO: 45개
- FIXME: 12개
- debug_trace: 8개

핵심 TODO:
- isaac_roarm_env.py line 188: Stage 로드 및 articulation 재획득
- isaac_roarm_env.py line 338: get_state() 구현
- isaac_controller_server.py: MCP 서버 Isaac 연동 (7개)
```

#### 조치 계획
1. 핵심 TODO 우선 처리 (P1-P2)
2. debug_trace 제거 (릴리스 전)
3. 일반 TODO 정리 (시간 있을 때)

---

## 📋 이슈 요약 테이블

| 이슈 | 우선순위 | 영향도 | 상태 | 해결 방안 |
|------|---------|-------|------|----------|
| #1 CollisionAPI 누락 | P0 | 🔴 치명적 | Open | USD에 CollisionAPI 추가 |
| #2 Base Link 고정 실패 | P1 | 🟡 중간 | Open | ArticulationRootAPI + fixedBase |
| #3 원격 GUI 렌더링 | P2 | 🟡 중간 | Open | WebRTC 스트리밍 설정 |
| #4 구형 API 사용 | P2 | 🟢 낮음 | Open | isaacsim.core.* 마이그레이션 |
| #5 Visual Mesh 누락 | P2 | 🟢 낮음 | Open | URDF mesh 경로 수정 |
| #6 환경 불일치 | P2 | 🟡 중간 | Open | 설정 명시적 기록/재사용 |
| #7 IPC 레이턴시 | P3 | 🟡 중간 | Open | End-to-end 측정 추가 |
| #8 TODO 정리 | P3 | 🟢 낮음 | Open | 단계적 정리 |

---

## 🎯 다음 프로젝트 액션 아이템

### Phase 1: Critical Issues (1-2일)
- [ ] Isaac Sim 5.0 공식 문서 조사 (fixed-base articulation)
- [ ] RoArm M3 URDF/USD 자료 수집
- [ ] USD 변환 + CollisionAPI 추가 스크립트 작성
- [ ] 검증 스크립트 작성 및 테스트

### Phase 2: High Priority (3-5일)
- [ ] WebRTC 스트리밍 설정 (또는 Headless + 메트릭)
- [ ] 최신 API 마이그레이션
- [ ] Base link 고정 검증

### Phase 3: Medium Priority (1주)
- [ ] IPC 레이턴시 측정 시스템
- [ ] 환경 설정 버전 관리
- [ ] Visual mesh 개선

---

**마지막 업데이트**: 2025년 10월 15일  
**다음 검토**: Phase 1 완료 후
