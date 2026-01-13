# 🚀 Isaac Sim 6.0 & Isaac Lab 2.x 현황

> **마지막 업데이트**: 2026-01-13 17:55  
> **RTX 5090 드라이버**: 580.105.08 ✅

---

## 📊 현재 환경 상태

| 항목 | 현재 값 | 상태 |
|------|--------|------|
| **GPU** | NVIDIA GeForce RTX 5090 | ✅ |
| **드라이버** | 580.105.08 | ✅ Blackwell 지원 |
| **Isaac Sim Docker** | `nvcr.io/nvidia/isaac-sim:5.1.0` (15.1GB) | ✅ 실행 중 |
| **Isaac Lab** | v0.47.2 (2.3.0) | ✅ 설치 완료 |
| **환경 마이그레이션** | `run_roarm_isaac_lab.py` | ✅ 테스트 성공 |
| **Isaac Sim 6.0** | Early Developer Preview | ⏳ GA 대기 |

---

## 🔥 Isaac Sim 6.0 현황 (2026-01-13 기준)

### Early Developer Preview
- **릴리스일**: 2026년 1월 5일
- **상태**: 오픈소스 / 평가용 / 소스 빌드 필요
- **Docker 이미지**: GA 버전 출시 전까지 미제공

### 주요 신기능
1. **OmniSensor USD Schema** - 센서 시뮬레이션 성능 향상
2. **LiDAR Deskewing** - 고속 움직임 라이다 왜곡 보정
3. **Neural Reconstruction** - 사진 → 3D 변환
4. **Omniverse Fabric** - 다중 로봇 물리 연산 최적화

---

## 🛤️ 권장 마이그레이션 경로

### 옵션 A: Isaac Sim 5.1 + Isaac Lab 2.x (권장 ✅)
```bash
# 현재 Docker 이미지 유지
docker-compose -f docker-compose.isaac-sim-5.1.yml up -d

# Isaac Lab 2.x 설치 (Isaac Sim 5.1 호환)
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout v2.0.0  # 안정 버전
./isaaclab.sh --install
```

**장점**:
- 안정적인 Docker 이미지 사용
- 빠른 환경 전환
- 프로덕션 준비 완료

### 옵션 B: Isaac Sim 6.0 소스 빌드 (실험적)
```bash
# Isaac Sim 6.0 소스 클론
git clone https://github.com/isaac-sim/IsaacSim.git
cd IsaacSim

# 빌드 (RTX 5090 호환 확인 필요)
./build.sh
```

**주의**:
- Early Developer Preview - 안정성 미보장
- Docker 이미지 미제공
- 빌드 시간 2-4시간 소요

---

## 📋 추천 액션 플랜

### Phase 1: Isaac Lab 2.x 먼저 도입 (현재 추천!)
1. ✅ Isaac Sim 5.1.0 Docker 유지
2. ⏳ Isaac Lab 2.x 설치 및 환경 전환
3. ⏳ 기존 Stable-Baselines3 → Isaac Lab RL 마이그레이션

### Phase 2: Isaac Sim 6.0 GA 대기
- **예상 GA 릴리스**: 2026년 Q1-Q2
- GA 릴리스 시 Docker 이미지 자동 제공
- RTX 5090 완전 최적화

---

## 🔧 즉시 실행 가능한 명령어

### Isaac Lab 2.x 설치 (Isaac Sim 5.1 환경)
```bash
# 1. Docker 컨테이너 진입
docker exec -it isaac-sim-5.1 /bin/bash

# 2. Isaac Lab 클론
cd /workspace
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# 3. 설치
./isaaclab.sh --install

# 4. 설치 검증
python -c "import omni.isaac.lab; print('Isaac Lab installed!')"
```

### 현재 Docker 이미지 확인
```bash
docker images | grep isaac-sim
# 결과: nvcr.io/nvidia/isaac-sim:5.1.0 (15.1GB) ✅
```

---

## 📚 참고 자료

- [Isaac Sim 6.0 Developer Preview](https://github.com/isaac-sim/IsaacSim)
- [Isaac Lab GitHub](https://github.com/isaac-sim/IsaacLab)
- [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/)

---

*작성: 2026-01-13 자비스*

---

## 📝 Isaac Lab 2.3.0 마이그레이션 가이드

### 설치 완료 항목 (2026-01-13)
- ✅ Isaac Lab v0.47.2 (tag: v2.3.0)
- ✅ DirectRLEnv, Articulation, Camera imports 검증
- ✅ h5py 추가 설치
- ✅ 예제 환경 생성: `envs/roarm_isaac_lab_env.py`

### 코드 비교: Gymnasium vs Isaac Lab

#### 환경 초기화

```python
# 기존 (Gymnasium)
class SimpleVisionEnv(gym.Env):
    def __init__(self):
        self.observation_space = spaces.Box(0, 1, (4, 84, 84))
        self.action_space = spaces.Box(-1, 1, (7,))

# Isaac Lab (DirectRLEnv)
@configclass
class RoArmEnvCfg(DirectRLEnvCfg):
    num_observations = 4 * 84 * 84
    num_actions = 7

class RoArmIsaacLabEnv(DirectRLEnv):
    cfg: RoArmEnvCfg
```

#### 로봇 로딩

```python
# 기존 (Isaac Sim Native)
from isaacsim.core.prims import SingleArticulation
add_reference_to_stage(usd_path=..., prim_path=...)
robot = SingleArticulation(prim_path='/World/Robot')

# Isaac Lab
robot: ArticulationCfg = ArticulationCfg(
    prim_path="{ENV_NS}/Robot",
    spawn=UrdfFileCfg(asset_path="robot.urdf"),
    actuators={...},
)
```

#### 카메라 설정

```python
# 기존
from isaacsim.sensors.camera import Camera
camera = Camera(prim_path="/World/Camera", resolution=(84, 84))

# Isaac Lab
camera: CameraCfg = CameraCfg(
    prim_path="{ENV_NS}/Robot/gripper_link/camera",
    height=84, width=84,
    data_types=["rgb", "depth"],
)
```

### 다음 단계

1. **환경 테스트**:
   ```bash
   docker exec isaac-sim-5.1 /isaac-sim/python.sh /workspace/envs/roarm_isaac_lab_env.py
   ```

2. **학습 스크립트 전환**:
   - `train_vision_sac.py` → Isaac Lab RL (`rl_games` or `rsl_rl`)

3. **기존 환경 유지**:
   - Isaac Lab 외부에서는 기존 `simple_vision_env.py` 계속 사용 가능

---

### 파일 콁스태크

| 항목 | 기존 파일 | Isaac Lab 파일 |
|------|----------|---------------|
| RL 환경 | `envs/simple_vision_env.py` | `envs/roarm_isaac_lab_env.py` |
| 학습 | `scripts/train/train_vision_sac.py` | (TBD) |
| 검증 | `scripts/test/test_isaac_lab_install.py` | ✅ |

---

*마지막 업데이트: 2026-01-13 18:15*
