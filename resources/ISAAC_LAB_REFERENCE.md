# Isaac Lab 2.3.0 Reference Guide

> **작성일**: 2026-01-13  
> **Isaac Lab 버전**: v0.47.2 (2.3.0)  
> **Isaac Sim 버전**: 5.1.0

---

## 📚 개요

Isaac Lab은 NVIDIA Isaac Sim 위에 구축된 로봇 강화학습 프레임워크입니다.
기존 Isaac Gym의 후속으로, 더 유연하고 확장 가능한 RL 환경을 제공합니다.

### 핵심 특징
- **DirectRLEnv**: 직접 제어 RL 환경 (Gymnasium 호환)
- **ManagerBasedRLEnv**: 설정 기반 RL 환경
- **GPU 병렬화**: 수천 개의 환경 동시 실행
- **Tensorized Operations**: PyTorch 기반 빠른 연산

---

## 🔧 DirectRLEnv 구현 패턴

### 필수 Configuration

```python
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

@configclass
class MyEnvCfg(DirectRLEnvCfg):
    # 필수 필드
    decimation = 2              # 물리 스텝 / 제어 스텝
    episode_length_s = 10.0     # 에피소드 길이
    action_space = 7            # 액션 차원 (int)
    observation_space = 16      # 관측 차원 (int)
    state_space = 0             # 상태 차원 (asymmetric actor-critic용)
    
    # Scene (InteractiveSceneCfg 사용)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096, 
        env_spacing=2.0,
        replicate_physics=True
    )
    
    # Simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1/60, 
        render_interval=decimation
    )
    
    # 로봇 설정
    robot_cfg: ArticulationCfg = ROBOT_CFG.replace(
        prim_path="/World/envs/env_.*/Robot"
    )
```

### _setup_scene() 구현 (공식 패턴)

```python
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane

def _setup_scene(self):
    # 1. 로봇 생성
    self.robot = Articulation(self.cfg.robot_cfg)
    
    # 2. Ground Plane 추가
    spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
    
    # 3. 환경 복제
    self.scene.clone_environments(copy_from_source=False)
    
    # 4. CPU 시뮬레이션용 충돌 필터링
    if self.device == "cpu":
        self.scene.filter_collisions(global_prim_paths=[])
    
    # 5. Scene에 로봇 등록
    self.scene.articulations["robot"] = self.robot
    
    # 6. 조명 추가
    light_cfg = sim_utils.DomeLightCfg(intensity=2000.0)
    light_cfg.func("/World/Light", light_cfg)
```

### 다른 필수 메서드

```python
def _pre_physics_step(self, actions: torch.Tensor) -> None:
    """물리 스텝 전 액션 저장"""
    self.actions = actions.clone()

def _apply_action(self) -> None:
    """로봇에 액션 적용"""
    self.robot.set_joint_position_target(self.actions)

def _get_observations(self) -> dict:
    """관측값 반환"""
    obs = torch.cat([self.joint_pos, self.joint_vel], dim=-1)
    return {"policy": obs}

def _get_rewards(self) -> torch.Tensor:
    """보상 계산"""
    return compute_rewards(...)

def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
    """종료 조건"""
    return terminated, truncated

def _reset_idx(self, env_ids: Sequence[int] | None):
    """환경 리셋"""
    super()._reset_idx(env_ids)
    self.robot.write_joint_state_to_sim(...)
```

---

## ⚠️ 알려진 이슈 및 해결 방법

### 1. spawn_ground_plane 에러 (Headless Docker) ⭐ **핵심 이슈**

**증상**: 
```
Python argument types in Stage.GetPrimAtPath(Stage, NoneType) did not match...
```

**근본 원인**: 
- USD Stage가 `None`을 반환함 (SimulationApp 초기화 완료 전에 spawn 시도)
- DirectRLEnv의 `_setup_scene()`이 Stage 초기화 완료 전에 호출됨

**해결 방법**:

```python
# 옵션 A: Stage 생성 후 spawn (추천)
def _setup_scene(self):
    # Stage가 준비될 때까지 대기
    import omni.usd
    while omni.usd.get_context().get_stage() is None:
        import time
        time.sleep(0.1)
    
    # 이제 Ground Plane 생성 가능
    spawn_ground_plane('/World/ground', GroundPlaneCfg())

# 옵션 B: sim_utils 직접 사용 (대안)
cfg = sim_utils.GroundPlaneCfg()
cfg.func('/World/ground', cfg)

# 옵션 C: Ground Plane 없이 테스트 (버그 격리용)
# Robot만 로드하고 ground plane은 나중에 추가
```

**Isaac Lab 뺄드 스크립트 사용 (권장)**:
```bash
# Isaac Lab의 isaaclab.sh가 환경을 올바르게 설정
cd /workspace/IsaacLab
./isaaclab.sh -p scripts/tutorials/03_envs/run_cartpole_rl_env.py --num_envs 1
```

### 2. Docker에서 RTX 렌더링 실패

**증상**: `HydraEngine rtx failed creating scene renderer`

**원인**: Docker 컨테이너에서 GPU 렌더링 미지원

**해결 방법**:
- `--headless` 플래그 사용
- 렌더링이 필요한 경우 `--no-window` 대신 X11 포워딩 설정

### 3. 환경이 즉시 종료됨

**증상**: Scene 생성 후 바로 셧다운

**해결 방법**:
```python
# 메인 함수에서 시뮬레이션 루프 유지
while simulation_app.is_running():
    env.step(actions)
```

---

## 📁 프로젝트 내 파일 구조

```
envs/
├── roarm_isaac_lab_env.py     # Isaac Lab 환경 (마이그레이션 진행 중)
├── simple_vision_env.py        # 기존 Isaac Sim Native 환경
└── roarm_pick_place_env.py     # Native Pick & Place 환경

IsaacLab/                       # 설치된 Isaac Lab v0.47.2
└── source/isaaclab_tasks/
    └── direct/cartpole/        # 참고 예제
```

---

## 🔗 참고 자료

- [Isaac Lab 공식 문서](https://isaac-sim.github.io/IsaacLab/)
- [DirectRLEnv 튜토리얼](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/03_envs/create_direct_rl_env.html)
- [Isaac Lab GitHub](https://github.com/isaac-sim/IsaacLab)
- [Cartpole 예제 코드](file:///home/roarm_m3/roarm_isaac_clean/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/cartpole/cartpole_env.py)

---

## 🛠️ 현재 마이그레이션 상태

| 단계 | 상태 | 노트 |
|------|------|------|
| Isaac Lab 설치 | ✅ | v0.47.2, Docker 내 |
| 핵심 import 검증 | ✅ | DirectRLEnv, Articulation |
| Scene 설정 | 🟡 | spawn_ground_plane 이슈 |
| 환경 테스트 | ⏳ | 디버깅 필요 |

### 다음 단계

1. **Docker 컨테이너 재시작**: 깨끗한 상태에서 테스트
2. **공식 Cartpole 예제 실행**: Isaac Lab 설치 검증
3. **환경 단순화**: Ground plane 없이 먼저 테스트
4. **점진적 기능 추가**: 로봇 → Ground → 카메라

---

*마지막 업데이트: 2026-01-13 18:40*
