# Pick-and-Place 환경 진단 체크리스트

**목적**: 강화학습 환경에서 로봇이 큐브를 인식하지 못하거나 상호작용하지 못할 때 체계적으로 진단

**참조**: Isaac Sim/Isaac Lab 전문가 프롬프트 기반

---

## 🔍 진단 프로세스 개요

```
1. Observation Check      → 큐브 정보가 관측에 포함되는가?
2. Coordinate Frame       → 좌표계가 일치하는가?
3. Camera Validation      → (Vision 사용 시) 카메라가 작동하는가?
4. Physics & Collision    → 물리 충돌이 올바르게 설정되었는가?
5. Reward Signal          → 보상이 학습을 유도하는가?
6. Task Integration       → 태스크가 올바르게 등록되었는가?
7. Runtime Diagnostics    → 실시간 데이터가 정상인가?
```

---

## ✅ 1. Observation Check

### 1.1 문제 증상
- 로봇이 큐브를 무시함
- 에피소드 동안 무작위 동작만 수행
- TensorBoard에서 보상이 -0.01 (time penalty)만 누적

### 1.2 진단 단계

#### Step 1: Observation 키 확인
```python
# scripts/rl/train_dense_reward.py 또는 env.py

def get_observations(self):
    obs = self._compute_observations()
    print("Available observation keys:", obs.keys())
    return obs
```

**Expected Output**:
```
Available observation keys: dict_keys(['policy', 'critic'])
# 또는
Available observation keys: dict_keys(['joint_pos', 'joint_vel', 'ee_pos', 'cube_pos', 'goal_pos'])
```

#### Step 2: 큐브 위치 포함 확인
```python
# env.py의 _compute_observations()에서

obs = {
    "joint_pos": self.robot.data.joint_pos,
    "joint_vel": self.robot.data.joint_vel,
    "ee_pos": self.ee_frame.data.target_pos_w[..., 0, :],
    
    # ✅ 큐브 정보 반드시 포함
    "cube_pos": self.cube.data.root_pos_w,
    "cube_quat": self.cube.data.root_quat_w,
    
    "goal_pos": self.goal_pos,
}

# 정책 관측 구성
policy_obs = torch.cat([
    obs["joint_pos"],
    obs["joint_vel"],
    obs["ee_pos"],
    obs["cube_pos"],      # ⚠️ 누락 시 로봇이 큐브 인식 불가!
    obs["goal_pos"],
], dim=-1)

return {"policy": policy_obs}
```

### 1.3 패치 예시

**문제**: `cube_pos`가 관측에 없음

**해결**:
```python
# env_cfg.py 또는 task config

# ❌ Before
env_cfg.observations.policy.obs_keys = ["joint_pos", "joint_vel", "ee_pos", "goal_pos"]

# ✅ After
env_cfg.observations.policy.obs_keys = [
    "joint_pos", "joint_vel", "ee_pos",
    "cube_pos", "cube_quat",  # 추가!
    "goal_pos"
]
```

---

## ✅ 2. Coordinate Frame Alignment

### 2.1 문제 증상
- 로봇이 큐브 위치로 이동하지만 빗나감
- 큐브가 예상 위치에 없음
- Observation 값이 이상함 (예: cube_pos = [100, 50, 30])

### 2.2 진단 단계

#### Step 1: 좌표계 확인
```python
# env.py

def _compute_observations(self):
    # World 좌표계
    cube_world_pos = self.cube.data.root_pos_w
    ee_world_pos = self.ee_frame.data.target_pos_w[..., 0, :]
    
    print(f"Cube (World): {cube_world_pos[0]}")
    print(f"EE (World): {ee_world_pos[0]}")
    
    # ⚠️ 둘 다 같은 좌표계(/World)인지 확인!
```

**Expected Output**:
```
Cube (World): tensor([0.20, 0.10, 0.05])  # 현실적인 값
EE (World): tensor([0.15, 0.05, 0.10])
```

#### Step 2: 상대 좌표 추가 (권장)
```python
# 로봇 중심 좌표계로 변환
robot_base_pos = self.robot.data.root_pos_w

cube_rel_pos = cube_world_pos - robot_base_pos
ee_rel_pos = ee_world_pos - robot_base_pos

obs["cube_rel_pos"] = cube_rel_pos
obs["ee_rel_pos"] = ee_rel_pos
```

### 2.3 패치 예시

**문제**: EE는 로봇 로컬 좌표, 큐브는 월드 좌표

**해결**:
```python
# ❌ Before (혼합 좌표계)
ee_local = self.robot.data.body_pos_w[:, ee_body_idx, :]  # Local
cube_world = self.cube.data.root_pos_w                     # World

# ✅ After (통일)
ee_world = self.ee_frame.data.target_pos_w[..., 0, :]
cube_world = self.cube.data.root_pos_w

# 또는 모두 상대 좌표로
base_pos = self.robot.data.root_pos_w
obs["ee_pos"] = ee_world - base_pos
obs["cube_pos"] = cube_world - base_pos
```

---

## ✅ 3. Camera Validation (Vision-based만 해당)

### 3.1 문제 증상
- 카메라 이미지가 검은색
- RGB/Depth 데이터가 비어 있음
- Vision-based 정책이 학습 안 됨

### 3.2 진단 단계

#### Step 1: 카메라 활성화 확인
```python
# env.py

def __init__(self):
    # 카메라 정의
    self.camera = Camera(
        prim_path="/World/envs/env_0/Camera",
        resolution=(256, 256),
        data_types=["rgb", "depth"],
    )
    
    # ✅ Render product 초기화 필수
    self.camera.initialize()
    print(f"Camera initialized: {self.camera.is_initialized}")
```

#### Step 2: FOV 및 위치 확인
```python
# Camera config
camera_cfg = CameraCfg(
    prim_path="/World/envs/env_.*/Camera",
    offset=CameraCfg.OffsetCfg(
        pos=(0.5, 0.0, 0.3),     # 카메라 위치
        rot=(0.0, -30.0, 0.0),   # 아래 30도 각도
    ),
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=24.0,
        horizontal_aperture=20.955,
        clipping_range=(0.1, 10.0),
    ),
)

# ⚠️ 큐브가 FOV 내에 있는지 확인!
```

#### Step 3: 이미지 시각화
```python
# 첫 프레임에 이미지 저장
if self.step_count == 0:
    rgb = self.camera.data.output["rgb"][0].cpu().numpy()
    cv2.imwrite("/tmp/debug_camera.png", rgb)
    print("Camera image saved to /tmp/debug_camera.png")
```

### 3.3 패치 예시

**문제**: 카메라 검은 화면

**해결**:
```python
# ❌ Before (render product 없음)
camera = Camera(prim_path="/World/Camera")

# ✅ After
camera = Camera(
    prim_path="/World/envs/env_.*/Camera",
    resolution=(256, 256),
    data_types=["rgb", "depth"],
)
camera.initialize()  # 필수!

# Simulation loop에서
camera.update(dt=sim.get_physics_dt())
```

---

## ✅ 4. Physics and Collision

### 4.1 문제 증상
- 큐브가 바닥을 뚫고 떨어짐
- 그리퍼가 큐브를 통과함
- 큐브가 움직이지 않음 (Static)

### 4.2 진단 단계

#### Step 1: USD Stage 확인
```python
# Isaac Sim GUI에서 확인
# Window > Browsers > Stage

# 큐브 Prim 선택 후:
# - Physics > RigidBodyAPI: Enabled
# - Physics > CollisionAPI: Enabled
# - Physics > Mass: > 0 (예: 0.1 kg)
```

#### Step 2: 코드로 확인
```python
# env.py

def _setup_scene(self):
    # 큐브 생성
    self.cube = RigidObject(
        cfg=RigidObjectCfg(
            prim_path="/World/envs/env_.*/Cube",
            spawn=sim_utils.CuboidCfg(
                size=(0.05, 0.05, 0.05),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    kinematic_enabled=False,     # ✅ Dynamic
                    disable_gravity=False,
                    max_depenetration_velocity=1.0,
                ),
                mass_props=sim_utils.MassPropertiesCfg(
                    mass=0.1,                    # ✅ 100g
                ),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True,      # ✅ 충돌 활성화
                ),
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    static_friction=0.6,         # 마찰
                    dynamic_friction=0.5,
                    restitution=0.0,             # 반발 계수
                ),
            ),
        )
    )
```

### 4.3 패치 예시

**문제**: 큐브가 kinematic (고정)으로 설정됨

**해결**:
```python
# ❌ Before
rigid_props = RigidBodyPropertiesCfg(
    kinematic_enabled=True,   # 큐브가 움직이지 않음!
)

# ✅ After
rigid_props = RigidBodyPropertiesCfg(
    kinematic_enabled=False,  # Dynamic
    disable_gravity=False,
    solver_position_iteration_count=8,
    solver_velocity_iteration_count=1,
)
```

---

## ✅ 5. Reward Signal

### 5.1 문제 증상
- 로봇이 큐브 정보를 받지만 무시함
- 보상이 time penalty (-0.01)만 누적
- 학습 곡선이 평평함

### 5.2 진단 단계

#### Step 1: Dense Reward 추가
```python
# env.py - compute_reward()

# ❌ Sparse만 (학습 느림)
reward = 100.0 if success else 0.0

# ✅ Dense + Sparse
ee_to_cube_dist = torch.norm(ee_pos - cube_pos, dim=-1)

# Dense reaching reward
reaching_reward = -0.1 * ee_to_cube_dist
reward += reaching_reward

# Sparse milestone reward
if ee_to_cube_dist < 0.05 and not self.reached:
    reward += 5.0
    self.reached = True
```

#### Step 2: Stage-wise Reward
```python
# 다단계 보상
reward = 0.0

# Stage 1: REACH (+5)
if ee_to_cube_dist < 0.05 and not self.reached:
    reward += 5.0
    self.reached = True

# Stage 2: GRIP (+10)
if grasp_valid and not self.gripped:
    reward += 10.0
    self.gripped = True

# Stage 3: LIFT (+15)
if cube_height > 0.05 and not self.lifted:
    reward += 15.0
    self.lifted = True

# Stage 4: SUCCESS (+100)
if cube_at_goal:
    reward += 100.0
```

### 5.3 패치 예시

**문제**: Sparse reward만 사용

**해결**:
```python
# ❌ Before
def compute_reward(self):
    success = (cube_to_goal < 0.05)
    return 100.0 if success else -0.01  # Time penalty만

# ✅ After
def compute_reward(self):
    reward = torch.zeros(self.num_envs, device=self.device)
    
    # Dense reaching (항상 신호)
    ee_to_cube = torch.norm(self.ee_pos - self.cube_pos, dim=-1)
    reward += -0.1 * ee_to_cube
    
    # Sparse milestones (1회성)
    reach_mask = (ee_to_cube < 0.05) & ~self.reached
    reward[reach_mask] += 5.0
    self.reached[reach_mask] = True
    
    # ... (GRIP, LIFT, SUCCESS)
    
    return reward
```

---

## ✅ 6. Task Integration

### 6.1 문제 증상
- 환경이 실행되지만 큐브가 spawn되지 않음
- `get_observations()`에서 에러 발생
- `AttributeError: 'RoArmEnv' object has no attribute 'cube'`

### 6.2 진단 단계

#### Step 1: 오브젝트 등록 확인
```python
# env.py

def _setup_scene(self):
    # ✅ 큐브 등록 필수!
    self.cube = RigidObject(cfg=self.cfg.cube)
    
    # Scene에 추가
    self.scene.articulations["robot"] = self.robot
    self.scene.rigid_objects["cube"] = self.cube
    
    # Clone environments
    self.scene.clone_environments(copy_from_source=False)
```

#### Step 2: 참조 작업과 비교
```python
# Isaac Lab의 pick_cube_env.py와 비교

# 필수 메서드:
# - _setup_scene()
# - _pre_physics_step()
# - _apply_action()
# - _get_observations()
# - _get_rewards()
# - _get_dones()
# - _reset_idx()
```

### 6.3 패치 예시

**문제**: 큐브가 Scene에 등록 안 됨

**해결**:
```python
# ❌ Before
def _setup_scene(self):
    self.robot = Articulation(cfg=self.cfg.robot)
    # 큐브 등록 누락!

# ✅ After
def _setup_scene(self):
    self.robot = Articulation(cfg=self.cfg.robot)
    self.cube = RigidObject(cfg=self.cfg.cube)
    self.goal = XFormPrim(cfg=self.cfg.goal_marker)
    
    # Scene에 추가
    self.scene.articulations["robot"] = self.robot
    self.scene.rigid_objects["cube"] = self.cube
    self.scene.xform_prims["goal"] = self.goal
    
    self.scene.clone_environments(copy_from_source=False)
    
    # ✅ 초기화 확인
    print(f"Scene objects: {self.scene.articulations.keys()}")
    print(f"Cube initialized: {self.cube is not None}")
```

---

## ✅ 7. Runtime Diagnostics

### 7.1 실시간 진단 코드

```python
# env.py - step() 또는 _post_physics_step()

def _post_physics_step(self):
    # 100 스텝마다 진단 출력
    if self.step_count % 100 == 0:
        print("\n=== Diagnostic Info ===")
        print(f"Step: {self.step_count}")
        
        # 1. Observation
        obs = self._get_observations()
        print(f"Cube pos: {self.cube.data.root_pos_w[0]}")
        print(f"EE pos: {self.ee_pos[0]}")
        print(f"Distance: {torch.norm(self.ee_pos[0] - self.cube.data.root_pos_w[0]):.3f}")
        
        # 2. Reward
        reward = self._get_rewards()
        print(f"Reward: {reward[0]:.2f}")
        
        # 3. Physics
        cube_vel = self.cube.data.root_lin_vel_w[0]
        print(f"Cube velocity: {torch.norm(cube_vel):.3f}")
        
        # 4. Gripper
        gripper_width = self.robot.data.joint_pos[0, 6] + self.robot.data.joint_pos[0, 7]
        print(f"Gripper width: {gripper_width:.3f}")
        
        print("======================\n")
```

### 7.2 TensorBoard 로그

```python
# train_dense_reward.py

# Custom callback for diagnostics
class DiagnosticCallback(BaseCallback):
    def _on_step(self) -> bool:
        if self.n_calls % 1000 == 0:
            # Milestone counts
            self.logger.record("diagnostics/reach_count", reach_count)
            self.logger.record("diagnostics/grip_count", grip_count)
            
            # Observation stats
            self.logger.record("diagnostics/mean_cube_height", cube_height.mean())
            self.logger.record("diagnostics/mean_ee_to_cube", ee_to_cube_dist.mean())
        
        return True

model.learn(
    total_timesteps=50000,
    callback=DiagnosticCallback()
)
```

---

## 📊 종합 진단 스크립트

### 자동 진단 실행

```bash
cd ~/roarm_isaac_clean/scripts/rl
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh diagnose_env.py
```

**diagnose_env.py** (다음 작업에서 생성):
- 1~7단계 자동 체크
- 문제 발견 시 자동 리포트 생성
- 패치 제안

---

## 🎯 RoArm M3 적용 예시

### 현재 문제: GRIP 미달성

#### 진단 결과 (Step 5: Reward Signal)
```
❌ grasp_valid 조건 너무 엄격:
   - ee_to_cube_dist < 0.08 m
   - gripper_width < 0.02 m
   - cube_pos[2] > 0.03 m

✅ 권장 패치:
   - ee_to_cube_dist < 0.10 m  (완화)
   - gripper_width < 0.03 m    (완화)
   - cube_pos[2] > 0.02 m      (완화)
```

#### 자동 패치
```python
# env.py - compute_reward()

# ❌ Before (엄격)
grasp_valid = (
    (ee_to_cube_dist < 0.08) &
    (gripper_width < 0.02) &
    (cube_pos[:, 2] > 0.03)
)

# ✅ After (완화)
grasp_valid = (
    (ee_to_cube_dist < 0.10) &
    (gripper_width < 0.03) &
    (cube_pos[:, 2] > 0.02)
)

# 또는 Contact force 추가
cube_contact_forces = self.cube.get_net_contact_forces()
contact_force_norm = torch.norm(cube_contact_forces, dim=-1)

grasp_valid = (
    (ee_to_cube_dist < 0.10) &
    (gripper_width < 0.03) &
    (contact_force_norm > 0.5)  # 접촉력 감지
)
```

---

## 📚 참고 자료

1. **Isaac Lab 공식 문서**: https://isaac-sim.github.io/IsaacLab/
2. **IsaacGymEnvs 예제**: https://github.com/NVIDIA-Omniverse/IsaacGymEnvs
3. **OpenAI Gym 디버깅 가이드**: https://github.com/openai/gym

---

**작성자**: GitHub Copilot  
**최종 업데이트**: 2025-10-20  
**다음**: diagnose_env.py 스크립트 생성
