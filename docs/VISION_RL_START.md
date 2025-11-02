# Vision RL 프로젝트 시작 요약

**날짜**: 2025-11-02  
**상태**: 계획 수립 완료 ✅

---

## 📋 오늘 작업 완료

### 1. Phase 1 완료 확인
✅ RoArm-M3 + D405 카메라 통합  
✅ IsaacLab Camera 센서 구현  
✅ RGB + Depth 이미지 캡처 (16개 이미지)  
✅ 8개 동작 시퀀스 테스트 성공  

**출력 위치**: `output/camera_images/20251102_155439/`

### 2. Vision RL 계획 수립
✅ `docs/VISION_RL_PLAN.md` 작성 (완료)

**주요 내용**:
- Architecture 설계 (Camera → CNN → PPO → Action)
- 4단계 구현 계획 (Env → CNN → Training → Eval)
- 기술 스택 정의
- Success Criteria 설정

### 3. 테스트 스크립트 작성
✅ `scripts/test/test_vision_env.py` 작성 (완료)  
✅ `scripts/test/test_vision_quick.py` 작성 (완료)

**기능**:
- Random policy로 환경 테스트
- RGB-D 이미지 저장
- Observation/Action space 검증
- Episode 통계 출력

### 4. Vision 전처리 테스트 ✅ **성공!**
✅ `test_vision_quick.py` 실행 성공  
✅ RGB-D 전처리 검증 (84×84 resize + normalize)  
✅ 10개 이미지 생성 (5 RGB + 5 Depth)

**출력 위치**: `output/vision_test_quick/20251102_160809/`

**검증 완료**:
- RGB: (480, 640, 3) → (84, 84, 3) → (3, 84, 84) [0, 1] ✅
- Depth: (480, 640) → (84, 84) → (1, 84, 84) [0, 1] ✅  
- RGBD: (4, 84, 84) [0, 1] ✅

---

## 🎯 다음 단계 (우선순위)

### Immediate (오늘/내일)

#### 1. Vision Environment 완성 🔥
**파일**: `envs/roarm_pick_place_env_vision.py`

**필요 작업**:
```python
# TODO 1: RGB-D Preprocessing 완성
def _preprocess_rgbd(self, rgb, depth):
    """Resize to 84x84 and normalize"""
    # resize, normalize, stack
    pass

# TODO 2: Reward Function 완성
def _compute_reward(self):
    """Dense reward + vision bonus"""
    # distance-based + in-view bonus
    pass

# TODO 3: Reset Logic 완성
def reset(self):
    """Randomize object/goal positions"""
    # random spawn, camera reset
    pass

# TODO 4: Helper Functions
def _is_object_in_view(self):
    """Check if object is in camera FOV"""
    pass

def _check_grasp(self):
    """Detect contact with object"""
    pass
```

**테스트 방법**:
```bash
cd /home/roarm_m3/roarm_isaac_clean
/home/roarm_m3/isaacsim/python.sh scripts/test/test_vision_env.py
```

**예상 출력**:
- `output/vision_env_test/*.png` (RGB-D 이미지)
- Episode reward: -500 ~ 0 (random policy)
- 3 episodes 완료

---

#### 2. Environment Config 작성
**파일**: `configs/vision_env_cfg.py`

```python
@configclass
class VisionEnvCfg(ManagerBasedRLEnvCfg):
    # Scene
    scene = VisionSceneCfg(num_envs=1, env_spacing=2.0)
    
    # Camera
    camera = CameraCfg(
        prim_path="/World/Robot/gripper_link/camera_link/Camera",
        height=480, width=640,
        data_types=["rgb", "distance_to_image_plane"],
    )
    
    # Robot
    robot = ArticulationCfg(...)
    
    # Object
    object = RigidObjectCfg(...)
    
    # Episode
    episode_length_s = 16.7  # 1000 steps @ 60Hz
```

---

#### 3. 첫 테스트 실행
```bash
# Test 1: Environment check
/home/roarm_m3/isaacsim/python.sh scripts/test/test_vision_env.py

# Expected output:
# ✅ Observation: (4, 84, 84)
# ✅ Action: (7,)
# ✅ 3 episodes completed
# ✅ Images saved to output/vision_env_test/

# Test 2: gym.check_env (나중에)
# from stable_baselines3.common.env_checker import check_env
# check_env(env)
```

---

## 📂 프로젝트 구조

```
roarm_isaac_clean/
├── assets/roarm_m3/
│   ├── urdf/roarm_m3_with_camera_correct.urdf  ✅
│   └── meshes/d405/d405.stl  ✅
├── configs/
│   └── vision_env_cfg.py  ⏳ TODO
├── envs/
│   └── roarm_pick_place_env_vision.py  🔄 In Progress
├── models/
│   └── cnn_extractor.py  ⏳ TODO
├── scripts/
│   ├── test/
│   │   ├── test_roarm_with_camera_isaaclab.py  ✅
│   │   └── test_vision_env.py  ✅
│   ├── train/
│   │   └── train_vision_ppo.py  ⏳ TODO
│   └── eval/
│       └── eval_vision_ppo.py  ⏳ TODO
├── docs/
│   ├── VISION_RL_PLAN.md  ✅
│   └── CAMERA_TEST_SUMMARY.md  ✅
└── output/
    ├── camera_images/20251102_155439/  ✅ (16 images)
    └── vision_env_test/  ⏳ Will be created
```

---

## 🔑 핵심 기술 요소

### 1. Observation (4, 84, 84)
```python
# RGB-D stacking
rgb = camera.data.rgb[:, :, :3]  # (480, 640, 3)
depth = camera.data.depth  # (480, 640)

# Resize
rgb_84 = cv2.resize(rgb, (84, 84))
depth_84 = cv2.resize(depth, (84, 84))

# Normalize
rgb_norm = rgb_84 / 255.0  # [0, 1]
depth_norm = (depth_84 - 0.1) / 1.9  # [0, 1] (0.1m~2m)

# Stack
obs = np.concatenate([rgb_norm, depth_norm[:, :, None]], axis=-1)
obs = obs.transpose(2, 0, 1)  # (4, 84, 84)
```

### 2. Reward Function
```python
# Distance-based (dense)
dist_reward = -10.0 * distance_to_object

# Vision bonus
if object_in_view:
    vision_bonus = +0.1

# Task milestones
grasp_bonus = +20.0
lift_bonus = +30.0
place_bonus = +100.0

total_reward = dist_reward + vision_bonus + milestones
```

### 3. CNN Architecture
```
Input: (4, 84, 84)
  ↓
Conv2d(4→32, 8×8, stride=4)  # (32, 20, 20)
  ↓
Conv2d(32→64, 4×4, stride=2)  # (64, 9, 9)
  ↓
Conv2d(64→64, 3×3, stride=1)  # (64, 7, 7)
  ↓
Flatten()  # 3136
  ↓
Linear(3136→512)  # Feature vector
```

---

## ⚠️ 주의사항

### 1. IsaacSim 실행
```bash
# ❌ Wrong
./isaaclab.sh --python script.py

# ✅ Correct
/home/roarm_m3/isaacsim/python.sh script.py
```

### 2. Camera 센서 초기화
```python
# Camera를 생성한 후 반드시 reset 호출
camera = Camera(cfg=camera_cfg)
sim.reset()
camera.reset()  # ← 필수!
```

### 3. --enable_cameras Flag
```bash
# RuntimeError 방지
/home/roarm_m3/isaacsim/python.sh script.py --enable_cameras
```

---

## 📊 예상 타임라인

| 단계 | 작업 | 예상 시간 | 상태 |
|------|------|-----------|------|
| 1 | Vision Env 완성 | 2-3일 | 🔄 In Progress |
| 2 | CNN Extractor | 1-2일 | ⏳ Waiting |
| 3 | PPO Training | 3-5일 | ⏳ Waiting |
| 4 | Evaluation | 2일 | ⏳ Waiting |
| **Total** | **Full Pipeline** | **8-12일** | - |

---

## ✅ Success Metrics

### Environment (Step 1)
- [ ] `gym.check_env()` 통과
- [ ] Random policy 3 episodes 완료
- [ ] RGB-D 이미지 정상 출력 (84×84)
- [ ] Reward 범위: -500 ~ 0

### Training (Step 3)
- [ ] PPO 학습 시작
- [ ] Episode reward 증가 확인
- [ ] TensorBoard 정상 동작

### Final (Step 4)
- [ ] Success rate >70%
- [ ] Video 녹화 성공
- [ ] 문서화 완료

---

## 🚀 시작 명령어

```bash
# 1. 환경 테스트
cd /home/roarm_m3/roarm_isaac_clean
/home/roarm_m3/isaacsim/python.sh scripts/test/test_vision_env.py

# 2. (나중에) 학습 시작
/home/roarm_m3/isaacsim/python.sh scripts/train/train_vision_ppo.py

# 3. (나중에) 평가
/home/roarm_m3/isaacsim/python.sh scripts/eval/eval_vision_ppo.py
```

---

**Last Updated**: 2025-11-02 15:57  
**Next Action**: Vision Env 완성 (`roarm_pick_place_env_vision.py`)  
**Contact**: Ready to start! 🚀
