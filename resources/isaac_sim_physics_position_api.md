# Isaac Sim 5.1: Real-time Physics Position API 연구

## 문제 상황

EE(End Effector) 위치가 물리 시뮬레이션 후에도 업데이트되지 않음:
```python
# 이 코드는 작동하지 않음!
xformable = UsdGeom.Xformable(gripper_prim)
world_transform = xformable.ComputeLocalToWorldTransform(0)  # 항상 초기값 반환
```

**증상**:
- Joint position은 변경됨 (0.01 → 0.05)
- EE distance는 변경 없음 (0.551m 고정)

---

## 원인 분석

### USD vs Physics Engine

| 방법 | 데이터 소스 | 실시간 | 용도 |
|------|------------|--------|------|
| `UsdGeom.ComputeLocalToWorldTransform()` | USD Stage | ❌ | 정적 오브젝트, 초기 위치 |
| `RigidPrim.get_world_pose()` | PhysX Engine (Fabric) | ✅ | 동적 물리 오브젝트 |
| `RigidPrimView.get_world_poses()` | PhysX Tensor API | ✅ | 다중 오브젝트 배치 처리 |
| `ArticulationView.get_link_poses()` | PhysX Engine | ✅ | 로봇 링크 위치 |

### 핵심 문제

Isaac Sim에서 **Fabric**이 활성화되면:
1. 물리 엔진 출력이 USD Stage에 즉시 반영되지 않음
2. `ComputeLocalToWorldTransform()`은 USD Stage만 읽음
3. 따라서 **항상 초기 위치만 반환**

---

## 해결 방법

### 방법 1: RigidPrim 사용 (권장)

```python
from isaacsim.core.prims import RigidPrim

# 초기화 시
gripper_prim = RigidPrim(prim_path="/World/RoArm/gripper_link", name="gripper")

# 매 스텝에서
position, orientation = gripper_prim.get_world_pose()
```

### 방법 2: RigidPrimView 사용 (배치 처리)

```python
from isaacsim.core.prims import RigidPrimView

# 여러 오브젝트를 한 번에
view = RigidPrimView(prim_paths_expr="/World/objects/object_*")
positions, orientations = view.get_world_poses()
```

### 방법 3: ArticulationView 사용 (로봇 관절)

```python
from omni.isaac.core.articulations import ArticulationView

robot_view = ArticulationView(prim_paths_expr="/World/RoArm")
link_poses = robot_view.get_link_poses(link_names=["gripper_link"])
```

### 방법 4: SingleArticulation 내장 메서드

```python
# SingleArticulation이 이미 있다면
robot = SingleArticulation(prim_path="/World/RoArm", name="roarm")

# 링크 위치 가져오기
ee_pose = robot.get_link_pose("gripper_link")  # 또는 end_effector_link
```

---

## Isaac Lab 예제 (참고)

```python
from isaaclab.assets import RigidObject

# RigidObject 초기화
cone_object = RigidObject(prim_path="/World/Cone")

# 시뮬레이션 루프
sim.step()
cone_object.update(sim_dt)  # 내부 버퍼 업데이트 필수!

# 위치 읽기
position = cone_object.data.root_pos_w
```

---

## 주의사항

1. **update() 호출 필수**: 시뮬레이션 스텝 후 `update()` 호출해야 내부 버퍼가 갱신됨
2. **호출 타이밍**: `world.step()` 이후에 위치를 읽어야 함
3. **Headless 모드**: Fabric이 기본 활성화되어 USD 동기화 안 됨
4. **성능**: `RigidPrimView`가 개별 `RigidPrim`보다 빠름

---

## 우리 코드 수정 방향

### 현재 코드 (작동 안 함)
```python
def _get_end_effector_pos(self):
    xformable = UsdGeom.Xformable(gripper_prim)
    world_transform = xformable.ComputeLocalToWorldTransform(0)  # ❌
```

### 수정 방향
```python
def _get_end_effector_pos(self):
    # 옵션 1: RigidPrim 사용
    from isaacsim.core.prims import RigidPrim
    gripper = RigidPrim(prim_path="/World/RoArm/gripper_link")
    position, _ = gripper.get_world_pose()
    return np.array(position)
    
    # 옵션 2: ArticulationView 사용
    # link_pose = self.robot_view.get_link_poses(link_names=["gripper_link"])
```

---

## 참고 자료

- Isaac Sim Core API - RigidPrim, RigidPrimView
- Isaac Sim Vectorized APIs 문서
- Isaac Lab Assets Tutorial
- GitHub Issues: isaac-sim/IsaacLab#1234 (Fabric와 USD 동기화 문제)

---

*작성: 2025-12-28 11:45*
*버전: Isaac Sim 5.1*
