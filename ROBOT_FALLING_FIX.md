# 로봇이 떨어지는 문제 해결 가이드

**문제**: 로봇 베이스는 고정되어 있지만, 팔이 중력에 의해 아래로 쳐짐

**원인**: Joint에 drive (actuator)가 설정되지 않아서 중력을 버티지 못함

**해결**: Joint drive 설정 추가

---

## 수정 내용

### 1. Joint Drive 설정 추가 (`envs/roarm_pick_place_env.py`)

```python
# ✅ Joint drive 설정 (USD API 사용)
from pxr import UsdPhysics, PhysxSchema

for i, joint_name in enumerate(self.joint_names):
    joint_prim = stage.GetPrimAtPath(f"{prim_path}/{joint_name}")
    drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    
    if i < 6:  # 팔 joint
        drive_api.GetStiffnessAttr().Set(10000.0)  # 강한 힘
        drive_api.GetDampingAttr().Set(1000.0)
        drive_api.GetMaxForceAttr().Set(1000.0)
    else:  # 그리퍼
        drive_api.GetStiffnessAttr().Set(1000.0)
        drive_api.GetDampingAttr().Set(100.0)
        drive_api.GetMaxForceAttr().Set(100.0)
```

### 2. 초기 자세 변경

```python
# 중력에 의해 떨어지지 않도록 안정적인 자세
home_positions = np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
```

---

## 테스트

```bash
# GUI로 확인
PYTHONUNBUFFERED=1 ~/isaacsim/python.sh scripts/test_trained_model.py --episodes 1
```

**확인 사항**:
- ✅ 로봇이 공중에 떠 있지 않고 바닥에 고정
- ✅ 팔이 중력에 의해 떨어지지 않음
- ✅ 팔이 부드럽게 움직임

---

## 관련 파일

- `envs/roarm_pick_place_env.py` - Joint drive 설정
- `scripts/test_trained_model.py` - GUI 테스트 스크립트
