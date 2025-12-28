# Isaac Sim URDF Import: defaultPrim 문제 해결 가이드

## 문제 현상

URDF를 USD로 변환 후 로드할 때 다음 오류 발생:
```
Warning: Unresolved reference prim path 
@/path/to/file.usd@<defaultPrim> introduced by...
```

## 원인

USD 파일에는 **defaultPrim**이 설정되어야 합니다:
- `defaultPrim`은 USD 파일의 루트(시작점)를 지정
- URDF 변환 시 기본적으로 설정되지 않음
- 다른 USD에서 reference로 가져올 때 이 오류 발생

## 해결 방법

### 방법 1: Python 스크립트에서 make_default_prim 설정

```python
import omni.kit.commands

result = omni.kit.commands.execute(
    'URDFParseAndImportFile',
    urdf_path=urdf_path,
    dest_path=usd_path,
    import_config={
        'make_default_prim': True,  # ★ 핵심 설정
        'merge_fixed_joints': False,
        'fix_base': False,
    }
)
```

### 방법 2: Isaac Sim GUI에서 설정

1. File > Import > URDF
2. Import Options에서 **"Set as the default prim"** 체크
3. Import 버튼 클릭

### 방법 3: USD 생성 후 defaultPrim 수동 설정

```python
from pxr import Usd

stage = Usd.Stage.Open(usd_path)
# 루트 prim 찾기
root_prim = stage.GetPrimAtPath("/JetBot")  # 또는 실제 루트 경로
stage.SetDefaultPrim(root_prim)
stage.GetRootLayer().Save()
```

### 방법 4: Reference로 로드할 때 prim 경로 직접 지정

```python
from isaacsim.core.utils.stage import add_reference_to_stage

# defaultPrim 대신 실제 prim 경로 지정
add_reference_to_stage(
    usd_path=usd_path + "#/JetBot",  # USD 내부 prim 경로 명시
    prim_path="/World/Jetbot"
)
```

## Isaac Sim URDF Import 모범 사례

### 1. 특수문자 회피
- USD prim 이름에 `-` (하이픈) 사용 불가
- URDF에서 `_` (언더스코어)로 대체 권장

### 2. 권장 Import 설정
```python
import_config = {
    'make_default_prim': True,       # defaultPrim 설정
    'merge_fixed_joints': False,      # 고정 조인트 유지
    'fix_base': False,                # 모바일 로봇은 False
    'make_instanceable': False,       # 단일 인스턴스
    'self_collision': False,          # 자체 충돌 비활성
    'default_drive_type': 1,          # Position drive (0=None, 1=Position, 2=Velocity)
}
```

### 3. Articulation 사용 시 주의
```python
# world.reset() 호출 후 articulation 생성
world.reset()
robot = SingleArticulation(prim_path="/World/Robot", name="robot")
world.scene.add(robot)
world.reset()  # 다시 reset 필요
```

## 참고 자료

- [NVIDIA Isaac Sim URDF Importer](https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/ext_omni_isaac_urdf.html)
- [USD DefaultPrim Documentation](https://openusd.org/dev/api/class_usd_stage.html#a5e3eb4ba0b01ec6c55f5b7d56e4c6e8e)

---
*작성: 2025-12-28*
*Isaac Sim 5.1 기준*
