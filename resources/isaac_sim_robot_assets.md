# Isaac Sim 표준 로봇 에셋 접근 가이드

## 문제
Isaac Sim 표준 모바일 로봇(Jetbot, Carter, Nova Carter)은 Nucleus Server에서 호스팅됩니다.
로컬에서 직접 다운로드할 수 없고, Nucleus 연결이 필요합니다.

## 현재 사용 중인 모바일 베이스

### Clearpath Husky A200 (야외/농업용 권장)
| 항목 | 값 |
|------|-----|
| 크기 | 99 x 67 x 39 cm |
| 무게 | 50 kg |
| 최대 적재 | 75 kg |
| 야외 적재 | 20 kg |
| 구동 | 4륜 스키드 스티어 |
| DOF | 4 (휠) |
| 로컬 경로 | `/assets/husky/usd/husky_mesh.usd` |
| 메시 | ✅ 포함 (DAE) |

## Isaac Sim 표준 로봇 에셋 경로

| 로봇 | Nucleus 경로 |
|------|-------------|
| Jetbot | `/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd` |
| Carter | `/Isaac/Robots/NVIDIA/Carter/carter_v1.usd` |
| Nova Carter | `/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd` |
| Nova Carter (ROS2) | `/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd` |

## 해결 방법

### 방법 1: Isaac Sim GUI에서 Nucleus 연결 (권장)

1. Isaac Sim GUI 실행
2. Window > Browsers > Content Browser
3. Omniverse 아이콘 클릭 → 로그인
4. `/Isaac/Robots/` 경로에서 로봇 에셋 확인
5. 드래그 앤 드롭으로 씬에 추가하면 로컬에 캐시됨

### 방법 2: omni.isaac.nucleus API 사용

```python
from omni.isaac.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
# 반환값: https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1

jetbot_path = f"{assets_root}/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
```

### 방법 3: 수동 캐시 확인

Nucleus 에셋은 다음 위치에 캐시될 수 있음:
- `~/.local/share/ov/data/`
- `~/.cache/ov/`

### 방법 4: NVIDIA NGC에서 Isaac Sim Assets 다운로드

일부 에셋은 NGC에서 다운로드 가능:
```bash
# NGC CLI 필요
ngc registry resource download-version "nvidia/isaac/assets:latest"
```

## 현재 상태

현재 시스템:
- Nucleus S3 URL확인됨: `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1`
- 직접 HTTP 다운로드 불가 (인증 필요)
- Isaac Sim GUI에서 Nucleus 로그인 필요

## 대안: 간단한 URDF 사용

Nucleus 연결 없이도 기능 테스트가 가능하도록 간단한 차동구동 로봇 URDF 제작:
- `/assets/jetbot/simple_jetbot.urdf` - 기본 형태의 2휠 차동구동 로봇
- 실제 운영 시 표준 에셋으로 교체 권장

---
*작성: 2025-12-28*
