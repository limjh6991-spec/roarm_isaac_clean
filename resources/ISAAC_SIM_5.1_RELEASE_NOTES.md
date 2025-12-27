# Isaac Sim 5.1.0 릴리즈 노트 및 변경사항

**최종 업데이트**: 2025년 12월 8일  
**버전**: 5.1.0 (October 2025)

---

## 📋 주요 변경사항

### 일반 (General)
- **Kit SDK 업데이트**: 107.3.1 → 107.3.3
- **DGX Spark 지원** 추가
- **호환성 검사기**: 더 이상 독립 실행형 앱이 아님 (Isaac Sim에 통합)
- ⚠️ **중요**: 모든 deprecated 확장은 **6.0 메이저 릴리즈에서 완전히 제거**될 예정

### PhysX 엔진
- **Articulation 개선**:
  - 로봇 그리퍼용 새로운 조인트 파라미터 튜닝 튜토리얼
  - **그리퍼 침투(penetration) 문제 해결**을 위한 articulation 충돌 접촉을 마지막에 해결하는 새 기능
  - Deformable 객체 상호작용 회귀 수정

### 센서 (Sensors)
- RTX Sensor를 사용한 **Semantic Segmentation (Object ID)** 지원 추가
- USD 속성을 통한 **Non-visual Materials** 지원
- **IMU 센서 GPU 처리** 활성화 (성능 향상)
- **RTX Sensor Annotators** 전체 성능 개선
- **Motion BVH 기본 비활성화** (성능 향상)
- LidarRtx API 버그 수정 다수

### 합성 데이터 생성 (SDG)
- **Cosmos용 합성 데이터 생성** 튜토리얼 추가
- Actor SDG의 Navigation Mesh API 업데이트
- MobilityGen 문서 개선

### Docker
- **Multi-arch 지원** 추가
- **컨테이너 기본 rootless 사용자**로 실행

### ROS 2
- **Simulation Interfaces v1.1.0** 지원
  - 새 서비스: GetAvailableWorlds, GetCurrentWorld, LoadWorld, UnloadWorld
- **rclpy executors** 사용으로 성능 최적화
- 시작 시 ROS Simulation Control 확장 활성화 설정 추가

---

## 🔄 API 변경사항 (Migration Guide)

### Deprecated → New API 매핑

| 이전 API (Deprecated) | 새 API (5.1+) |
|----------------------|---------------|
| `omni.isaac.kit.SimulationApp` | `isaacsim.SimulationApp` |
| `omni.isaac.core.World` | `isaacsim.core.api.World` |
| `omni.isaac.core.*` | `isaacsim.core.api.*` |
| `omni.isaac.sensor.Camera` | `isaacsim.sensors.camera.Camera` |
| `omni.isaac.sensor.*` | `isaacsim.sensors.camera/physics/physx/rtx.*` |
| `omni.isaac.manipulators.*` | `isaacsim.robot.manipulators.*` |
| `omni.isaac.wheeled_robots.*` | `isaacsim.robot.wheeled_robots.*` |
| `omni.isaac.motion_generation.*` | `isaacsim.robot_motion.motion_generation.*` |
| `omni.isaac.lula.*` | `isaacsim.robot_motion.lula.*` |
| `omni.isaac.range_sensor.*` | `isaacsim.sensors.physx.*` |
| `omni.isaac.cloner.*` | `isaacsim.core.cloner.*` |
| `omni.isaac.surface_gripper.*` | `isaacsim.robot.surface_gripper.*` |
| `omni.isaac.cortex.*` | `isaacsim.cortex.framework.*` |
| `omni.isaac.nucleus.*` | `isaacsim.storage.native.*` |
| `omni.isaac.version.*` | `isaacsim.core.version.*` |
| `omni.isaac.menu.*` | `isaacsim.gui.menu.*` |
| `omni.replicator.isaac.*` | `isaacsim.replicator.*` |

### isaacsim.simulation_app 수정사항
- **`close()` 메서드**: `skip_cleanup` 파라미터 추가
- **종료 시 hang 수정**: stage 강제 종료로 해결
- **뷰포트 초기화 문제** 수정
- **experience 파일 None 허용** 수정
- **DISPLAY 환경변수 없으면 headless 강제**

---

## 📁 Experience Files (앱 설정)

Isaac Sim 5.1.0은 다양한 experience 파일 제공:

| 파일명 | 용도 | 로드 시간 |
|--------|------|----------|
| `isaacsim.exp.base.python.kit` | **경량 Python 스크립팅** (권장) | ~15초 |
| `isaacsim.exp.base.kit` | 기본 앱 | ~20초 |
| `isaacsim.exp.full.kit` | 전체 GUI 앱 | ~25초 |
| `isaacsim.exp.full.streaming.kit` | WebRTC 스트리밍 (Docker 기본) | ~30초+ |

### Docker에서 경량 모드 사용

```bash
# 기본 (streaming 모드 - 느림)
docker run --rm --gpus all -e ACCEPT_EULA=Y \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  /isaac-sim/python.sh script.py

# 경량 모드 (권장)
docker run --rm --gpus all -e ACCEPT_EULA=Y \
  --entrypoint "" \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  /isaac-sim/kit/kit \
  /isaac-sim/apps/isaacsim.exp.base.python.kit \
  --no-window \
  --exec "/workspace/script.py"
```

---

## 🐛 알려진 이슈 및 수정사항

### isaacsim.simulation_app 수정된 버그
1. **종료 시 hang** → stage 강제 종료로 해결
2. **시작 시 뷰포트 대기 hang** → 수정됨
3. **종료 시 hang** → 수정됨
4. **DISPLAY 없을 때 hang** → headless 자동 강제

### 센서 관련 수정
- SingleViewDepthSensorAsset 위치/방향 설정 수정
- Camera.set_opencv_pinhole_properties imageSize 속성 수정
- LidarRtx prim 방향/위치 재설정 문제 수정

---

## 📦 주요 의존성 업데이트

| 패키지 | 이전 버전 | 새 버전 |
|--------|----------|---------|
| omni.warp | 1.7.1 | **1.8.2** |
| omni.physx | 107.3.18 | **107.3.26** |
| omni.replicator.core | 1.12.16 | **1.12.27** |
| omni.kit.asset_converter | 4.1.4 | **5.0.16** |

---

## 🚀 새로운 로봇

- **Schunk**: Egk25, Egu50, Ezu35, SVH hand (좌/우)
- **Booster T1**
- **G1 업데이트**: 좌/우 손 변형, 세 손가락/inspire hand 지원
- **UR10e 타겟 추적 예제** 추가

---

## 📚 참고 링크

- [공식 릴리즈 노트](https://docs.isaacsim.omniverse.nvidia.com/latest/overview/release_notes.html)
- [Python 스크립팅 가이드](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/index.html)
- [SimulationApp 문서](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/manual_standalone_python.html)
- [API Reference](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_python_api.html)
