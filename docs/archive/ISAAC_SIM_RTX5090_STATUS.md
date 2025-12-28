# Isaac Sim & RTX 5090 현황 분석 (2025-11-17)

## 📊 최신 Isaac Sim 버전

### 공식 릴리즈
- **Isaac Sim 5.1.0 GA** (2024년 10월 31일 릴리즈)
  - 최신 안정 버전
  - GitHub: https://github.com/isaac-sim/IsaacSim
  - 다운로드: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html

### 우리 환경
- **Isaac Sim 5.0.0-rc.45** (Release Candidate)
  - 2024년 10월 초 버전
  - Warp 1.7.1 내장 (1.10.0으로 업그레이드 완료)

## 🎮 RTX 5090 (Blackwell) 지원 현황

### GPU 스펙
- **Architecture**: Blackwell (sm_120)
- **Driver**: 580.95.05 (CUDA 13.0 support) ✅
- **CUDA Toolkit**: 12.6.85 ✅
- **PyTorch**: 2.8.0+cu128 with sm_120 support ✅

### Isaac Sim 호환성

#### ✅ 작동하는 기능
1. **SimulationApp 초기화**
   - Headless mode 정상 부팅
   - Extension 로딩 완료 (150+ extensions)

2. **World 생성**
   - `isaacsim.core.api.World` 정상 작동
   - Ground plane 추가
   - PhysX GPU acceleration (2949MB VRAM 사용)

3. **기본 Rendering**
   - Vulkan backend 작동
   - RTX rendering 활성화

4. **Warp CUDA 초기화** (1.10.0 업그레이드 후)
   - sm_120 architecture 인식
   - CUDA kernel compilation 성공

#### ❌ 크래시 발생하는 기능

1. **Camera Synthetic Data Generation**
   - **증상**: Segmentation Fault
   - **위치**: `omni.syntheticdata.plugin.so`, `omni.graph.core.plugin.so`
   - **Stack Trace**:
     ```
     libomni.syntheticdata.plugin.so!std::__throw_bad_variant_access
     libomni.graph.core.plugin.so!GraphPipelineStage
     ```
   - **발생 시점**: 
     - Camera `get_rgba()` / `get_depth()` 호출 후
     - `env.step()`에서 rendering 수행 중

2. **관련 코드 패턴** (GitHub 분석 결과)
   ```python
   # 크래시 유발 가능성 높은 패턴
   camera.get_current_frame()
   rgb = camera.get_rgba()  # <-- Empty array 반환 가능
   depth = camera.get_depth()  # <-- Empty array 반환 가능
   ```

## 🔍 GitHub Isaac Sim 코드베이스 분석

### Camera Rendering 관련 발견 사항

1. **CameraView Tiled Sensor**
   ```python
   # source/extensions/isaacsim.sensors.camera/isaacsim/sensors/camera/camera_view.py
   def get_data(annotator_type: str, *, tiled: bool = False, out: Optional[wp.array] = None)
   ```
   - Replicator의 tiled sensor 사용
   - CUDA device 간 메모리 복사 수행
   - **잠재적 이슈**: Device mismatch, memory alignment 문제

2. **Synthetic Data Pipeline**
   ```cpp
   // source/extensions/isaacsim.ros2.bridge/nodes/OgnROS2PublishImage.cpp
   cudaMemcpy2DFromArrayAsync(..., cudaMemcpyDeviceToHost, ...)
   ```
   - CUDA stream synchronization 필요
   - RTX 5090 Blackwell architecture에서 메모리 전송 동기화 이슈 가능성

3. **Camera Annotator Device**
   ```python
   # Camera 초기화 시
   Camera(annotator_device="cpu")  # CPU로 강제 설정 가능
   ```

## 🐛 알려진 RTX 5090 관련 이슈

### NVIDIA Forums 검색 결과

1. **"Isaac sim failed to run any example with GPU crash"**
   - 포럼 게시일: 2024-11-15
   - 증상: GPU crash with segmentation fault
   - **해결 방법**: 아직 공식 답변 없음

2. **"Strange phenomena occur with SDF collisions"**
   - PhysX SDF collider와 RTX rendering 충돌 가능성

3. **Warp CUDA Issues**
   - `cuDeviceGetUuid` 에러 (우리는 Warp 1.10.0 업그레이드로 해결)

## 💡 해결 방안

### 1. Isaac Sim 5.1.0 GA 업그레이드 (권장 ★★★★★)

**이유**:
- 5.0.0 RC는 RTX 5090 출시 전 버전
- 5.1.0 GA에 Blackwell 최적화 포함 가능성 높음
- Warp 1.10.0+ 기본 탑재

**설치 방법**:
```bash
# Omniverse Launcher를 통한 설치 (가장 안전)
# 또는 GitHub Release 다운로드
wget https://github.com/isaac-sim/IsaacSim/releases/latest
```

### 2. Camera Annotator Device 변경 (임시 우회 ★★★)

```python
# CPU rendering으로 우회 (느리지만 안정적)
camera = Camera(
    prim_path="/World/Camera",
    annotator_device="cpu",  # <-- CUDA 대신 CPU 사용
    resolution=(640, 480)
)
```

### 3. Replicator 기반 데이터 수집 (★★★★)

```python
# Isaac Sim 공식 예제 패턴
import omni.replicator.core as rep

render_product = rep.create.render_product(camera_path, (640, 480))
rgb_annotator = rep.annotators.get("rgb")
rgb_annotator.attach(render_product)

# Step rendering
rep.orchestrator.step()
rgb_data = rgb_annotator.get_data()
```

### 4. Isaac Sim 4.2 롤백 (안정성 우선 ★★★)

- Isaac Sim 4.2는 RTX 4090까지 검증됨
- RTX 5090에서도 작동할 가능성 높음
- Vision RL 환경은 API 변경 필요

## 📝 다음 단계 권장사항

### 우선순위 1: Isaac Sim 5.1.0 GA 테스트
```bash
# 1. Backup 현재 환경
mv ~/isaacsim ~/isaacsim_5.0_rc_backup_20251117

# 2. Isaac Sim 5.1.0 다운로드 및 설치
# Omniverse Launcher 사용 또는
# https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html

# 3. Warp 버전 확인
~/isaacsim/python.sh -c "import warp; print(warp.__version__)"
```

### 우선순위 2: CPU Annotator 임시 테스트
```python
# simple_vision_env_v2.py 수정
self.camera = Camera(
    prim_path="/World/RoArm/.../D405",
    annotator_device="cpu",  # <-- 추가
    frequency=30,
    resolution=(640, 480)
)
```

### 우선순위 3: Community 버그 리포트
- NVIDIA Isaac Sim Forum에 RTX 5090 crash 리포트
- 제공할 정보:
  - GPU: RTX 5090
  - Driver: 580.95.05
  - Isaac Sim: 5.0.0-rc.45
  - Crash location: Camera synthetic data rendering
  - Stack trace 첨부

## 🔗 참고 링크

- **Isaac Sim Documentation**: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/
- **NVIDIA Forum**: https://forums.developer.nvidia.com/c/omniverse/simulation/69
- **GitHub Issues**: https://github.com/isaac-sim/IsaacSim/issues
- **RTX 5090 Specs**: https://www.nvidia.com/en-us/geforce/graphics-cards/50-series/

---

**결론**: Isaac Sim 5.0.0 RC는 RTX 5090 출시 전 버전이므로 Blackwell 최적화 미흡. **Isaac Sim 5.1.0 GA 업그레이드 강력 권장**.
