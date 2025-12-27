# Isaac Sim 5.1.0 Docker 최적화 실행 가이드

**최종 업데이트**: 2025년 12월 8일

---

## 🚨 문제점: 기본 Docker 실행이 느린 이유

Isaac Sim 5.1.0 Docker 이미지의 기본 entrypoint는 `/isaac-sim/runheadless.sh`로, 
**`isaacsim.exp.full.streaming.kit`** (WebRTC 스트리밍 앱)을 로드합니다.

이로 인해:
- **150+ 확장 모두 로드** (대부분 불필요)
- **WebRTC 서버 시작** (headless Python에서 불필요)
- **~30초 로드 시간** (경량 모드의 2배)
- **GPU 메모리 과다 사용**

---

## ✅ 해결책: 경량 Experience 사용

### 방법 1: Kit 직접 실행 (가장 빠름)

```bash
docker run --rm --gpus all \
  -e ACCEPT_EULA=Y \
  --entrypoint "" \
  -v $(pwd):/workspace \
  -w /workspace \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  /isaac-sim/kit/kit \
  /isaac-sim/apps/isaacsim.exp.base.python.kit \
  --no-window \
  --exec "script.py"
```

**장점**:
- 로드 시간: ~15초 (50% 단축)
- 최소 확장만 로드
- WebRTC 서버 없음

### 방법 2: python.sh 사용 (호환성 좋음)

```bash
docker run --rm --gpus all \
  -e ACCEPT_EULA=Y \
  --entrypoint "" \
  -v $(pwd):/workspace \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  /isaac-sim/python.sh /workspace/script.py
```

**주의**: 이 방법은 여전히 streaming 앱을 사용하지만, 호환성이 좋습니다.

### 방법 3: 커스텀 Experience 파일

```bash
# 1. 경량 experience 파일 생성
cat > my_headless.kit << 'EOF'
[package]
title = "Isaac Sim Headless Python"
version = "1.0.0"

[dependencies]
"isaacsim.core.api" = {}
"isaacsim.sensors.camera" = {}
"omni.physx" = {}

[settings]
app.window.enabled = false
EOF

# 2. 사용
docker run --rm --gpus all \
  -e ACCEPT_EULA=Y \
  --entrypoint "" \
  -v $(pwd):/workspace \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  /isaac-sim/kit/kit \
  /workspace/my_headless.kit \
  --no-window \
  --exec "/workspace/script.py"
```

---

## 📝 실행 스크립트 템플릿

### run_isaac_optimized.sh

```bash
#!/bin/bash
# Isaac Sim 5.1.0 최적화 실행 스크립트

SCRIPT=${1:-"script.py"}
TIMEOUT_SEC=${2:-120}
EXPERIENCE=${3:-"isaacsim.exp.base.python.kit"}

echo "=========================================="
echo "Isaac Sim 5.1.0 최적화 실행"
echo "스크립트: $SCRIPT"
echo "Experience: $EXPERIENCE"
echo "타임아웃: ${TIMEOUT_SEC}초"
echo "=========================================="

CONTAINER_NAME="isaac_$$"

docker run --rm --gpus all \
  --name "$CONTAINER_NAME" \
  --entrypoint "" \
  -e ACCEPT_EULA=Y \
  -v $(pwd):/workspace \
  -w /workspace \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  /isaac-sim/kit/kit \
  /isaac-sim/apps/$EXPERIENCE \
  --no-window \
  --exec "$SCRIPT" &

DOCKER_PID=$!

# 타임아웃 처리
(sleep $TIMEOUT_SEC && docker kill "$CONTAINER_NAME" 2>/dev/null) &
TIMEOUT_PID=$!

wait $DOCKER_PID
RESULT=$?

kill $TIMEOUT_PID 2>/dev/null

echo ""
echo "=== 완료 (종료 코드: $RESULT) ==="
nvidia-smi --query-gpu=memory.used,memory.free --format=csv,noheader

exit $RESULT
```

---

## 🔧 Experience 파일 비교

| Experience | 로드 시간 | 확장 수 | 용도 |
|------------|----------|---------|------|
| `isaacsim.exp.base.python.kit` | ~15초 | ~80 | **Python 스크립팅 (권장)** |
| `isaacsim.exp.base.kit` | ~20초 | ~100 | 기본 시뮬레이션 |
| `isaacsim.exp.full.kit` | ~25초 | ~140 | 전체 GUI |
| `isaacsim.exp.full.streaming.kit` | ~30초 | ~150+ | WebRTC 스트리밍 |

---

## 💡 추가 최적화 팁

### 1. GPU 메모리 제한

```python
CONFIG = {
    "headless": True,
    "max_gpu_memory_fraction": 0.5,  # 50%
    "physics_gpu": 0,
    "active_gpu": 0,
}
```

### 2. 안티앨리어싱 비활성화

```python
CONFIG = {
    "headless": True,
    "anti_aliasing": 0,  # Off
}
```

### 3. 가벼운 렌더러 사용

```python
CONFIG = {
    "headless": True,
    "renderer": "RayTracedLighting",  # PathTracing보다 빠름
}
```

### 4. 렌더링 건너뛰기 (Physics만 필요할 때)

```python
for i in range(1000):
    world.step(render=False)  # 렌더링 안 함
```

---

## 🐳 Docker Compose 예시

```yaml
# docker-compose.isaac-optimized.yml
version: "3.8"

services:
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:5.1.0
    entrypoint: ""
    command: >
      /isaac-sim/kit/kit
      /isaac-sim/apps/isaacsim.exp.base.python.kit
      --no-window
      --exec "/workspace/script.py"
    environment:
      - ACCEPT_EULA=Y
      - DISPLAY=${DISPLAY}
    volumes:
      - .:/workspace
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    working_dir: /workspace
    stdin_open: true
    tty: true
```

사용:
```bash
docker-compose -f docker-compose.isaac-optimized.yml up
```

---

## 📊 성능 비교 (RTX 5090 기준)

| 모드 | 로드 시간 | GPU 메모리 | 초당 스텝 |
|------|----------|-----------|----------|
| Streaming (기본) | ~30초 | ~4GB | ~200 |
| Python (최적화) | ~15초 | ~2GB | ~300 |
| Zero Delay | ~15초 | ~2GB | ~350 |

---

## ⚠️ 알려진 이슈

1. **`--exec` 모드에서 `SimulationApp` import 오류**
   - Kit이 이미 실행 중이므로 `from isaacsim import SimulationApp` 불필요
   - 직접 `from isaacsim.core.api import World` 사용

2. **타임아웃 필요**
   - Docker 내 Isaac Sim은 graceful shutdown이 느림
   - 항상 타임아웃 설정 권장

3. **GPU 메모리 누수**
   - 컨테이너 종료 후에도 GPU 메모리가 해제되지 않을 수 있음
   - `docker kill` 사용 시 확실히 해제됨
