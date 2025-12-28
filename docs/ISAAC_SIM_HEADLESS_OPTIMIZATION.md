# Isaac Sim Headless 모드 최적화 가이드

**작성일**: 2025-11-03  
**문제**: Headless 모드에서 Isaac Sim 초기화가 매우 느리고 CUDA 경고가 폭발적으로 발생

---

## 🚧 발견된 문제점

### 1. **Isaac Sim 초기화가 매우 느림** (5-10분)
- **증상**: `AppLauncher` 실행 후 환경 생성까지 5-10분 소요
- **원인**: 
  - Headless 모드에서도 렌더러 초기화 수행
  - USD 스테이지 로딩 및 Physics 엔진 초기화
  - 첫 실행 시 shader 컴파일
  - Camera 센서 초기화 (특히 RTX renderer)

### 2. **Python stdout 버퍼링**
- **증상**: `print()` 출력이 실시간으로 보이지 않음
- **원인**: Python 기본 stdout 버퍼링 (line buffering)
- **영향**: 진행 상황 추적 불가능

### 3. **CUDA 경고 폭발**
```
[Warning] [omni.graph.core.plugin] OmniGraphSettings::getCudaDeviceOrdinal: 
unable to get a valid CUDA device id from the renderer. Defaulting to GPU0.
```
- **원인**: Headless 모드에서 렌더러가 없어 OmniGraph가 CUDA 디바이스를 찾지 못함
- **빈도**: 초당 수백 개 (로그 파일을 금방 채움)
- **영향**: 로그에서 실제 중요한 메시지 찾기 어려움

---

## ✅ 해결책

### 1. Python stdout 버퍼링 제거

#### 방법 A: 환경 변수 사용 (권장)
```bash
PYTHONUNBUFFERED=1 /isaac-sim/python.sh your_script.py
```

#### 방법 B: Python 스크립트 수정
```python
import sys
sys.stdout = sys.__stdout__  # unbuffered stdout
sys.stderr = sys.__stderr__  # unbuffered stderr

# 또는 모든 print에 flush 추가
print("메시지", flush=True)
```

#### 방법 C: Python 실행 옵션
```bash
/isaac-sim/python.sh -u your_script.py  # -u = unbuffered
```

### 2. CUDA 경고 필터링

#### 터미널에서 grep으로 필터링
```bash
/isaac-sim/python.sh script.py 2>&1 | grep -v "getCudaDeviceOrdinal"
```

#### Python에서 경고 억제
```python
import warnings
import logging

# OmniGraph 경고 억제
logging.getLogger("omni.graph.core.plugin").setLevel(logging.ERROR)

# 또는 모든 경고 억제 (권장하지 않음)
warnings.filterwarnings("ignore")
```

#### Isaac Sim 설정에서 경고 레벨 조정
```python
from omni.isaac.lab.app import AppLauncher

# AppLauncher 생성 전
import carb
settings = carb.settings.get_settings()
settings.set("/log/level", "error")  # "warning" -> "error"
settings.set("/log/fileLogLevel", "error")
```

### 3. Isaac Sim 초기화 속도 향상

#### A. 렌더러 비활성화 (Headless 모드)
```python
from omni.isaac.lab.app import AppLauncher

# Renderer 완전 비활성화
launcher_args = AppLauncher.add_app_launcher_args(parser)
args.headless = True
args.enable_cameras = True  # 카메라는 활성화 (Vision RL 필요)

# 렌더링 품질 최소화
import carb.settings
settings = carb.settings.get_settings()
settings.set("/rtx/reflections/enabled", False)
settings.set("/rtx/shadows/enabled", False)
settings.set("/rtx/ambientOcclusion/enabled", False)
settings.set("/rtx/raytracing/fractionalCutoutOpacity", False)
```

#### B. Physics 업데이트 빈도 조정
```python
from omni.isaac.lab.sim import SimulationCfg

sim_cfg = SimulationCfg(
    dt=0.01,  # 더 큰 timestep (0.005 -> 0.01)
    substeps=2,  # 더 적은 substeps (4 -> 2)
    gravity=(0.0, 0.0, -9.81),
)
```

#### C. USD 스테이지 최적화
```python
# 불필요한 에셋 제거
# 저해상도 메시 사용
# 단순한 충돌 geometry 사용 (convex hull 대신 box)
```

#### D. Camera 해상도 감소 (Vision RL에서)
```python
camera_cfg = CameraCfg(
    height=84,  # 224 -> 84
    width=84,
    update_period=0.1,  # 10Hz (더 낮은 빈도)
)
```

### 4. 빠른 테스트 스크립트 패턴

```python
#!/usr/bin/env python3
"""
초고속 환경 검증 스크립트
"""
import sys
import argparse

# 1. stdout 버퍼링 제거
sys.stdout = sys.__stdout__
sys.stderr = sys.__stderr__

# 2. AppLauncher (경고 레벨 설정 전)
from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True

print("🚀 초기화 시작...", flush=True)

# 3. 경고 레벨 조정
import carb
settings = carb.settings.get_settings()
settings.set("/log/level", "error")

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

print("✅ Isaac Sim 로드 완료!", flush=True)

# 4. 환경 생성 (최소한으로)
# ... 환경 코드 ...

print("🎉 테스트 완료!", flush=True)
simulation_app.close()
```

---

## 📊 성능 벤치마크

### 기본 설정 (최적화 전)
- 초기화 시간: **5-10분**
- 메모리 사용: 8-10GB
- 로그 파일: 3.9MB (3분, 대부분 CUDA 경고)

### 최적화 후 (예상)
- 초기화 시간: **1-3분**
- 메모리 사용: 3-5GB
- 로그 파일: <1MB (실제 출력만)

---

## 🔍 디버깅 팁

### 1. 초기화 단계별 시간 측정
```python
import time

start = time.time()
print(f"[{time.time()-start:.1f}s] AppLauncher 생성...", flush=True)
app_launcher = AppLauncher(args)

print(f"[{time.time()-start:.1f}s] SimulationApp 초기화...", flush=True)
simulation_app = app_launcher.app

print(f"[{time.time()-start:.1f}s] 환경 생성...", flush=True)
env = YourEnv()

print(f"[{time.time()-start:.1f}s] Reset...", flush=True)
obs = env.reset()

print(f"[{time.time()-start:.1f}s] 완료!", flush=True)
```

### 2. 프로세스 상태 모니터링 (별도 터미널)
```bash
watch -n 1 'docker exec zen_yonath ps aux | grep python3 | grep -v grep'
```

### 3. GPU 사용률 모니터링
```bash
watch -n 1 nvidia-smi
```

---

## 🎯 권장 워크플로우

### 개발 단계
1. **빠른 검증 스크립트** (1-2분)
   - stdout unbuffered
   - 경고 억제
   - 1 episode만
   
2. **중간 테스트** (5-10분)
   - 10 episodes
   - 로그 CUDA 경고 필터링
   
3. **전체 트레이닝** (수 시간)
   - Background 실행
   - TensorBoard 모니터링

### 프로덕션 단계
- 모든 최적화 적용
- 로그를 파일로 저장
- 주기적인 체크포인트 저장

---

## 📚 참고 자료

- [Isaac Sim Standalone Python](https://docs.omniverse.nvidia.com/isaacsim/latest/manual_standalone_python.html)
- [Isaac Lab Performance Optimization](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/04_performance.html)
- [NVIDIA Omniverse Forums](https://forums.developer.nvidia.com/c/omniverse/simulation/69)
- [Isaac Lab GitHub Discussions](https://github.com/isaac-sim/IsaacLab/discussions)

---

## 🚀 다음 단계

이 최적화를 적용한 후:
1. ✅ 빠른 환경 검증 (1-2분)
2. ✅ SAC 트레이닝 시작 (백그라운드)
3. ✅ TensorBoard로 실시간 모니터링
