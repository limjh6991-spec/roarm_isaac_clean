# Isaac Sim Python 스크립팅 가이드

**날짜**: 2025-10-19  
**목적**: Isaac Sim 5.0에서 Python 스크립트 올바르게 실행하기

---

## 🔑 핵심 사항

### 1. 올바른 Python 실행 방법

**❌ 잘못된 방법**:
```bash
python script.py                    # isaacsim 모듈 인식 안됨
python3 script.py                   # 동일 문제
source ~/isaacsim-venv/bin/activate
python script.py                    # 여전히 안됨
```

**✅ 올바른 방법**:
```bash
~/isaacsim/python.sh script.py      # Isaac Sim Python 환경 사용
```

또는 전체 경로:
```bash
/home/$(whoami)/isaacsim/python.sh /path/to/script.py
```

---

### 2. stdout Buffering 문제 해결

**문제**: `print()` 출력이 터미널에 표시되지 않음

**해결**:
```python
import sys

# 스크립트 맨 위에 추가 (SimulationApp import 전)
sys.stdout = sys.stderr
```

**이유**: Isaac Sim의 내부 로깅 시스템이 stdout을 캡처하므로 stderr로 리디렉션 필요

---

### 3. 기본 스크립트 구조

```python
#!/usr/bin/env python3
import sys

# 1. stdout buffering 해결
sys.stdout = sys.stderr

# 2. SimulationApp import (다른 Isaac Sim 모듈보다 먼저!)
from isaacsim import SimulationApp

# 3. SimulationApp 생성 (즉시!)
simulation_app = SimulationApp({"headless": False})

# 4. 이제 다른 Isaac Sim 모듈 import 가능
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

# 5. 메인 코드
print("Hello from Isaac Sim!")  # 이제 출력됨

world = World()
world.scene.add_default_ground_plane()

cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0.0, 0.0, 0.5]),
        size=0.05,
        color=np.array([1.0, 0.0, 0.0]),
    )
)

world.reset()

for i in range(100):
    world.step(render=True)

# 6. 종료
simulation_app.close()
```

---

### 4. 실행 예제

**간단한 테스트**:
```bash
~/isaacsim/python.sh scripts/test_basic_isaac.py
```

**로그 저장**:
```bash
~/isaacsim/python.sh scripts/test_basic_isaac.py 2>&1 | tee /tmp/output.log
```

**백그라운드 실행**:
```bash
~/isaacsim/python.sh scripts/test_basic_isaac.py &
```

---

### 5. 환경 변수 설정 (필요 시)

```bash
# Isaac Sim Python 경로 확인
~/isaacsim/python.sh -c "import sys; print(sys.executable)"

# 출력: /home/user/isaacsim/kit/python/bin/python3

# PYTHONPATH에 추가 (선택사항)
export ISAAC_PATH=~/isaacsim
export PYTHONPATH=$PYTHONPATH:$ISAAC_PATH
```

---

### 6. Conda 환경과 충돌 방지

**문제**: Conda 환경이 활성화되어 있으면 충돌 가능

**해결**:
```bash
# Conda 비활성화
conda deactivate

# 그 다음 Isaac Sim Python 사용
~/isaacsim/python.sh script.py
```

**경고 메시지**:
```
Warning: running in conda env, please deactivate before executing this script
```

---

### 7. Headless 모드 (GUI 없이 실행)

```python
simulation_app = SimulationApp({"headless": True})
```

**용도**:
- 서버 환경
- 대량 데이터 생성
- CI/CD 파이프라인

---

### 8. 문제 해결 (Troubleshooting)

#### 문제 1: `ModuleNotFoundError: No module named 'isaacsim'`
**원인**: 일반 python 사용  
**해결**: `~/isaacsim/python.sh` 사용

#### 문제 2: Print 출력 안보임
**원인**: stdout buffering  
**해결**: `sys.stdout = sys.stderr` 추가

#### 문제 3: Import 순서 오류
**원인**: SimulationApp보다 먼저 다른 모듈 import  
**해결**: 
```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({...})  # 즉시 생성
# 이제 다른 모듈 import
from isaacsim.core.api import World
```

#### 문제 4: 스크립트가 즉시 종료됨
**원인**: 
- Python 예외 발생 (stderr 확인 필요)
- Import 오류
- URDF/USD 경로 문제

**해결**:
```bash
# stderr 확인
~/isaacsim/python.sh script.py 2>&1 | grep -i "error\|exception\|traceback"
```

---

### 9. 성공적인 실행 확인

```bash
~/isaacsim/python.sh scripts/test_flush.py 2>&1 | tee /tmp/test.log
```

**예상 출력**:
```
🚀 Isaac Sim 테스트 시작
1. SimulationApp 생성 중...
   ✅ SimulationApp 생성 완료
2. Isaac Sim 모듈 import 중...
   ✅ Import 완료
3. World 생성 중...
   ✅ World 생성 완료
...
   ⏱️  Step 0/300
   ⏱️  Step 60/300
...
✅ 모든 테스트 통과!
```

---

### 10. 환경 파일 경로 확인

**RoArm Pick and Place 환경 사용 시**:
```python
# 스크립트 위치: scripts/demo.py
# 환경 위치: envs/roarm_pick_place_env.py

import sys
from pathlib import Path

# 프로젝트 루트를 PYTHONPATH에 추가
project_root = Path(__file__).parent.parent
sys.path.append(str(project_root))

# 이제 import 가능
from envs.roarm_pick_place_env import RoArmPickPlaceEnv
```

---

## ✅ 검증된 워크플로우

1. **기본 테스트** (성공 ✅):
   ```bash
   ~/isaacsim/python.sh scripts/test_flush.py
   ```

2. **환경 데모** (진행 중):
   ```bash
   ~/isaacsim/python.sh scripts/demo_roarm_fixed.py --episodes 1 --steps 100
   ```

3. **학습 시작** (대기):
   ```bash
   ~/isaacsim/python.sh scripts/train_roarm_rl.py --mode train --timesteps 50000
   ```

---

**작성**: GitHub Copilot (Jarvis)  
**검증**: 2025-10-19, Isaac Sim 5.0.0-rc.45  
**소요 시간**: 기본 테스트 10분, 디버깅 30분
