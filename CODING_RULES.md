# 코딩 규칙 (Coding Rules)

## 🎯 코딩 전 필수 체크리스트

### ✅ STEP 1: 자료 조사 (Research)
```
[ ] 온라인 최신 정보 검색
    - Isaac Sim 공식 문서 확인
    - GitHub Issues 검색
    - Stack Overflow / Forums 확인
    - 최신 릴리스 노트 확인

[ ] API 버전 확인
    - Isaac Sim 버전: __________
    - Python 버전: __________
    - 주요 라이브러리 버전 확인 (별도 문서 참조)

[ ] 관련 이슈 점검
    - docs/TROUBLESHOOTING.md 확인
    - GitHub Issues 검색
    - 이전 해결 방법 리뷰

[ ] API 문서 재학습
    - 공식 API 문서 읽기
    - 예제 코드 확인
    - Deprecated API 체크
```

### ✅ STEP 2: 계획 수립 (Planning)
```
[ ] 작업 범위 명확화
    - 목적: 무엇을 만들 것인가?
    - 입력: 무엇을 받는가?
    - 출력: 무엇을 반환하는가?
    - 부작용: 어떤 상태를 변경하는가?

[ ] 인터페이스 설계
    - 함수 시그니처 정의
    - 클래스 구조 정의
    - 의존성 파악

[ ] 테스트 계획
    - 단위 테스트 시나리오
    - 통합 테스트 시나리오
    - Edge cases 정의
```

### ✅ STEP 3: 코딩 규칙 확인 (Review Rules)
```
[ ] 이 문서의 코딩 규칙 재확인
[ ] 프로젝트별 컨벤션 확인
[ ] API 버전 체크리스트 확인 (docs/API_CHECKLIST.md)
```

---

## 📐 코딩 스타일

### Python PEP 8 준수
```python
# ✅ Good
def calculate_reward(
    distance: float,
    success: bool,
    time_step: int
) -> float:
    """보상 계산
    
    Args:
        distance: 목표까지 거리 (m)
        success: 성공 여부
        time_step: 현재 타임스텝
        
    Returns:
        계산된 보상값
    """
    if success:
        return 100.0
    return -0.01 * time_step

# ❌ Bad
def calc_rew(d,s,t):
    if s:return 100.0
    return -0.01*t
```

### 타입 힌팅 필수
```python
# ✅ Good
from typing import List, Dict, Optional, Tuple
import numpy as np

def get_joint_positions(
    robot: Articulation,
    joint_indices: Optional[List[int]] = None
) -> np.ndarray:
    pass

# ❌ Bad
def get_joint_positions(robot, joint_indices=None):
    pass
```

### Docstring 작성
```python
# ✅ Google Style (권장)
def process_observation(obs: np.ndarray) -> Dict[str, np.ndarray]:
    """관찰값을 처리하여 딕셔너리로 변환
    
    Args:
        obs: 원시 관찰값 (shape: [N,])
        
    Returns:
        처리된 관찰값 딕셔너리
        - 'robot_state': 로봇 상태 (shape: [6,])
        - 'object_pos': 물체 위치 (shape: [3,])
        
    Raises:
        ValueError: obs 크기가 맞지 않을 때
        
    Example:
        >>> obs = np.random.randn(25)
        >>> result = process_observation(obs)
        >>> result['robot_state'].shape
        (6,)
    """
    pass
```

---

## 🔍 API 사용 규칙

### 1. 버전 확인 필수
```python
# ✅ Good: 버전 체크 포함
import omni.isaac.core

# Isaac Sim 5.0+ API 사용
if hasattr(omni.isaac.core, '__version__'):
    print(f"Isaac Core version: {omni.isaac.core.__version__}")

# Deprecated API 경고 처리
import warnings
with warnings.catch_warnings():
    warnings.filterwarnings('ignore', category=DeprecationWarning)
    from omni.isaac.core.articulations import Articulation

# ❌ Bad: 버전 확인 없이 사용
from some_module import some_function
```

### 2. Deprecated API 회피
```python
# ✅ Good: 최신 API 사용 (Isaac Sim 5.0+)
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation

# ❌ Bad: Deprecated API (Isaac Sim 4.x)
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
```

### 3. API 변경 대응
```python
# ✅ Good: Fallback 제공
try:
    from isaacsim.core.api import World  # Isaac Sim 5.0+
except ImportError:
    from omni.isaac.core.world import World  # Fallback for 4.x
    print("Warning: Using deprecated API")
```

---

## 🧪 테스트 규칙

### 1. 단위 테스트 필수
```python
# tests/test_reward_function.py
import unittest
import numpy as np
from envs.roarm_pick_place_env import calculate_reward

class TestRewardFunction(unittest.TestCase):
    def test_success_reward(self):
        """성공 시 큰 보상"""
        reward = calculate_reward(distance=0.01, success=True, time_step=10)
        self.assertGreater(reward, 90.0)
    
    def test_failure_penalty(self):
        """실패 시 시간 패널티"""
        reward = calculate_reward(distance=1.0, success=False, time_step=100)
        self.assertLess(reward, -0.5)
```

### 2. 통합 테스트
```python
# tests/integration/test_env_creation.py
def test_environment_creation():
    """환경 생성 및 초기화 테스트"""
    from envs.roarm_pick_place_env import RoArmPickPlaceEnv
    
    env = RoArmPickPlaceEnv()
    obs = env.reset()
    
    assert obs.shape == (25,), "Observation shape mismatch"
    assert env.action_space.shape == (8,), "Action space shape mismatch"
```

### 3. URDF 검증 테스트
```python
# 1_urdf_workspace/tests/test_urdf_validity.py
def test_urdf_joint_types():
    """URDF 조인트 타입 확인"""
    import xml.etree.ElementTree as ET
    
    tree = ET.parse('assets/roarm_m3/urdf/roarm_m3_multiprim.urdf')
    root = tree.getroot()
    
    gripper_joints = [j for j in root.findall('.//joint') 
                      if 'gripper' in j.get('name')]
    
    for joint in gripper_joints:
        assert joint.get('type') == 'prismatic', \
            f"Gripper joint {joint.get('name')} should be prismatic"
```

---

## 📝 문서화 규칙

### 1. 코드 변경 시 문서 업데이트
```
[ ] README.md 업데이트
[ ] CHANGELOG.md 항목 추가
[ ] API 문서 업데이트
[ ] 예제 코드 업데이트
```

### 2. Commit 메시지 규칙
```bash
# Format: <type>(<scope>): <subject>

# ✅ Good
feat(urdf): Add prismatic gripper joints
fix(env): Correct gripper joint indices
docs(readme): Update installation guide
test(env): Add reward function tests

# ❌ Bad
Update files
Fix bug
WIP
```

### 3. PR/Issue 템플릿
```markdown
## 변경 사항
- 무엇을 변경했는가?

## 동기
- 왜 변경했는가?

## 테스트
- 어떻게 테스트했는가?

## 체크리스트
- [ ] 코딩 규칙 준수
- [ ] API 버전 확인
- [ ] 테스트 통과
- [ ] 문서 업데이트
```

---

## 🚨 오류 처리

### 1. 명시적 오류 처리
```python
# ✅ Good
def load_urdf(path: str) -> Articulation:
    """URDF 로드
    
    Raises:
        FileNotFoundError: URDF 파일이 없을 때
        ValueError: URDF 파싱 실패 시
        RuntimeError: Isaac Sim 초기화 안 됨
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"URDF not found: {path}")
    
    try:
        robot = import_urdf(path)
    except Exception as e:
        raise ValueError(f"Failed to parse URDF: {e}")
    
    return robot

# ❌ Bad
def load_urdf(path):
    robot = import_urdf(path)  # 오류 처리 없음
    return robot
```

### 2. 로깅 사용
```python
import logging

# ✅ Good: 로깅 설정
logger = logging.getLogger(__name__)

def train_agent(timesteps: int):
    logger.info(f"Starting training for {timesteps} steps")
    try:
        # 학습 코드
        pass
    except Exception as e:
        logger.error(f"Training failed: {e}", exc_info=True)
        raise

# ❌ Bad: print 사용
def train_agent(timesteps):
    print(f"Starting training for {timesteps} steps")
    # 학습 코드
```

---

## 🔐 보안 및 성능

### 1. 하드코딩 금지
```python
# ✅ Good: 설정 파일 사용
import yaml

with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)
    
URDF_PATH = config['urdf_path']
LOG_DIR = config['log_dir']

# ❌ Bad: 하드코딩
URDF_PATH = "/home/user/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_multiprim.urdf"
```

### 2. 성능 고려
```python
# ✅ Good: NumPy 벡터화
import numpy as np

def calculate_distances(positions: np.ndarray, target: np.ndarray) -> np.ndarray:
    """벡터화된 거리 계산"""
    return np.linalg.norm(positions - target, axis=1)

# ❌ Bad: 반복문 사용
def calculate_distances(positions, target):
    distances = []
    for pos in positions:
        dist = np.sqrt(np.sum((pos - target) ** 2))
        distances.append(dist)
    return np.array(distances)
```

---

## 📚 참고 자료

### 반드시 확인할 문서
1. **API_CHECKLIST.md**: API 버전 및 호환성
2. **TROUBLESHOOTING.md**: 알려진 이슈 및 해결 방법
3. **PROJECT_STRUCTURE.md**: 프로젝트 구조
4. **DEVELOPMENT_WORKFLOW.md**: 개발 워크플로우

### 온라인 리소스
- Isaac Sim 공식 문서: https://docs.omniverse.nvidia.com/isaacsim/latest/
- Stable-Baselines3 문서: https://stable-baselines3.readthedocs.io/
- Isaac Lab 마이그레이션 가이드: https://isaac-sim.github.io/IsaacLab/

---

## ✅ 코딩 전 체크리스트 요약

```
코딩 시작 전 반드시 확인:

1. [ ] 온라인 최신 정보 검색 완료
2. [ ] API 버전 확인 (docs/API_CHECKLIST.md)
3. [ ] 관련 이슈 점검 (docs/TROUBLESHOOTING.md)
4. [ ] API 문서 재학습
5. [ ] 코딩 규칙 재확인 (이 문서)
6. [ ] 작업 범위 명확화
7. [ ] 테스트 계획 수립

코딩 완료 후 반드시 확인:

1. [ ] 단위 테스트 작성 및 통과
2. [ ] 통합 테스트 통과
3. [ ] 문서 업데이트
4. [ ] Commit 메시지 규칙 준수
5. [ ] 코드 리뷰 (가능하면)
```

---

**Last Updated**: 2025-10-20  
**Version**: 1.0  
**Maintainer**: RoArm Isaac Lab Team
