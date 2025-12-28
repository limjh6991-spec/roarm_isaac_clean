# Vision RL 전략 연구: Search → Approach → Grasp

## 연구 배경

현재 문제: 카메라에서 큐브가 보이지 않음 (Red pixels: 0%)
필요한 전략: 3단계 Active Vision RL

---

## 3단계 전략 구조

### Phase 1: SEARCH (탐색)
**목표**: 팔을 움직여 카메라에서 큐브를 찾음

| 접근법 | 설명 | 출처 |
|--------|------|------|
| **Active Vision** | 카메라를 움직여 시야 탐색 | AV-ALOHA (arxiv) |
| **Exploration Reward** | 새로운 영역 탐색 시 보상 | Active RL 연구 |
| **Object Detection Reward** | 이미지에서 빨간색 픽셀 감지 시 보상 | Visual Servoing |

**보상 설계**:
```python
# Phase 1: Search Reward
red_pixels = detect_red_pixels(image)
search_reward = 10.0 if red_pixels > threshold else -0.01
```

### Phase 2: APPROACH (접근)
**목표**: 큐브 발견 후 EE를 큐브 방향으로 이동

| 접근법 | 설명 | 출처 |
|--------|------|------|
| **Image-Based Visual Servoing (IBVS)** | 이미지 특징 기반 제어 | Classical VS |
| **Pose-Based Visual Servoing (PBVS)** | 3D 포즈 추정 기반 제어 | Classical VS |
| **End-to-End RL** | CNN이 이미지→액션 직접 매핑 | Isaac Lab |

**보상 설계**:
```python
# Phase 2: Approach Reward (큐브가 보일 때만)
if red_pixels > threshold:
    # 이미지 중앙으로 큐브 유도
    cube_center = get_cube_center(image)
    image_center = (42, 42)  # 84x84 이미지 중앙
    center_error = distance(cube_center, image_center)
    approach_reward = 5.0 * exp(-center_error)
    
    # 물리 거리 보상 (보조)
    approach_reward += 10.0 * (prev_dist - current_dist)
```

### Phase 3: GRASP (파지)
**목표**: 큐브 접촉 후 그리퍼로 잡기

| 접근법 | 설명 | 출처 |
|--------|------|------|
| **Contact-based Trigger** | 접촉 감지 시 그리퍼 닫기 | 전통 로보틱스 |
| **Distance Threshold** | EE-Cube 거리 임계값 도달 시 | 현재 구현 |
| **RL Grasping** | 그리퍼 액션도 RL로 학습 | 현재 구현 |

---

## Isaac Lab / Isaac Sim 예제

### 1. TiledCamera API
- 다중 카메라 효율적 렌더링
- Vision RL 환경에 최적화
- Isaac Sim 4.2.0+ 필요

### 2. Franka Vision 예제
GitHub Discussion: "[Question] Franka Lift Cube Task with Visual Observation"
- `FrankaLiftCameraEnvCfg` 구현
- End-effector에 카메라 마운트
- RGB observation 사용

### 3. CartpoleCameraEnv
- 기본 Vision RL 예제
- RGB 정규화 (Isaac Lab 방식)
- `--enable_cameras` 옵션 사용

---

## 제안: 하이브리드 보상 함수

```python
def compute_reward(self, image, ee_pos, cube_pos):
    # 1. Vision-based: 이미지에서 큐브 감지
    red_mask = (image[:,:,0] > 150) & (image[:,:,1] < 100)
    red_ratio = np.sum(red_mask) / image.size
    
    # Phase 1: Search (큐브가 안 보일 때)
    if red_ratio < 0.005:  # 0.5% 미만
        # 탐색 보상: 움직임 장려
        exploration_reward = -0.01  # 시간 패널티
        return exploration_reward
    
    # Phase 2: Approach (큐브가 보일 때)
    # 이미지 중앙으로 큐브 유도
    cube_center = get_centroid(red_mask)
    center_error = np.linalg.norm(cube_center - [42, 42])
    centering_reward = 5.0 * np.exp(-0.1 * center_error)
    
    # 물리 거리 보상 (보조)
    dist = np.linalg.norm(ee_pos - cube_pos)
    distance_reward = 10.0 * (self._prev_dist - dist)
    
    # Phase 3: Grasp (가까울 때)
    if dist < 0.05:
        grasp_reward = 20.0 if self.gripper_width < 0.03 else 0
    else:
        grasp_reward = 0
    
    return centering_reward + distance_reward + grasp_reward
```

---

## 핵심 차이점: 현재 vs 제안

| 항목 | 현재 구현 | 제안 |
|------|----------|------|
| **Search Phase** | ❌ 없음 | ✅ 이미지 기반 탐색 보상 |
| **Approach** | 물리 거리만 | 이미지 중심화 + 물리 거리 |
| **Grasp Trigger** | 거리 기반 | 이미지 + 거리 복합 |
| **카메라 활용** | Observation만 | Observation + Reward |

---

## 다음 단계

1. **카메라 위치 확인**: 현재 카메라가 큐브를 볼 수 있는 위치인지 확인
2. **Search Phase 보상 추가**: 이미지에서 큐브 감지 시 보상
3. **Centering 보상 추가**: 큐브를 이미지 중앙으로 유도
4. **테스트**: 새 보상 함수로 학습 재시작

---

*작성: 2025-12-28*
*참고: Isaac Lab, AV-ALOHA, Visual Servoing 연구*
