# RoArm-M3 RL 학습 개선 계획

## 🔍 현재 문제

### 학습 성능
- 평균 보상: -2,700 ~ -3,000
- 성공률: 0%
- 에피소드: 항상 타임아웃 (601 steps)
- 개선도: 거의 없음

## ❌ 발견된 문제들

### 1. **보상 함수가 너무 Sparse**
```python
# 현재 보상:
distance_reward = -distance * 10.0  # 거리만 패널티
success_bonus = 100.0 (성공 시에만)

# 문제:
- 큐브에 가까이 가도 보상 없음
- 큐브를 잡아도 보상 없음
- 목표 방향으로 움직여도 보상 없음
```

### 2. **관찰 공간이 부족**
```python
# 현재 관찰 (15 dim):
- Joint positions (8)
- End-effector position (3)
- Cube position (3)
- Distance to target (1)

# 빠진 정보:
- End-effector와 큐브 사이 거리
- 큐브와 목표 사이 벡터
- 그리퍼가 열렸는지/닫혔는지
- 큐브를 잡고 있는지
```

### 3. **액션 스케일이 너무 작음**
```python
max_deltas = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.005, 0.005]
# 600 steps에 목표 도달하기 어려움
```

### 4. **Joint Drive 설정이 과도**
```python
stiffness = 10000  # 너무 강함
damping = 1000     # 너무 큼
# → 부자연스러운 움직임, 큐브를 밀침
```

## ✅ 개선 방안

### 1. **Dense Reward 함수** (핵심!)
```python
# Shaped Reward:
1. 큐브에 접근: -ee_to_cube_distance * 5.0
2. 큐브 잡기: is_grasped * 10.0
3. 목표로 이동: -cube_to_target_distance * 10.0
4. 성공: success * 100.0
5. 시간 패널티: -0.01 (효율성)

# 예시:
- 큐브 멀리: -5.0 (ee 1m)
- 큐브 가까이: -1.0 (ee 0.2m)
- 큐브 잡음: +10.0 - 3.0 (target 0.3m)
- 목표 도달: +100.0
```

### 2. **관찰 공간 확장**
```python
obs = [
    joint_positions (8),
    ee_position (3),
    cube_position (3),
    target_position (3),  # 추가
    ee_to_cube_vector (3),  # 추가
    cube_to_target_vector (3),  # 추가
    gripper_width (1),  # 추가
    is_grasped (1),  # 추가
]
# Total: 25 dim (기존 15 → 25)
```

### 3. **액션 스케일 조정**
```python
# 팔: 더 빠르게
max_deltas = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.01, 0.01]

# 또는 속도 제어로 변경
max_velocities = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.05, 0.05]
```

### 4. **Joint Drive 완화**
```python
# 더 부드러운 움직임:
stiffness = 5000  # 10000 → 5000
damping = 500     # 1000 → 500
max_force = 500   # 1000 → 500
```

### 5. **Curriculum Learning** (선택)
```python
# Stage 1: 큐브가 가까이 (0.2m)
# Stage 2: 큐브가 중간 (0.3m)
# Stage 3: 큐브가 멀리 (0.4m)
# Stage 4: 랜덤 위치
```

## 🚀 우선순위

### 1단계: Dense Reward (필수!) ⭐⭐⭐
- 구현 시간: 30분
- 효과: 매우 큼
- 학습 성공 가능성: 70% → 90%

### 2단계: 관찰 공간 확장 ⭐⭐
- 구현 시간: 20분
- 효과: 큼
- 정책이 더 똑똑해짐

### 3단계: Joint Drive 조정 ⭐
- 구현 시간: 10분
- 효과: 중간
- 물리가 더 현실적

### 4단계: 액션 스케일 조정 ⭐
- 구현 시간: 5분
- 효과: 작음
- 더 빠른 움직임

## 📊 예상 결과

### 현재 (Sparse Reward):
```
50K steps: -2,800 보상, 0% 성공
100K steps: -2,600 보상, 0% 성공
```

### 개선 후 (Dense Reward):
```
50K steps: -500 보상, 10% 성공
100K steps: +20 보상, 40% 성공
200K steps: +60 보상, 70% 성공
```

## 🎯 다음 단계

**바로 시작할까요?**

1. **Dense Reward 구현** (가장 중요!)
2. 학습 재시작 (100K steps)
3. 결과 확인
4. 추가 튜닝

**예상 시간**: 1시간 구현 + 3시간 학습 = 4시간
