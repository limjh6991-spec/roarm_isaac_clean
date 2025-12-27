# RoArm-M3 Actuator Configuration Guide

## 실제 서보 스펙

### 서보 모델별 토크
- **ST3215** (Standard): 30 kg·cm @ 12V = 2.94 Nm
- **ST3235** (Pro): 30 kg·cm @ 12V = 2.94 Nm  
- **Dual-Drive Shoulder**: 60 kg·cm @ 12V = 5.88 Nm

### 각 조인트별 서보 배치

| 조인트 | 서보 | 토크 (Nm) | 회전 범위 | 견딜 무게 |
|--------|------|-----------|-----------|-----------|
| Base | ST3215/ST3235 | 2.94 | 360° | 0.2kg |
| Shoulder | ST3215/ST3235 (Dual) | 5.88 | 180° | 0.3~0.5kg |
| Elbow | ST3215/ST3235 | 2.94 | 225° | 0.2kg |
| Wrist Pitch | ST3215/ST3235 | 2.94 | 135° | 0.2kg |
| Wrist Yaw | ST3215/ST3235 | 2.94 | 270° | 0.2kg |
| Gripper | ST3215/ST3235 | 1.96 | 135° | 0.15kg |

## Isaac Sim Actuator 설정 변환

### Stiffness 계산
```
Stiffness = Torque / Acceptable_Angular_Deflection

허용 처짐 1° (0.0175 rad) 기준:
- Shoulder (5.88 Nm): 5.88 / 0.0175 ≈ 336 Nm/rad
- Standard (2.94 Nm): 2.94 / 0.0175 ≈ 168 Nm/rad
- Gripper (1.96 Nm): 1.96 / 0.0175 ≈ 112 Nm/rad
```

### Damping 계산
```
Damping = 2 × sqrt(Inertia × Stiffness) × Damping_Ratio

Critically damped (ratio=1.0) 기준:
- Shoulder: 2 × sqrt(0.01 × 336) × 1.0 ≈ 3.7 Nm·s/rad
- Standard: 2 × sqrt(0.01 × 168) × 1.0 ≈ 2.6 Nm·s/rad
- Gripper: 2 × sqrt(0.001 × 112) × 1.0 ≈ 0.67 Nm·s/rad
```

## 권장 IsaacLab 설정

### 안전 계수 적용 (2x)
실제 시뮬레이션에서는 안정성을 위해 2배 여유 적용

```python
stiffness={
    "base_link_to_link1": 400.0,      # 168 × 2 ≈ 336
    "link1_to_link2": 800.0,          # 336 × 2 ≈ 672 (Shoulder, Dual-drive)
    "link2_to_link3": 400.0,          # 168 × 2 ≈ 336 (Elbow)
    "link3_to_link4": 400.0,          # 168 × 2 ≈ 336 (Wrist Pitch)
    "link4_to_link5": 400.0,          # 168 × 2 ≈ 336 (Wrist Yaw)
    "link5_to_gripper_link": 250.0,   # 112 × 2 ≈ 224 (Gripper)
}

damping={
    "base_link_to_link1": 8.0,        # 2.6 × 3
    "link1_to_link2": 12.0,           # 3.7 × 3 (Shoulder, 강화)
    "link2_to_link3": 8.0,            # 2.6 × 3
    "link3_to_link4": 8.0,            # 2.6 × 3
    "link4_to_link5": 8.0,            # 2.6 × 3
    "link5_to_gripper_link": 5.0,     # 0.67 × 7
}
```

## 카메라 추가 시 조정

### D405 카메라 (60g @ 5cm forward, 5cm up)
Shoulder와 Elbow에 추가 부하

```
추가 토크 = 0.06kg × 0.05m × 9.81 = 0.0294 Nm
```

### 카메라 보상 설정
```python
stiffness={
    "base_link_to_link1": 400.0,
    "link1_to_link2": 1000.0,         # Shoulder 강화 (800 → 1000)
    "link2_to_link3": 600.0,          # Elbow 강화 (400 → 600)
    "link3_to_link4": 500.0,          # Wrist 강화 (400 → 500)
    "link4_to_link5": 400.0,
    "link5_to_gripper_link": 250.0,
}

damping={
    "base_link_to_link1": 8.0,
    "link1_to_link2": 20.0,           # Shoulder 강화 (12 → 20)
    "link2_to_link3": 15.0,           # Elbow 강화 (8 → 15)
    "link3_to_link4": 12.0,           # Wrist 강화 (8 → 12)
    "link4_to_link5": 8.0,
    "link5_to_gripper_link": 5.0,
}
```

## 문제 해결 가이드

### 증상별 조정

1. **로봇이 뒤로 꺾임** (현재 문제)
   - Stiffness가 너무 낮음
   - Shoulder/Elbow stiffness 증가
   - Damping도 비례하여 증가

2. **로봇이 떨림/진동**
   - Damping이 너무 낮음
   - Damping 증가 (Stiffness의 2~5%)

3. **로봇이 너무 뻣뻣함**
   - Stiffness가 너무 높음
   - 실제 토크 기반 값으로 감소

4. **로봇이 천천히 떨어짐**
   - Damping이 너무 높음
   - Damping 감소

## 참고 자료

- RoArm-M3 Wiki: https://www.waveshare.com/wiki/RoArm-M3
- ST3215 Servo Spec: 30 kg·cm @ 12V
- Isaac Sim Articulation: PD Control
