# Vision RL CNN Small Object Issue - Research Report

## 문제 현상

### 현재 상황
- **입력 이미지**: 84x84x3 RGB
- **큐브 크기**: 4cm (화면에서 ~130 pixels, 1.8%)
- **CNN 오차**: 9.32cm (평균값 예측 = 9.2cm)
- **<1cm 정확도**: 0.0%

### 결론
> **CNN이 평균값만 예측함** - 이미지에서 큐브 위치를 학습하지 못함

---

## 원인 분석

### 1. Small Object Detection 문제

| 원인 | 설명 |
|------|------|
| **해상도 손실** | CNN pooling 레이어가 작은 객체 피처를 소멸시킴 |
| **픽셀 수 부족** | 84x84 이미지에서 4cm 큐브는 ~130 픽셀 (1.8%) |
| **다운샘플링** | NatureCNN: 84→20→9→7 로 축소 시 큐브 피처 손실 |
| **입력과 출력 연결 부재** | CNN이 큐브 위치와 이미지 피처 간 관계를 못 찾음 |

### 2. Position Regression 평균값 예측 문제

| 원인 | 설명 |
|------|------|
| **MSE Loss 특성** | 평균값 예측이 랜덤 예측보다 낮은 loss → 안전한 선택 |
| **약한 상관관계** | 이미지 피처와 큐브 위치 사이 연결고리 희미 |
| **모델 복잡도 부족** | 작은 객체 위치화에 필요한 표현력 부족 |

---

## 해결 방법

### A. 이미지/카메라 관련

| 방법 | 설명 | 난이도 |
|------|------|--------|
| **1. 해상도 증가** | 84x84 → 224x224 (9배 픽셀 증가) | 쉬움 |
| **2. 카메라 줌/근접** | 큐브가 화면의 20%+ 차지하도록 | 쉬움 |
| **3. 이미지 크롭** | 관심 영역만 추출 후 학습 | 중간 |
| **4. Super-Resolution** | 이미지 업스케일링 사전 처리 | 중간 |

### B. CNN 아키텍처 관련

| 방법 | 설명 | 난이도 |
|------|------|--------|
| **1. Feature Pyramid Network (FPN)** | 다중 스케일 피처 결합 | 중간 |
| **2. Dilated Convolution** | 넓은 수용 영역 유지하며 해상도 보존 | 중간 |
| **3. 적은 Pooling** | Stride 줄이고 해상도 유지 | 쉬움 |
| **4. Spatial Softmax** | 키포인트 위치 추출 레이어 추가 | 중간 |

### C. 학습 관련

| 방법 | 설명 | 난이도 |
|------|------|--------|
| **1. Pretrained Encoder** | ResNet/ViT freeze + head만 학습 | 중간 |
| **2. Auxiliary Loss** | 큐브 heatmap 예측 보조 태스크 | 중간 |
| **3. Focal Loss** | 어려운 샘플에 가중치 부여 | 쉬움 |
| **4. Data Augmentation** | 크롭, 줌, 색상 변형 | 쉬움 |

---

## 권장 해결책 (우선순위)

### 1단계: 해상도 및 카메라 조정 (즉시)
```python
# 해상도 증가
self.render_product = rep.create.render_product(camera_path, (224, 224))

# observation_space 수정
self.observation_space = spaces.Box(
    low=0.0, high=1.0,
    shape=(4, 224, 224),  # 기존 84x84 에서 변경
    dtype=np.float32
)
```

### 2단계: 카메라 위치 최적화
```python
# 큐브 중심에 더 가깝게
self.camera = rep.create.camera(
    position=(0.35, 0.15, 0.25),  # 더 가깝게
    look_at=(0.22, 0.0, 0.05),
    clipping_range=(0.01, 10.0),
    focal_length=35  # 줌 효과
)
```

### 3단계: CNN 아키텍처 개선
```python
# Spatial Softmax 추가
class ImprovedCNN(nn.Module):
    def __init__(self):
        # ... convolution layers ...
        self.spatial_softmax = SpatialSoftmax()  # 키포인트 추출
        
    def forward(self, x):
        features = self.conv_layers(x)
        keypoints = self.spatial_softmax(features)
        return keypoints
```

### 4단계: Pretrained Encoder + Fine-tuning
```python
# ResNet18 encoder (frozen)
encoder = models.resnet18(pretrained=True)
encoder.fc = nn.Identity()  # 마지막 레이어 제거
for param in encoder.parameters():
    param.requires_grad = False  # freeze

# 작은 head만 학습
head = nn.Linear(512, 3)  # x, y, z 출력
```

---

## Isaac Sim 관련 추가 권장사항

1. **카메라 설정 확인**
   - `clipping_range` 설정 필수
   - 카메라 prim path 사용 (`rep.create.camera` 객체 대신)

2. **Domain Randomization**
   - 조명, 텍스처 랜덤화로 일반화 향상

3. **고해상도 센서**
   - 84x84는 Atari 게임 기준, 로봇 조작에는 부족

---

## 참고 자료

### 논문 및 기술 문서
1. [FPN for Small Object Detection](https://arxiv.org/abs/1612.03144)
2. [Super-Resolution for Object Detection](https://arxiv.org/abs/1707.02921)
3. [Spatial Softmax for Robot Control](https://arxiv.org/abs/1509.06113)

### Isaac Sim 문서
1. [Camera Configuration](https://nvidia.com/isaac-sim/cameras)
2. [Synthetic Data Generation](https://nvidia.com/isaac-sim/sdg)
3. [RL with Visual Observations](https://isaac-sim.github.io/IsaacLab)

### 커뮤니티 리소스
1. [CNN predicts mean value - StackExchange](https://stackexchange.com)
2. [Small Object Detection challenges](https://quantumobile.com)

---

## 다음 단계

1. [ ] 해상도 224x224로 증가하고 테스트
2. [ ] 카메라를 큐브에 더 가깝게 배치
3. [ ] CNN 아키텍처에 Spatial Softmax 추가
4. [ ] Pretrained ResNet18 encoder 적용 테스트

---

*작성: 2025-12-27*
*기반: Isaac Sim 5.1 + PyTorch 2.7.0*
