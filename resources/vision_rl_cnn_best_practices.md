# Vision RL CNN Object Recognition - Best Practices 2024

## 개요

Vision-based Reinforcement Learning (VRL)에서 CNN이 객체(큐브)를 인식하도록 학습시키는 최신 기법 정리.

---

## 1. 핵심 과제: 왜 Vision RL이 어려운가?

### Sample Inefficiency
| 방식 | 입력 차원 | 일반적 수렴 시간 |
|------|-----------|------------------|
| State RL | ~20 | 50K ~ 200K steps |
| Vision RL | 28,224 (84×84×4) | **1M ~ 10M steps** |

### Credit Assignment 문제
- CNN이 "빨간 픽셀 = 큐브" 학습 필요
- 보상 신호와 시각 입력 간 연결 학습 시간 필요

---

## 2. 해결 방법: Pretrained Encoder Pipeline

### 핵심 아이디어
```
기존: [이미지] → CNN (처음부터 학습) → Policy
개선: [이미지] → Pretrained CNN (고정) → Policy (학습)
```

### NVIDIA Isaac Sim 권장 접근법

1. **ResNet18 (Frozen)** + Spatial Softmax
   - ImageNet pretrained 모델 사용
   - 1024-D 특징 벡터 추출
   - Policy는 이 특징만 학습
   
2. **Vision Transformer (ViT)**  
   - `vit_tiny_patch16_224` 사용
   - Classification head 제거 후 feature 추출

### 구현 예시 (PyTorch)
```python
import torchvision.models as models
import torch.nn as nn

class PretrainedEncoder(nn.Module):
    def __init__(self):
        super().__init__()
        # ResNet18 pretrained on ImageNet
        resnet = models.resnet18(pretrained=True)
        # Remove classification head
        self.encoder = nn.Sequential(*list(resnet.children())[:-2])
        # Freeze all parameters
        for param in self.encoder.parameters():
            param.requires_grad = False
        
        # Spatial softmax for 2D keypoints
        self.spatial_softmax = SpatialSoftmax()
        
    def forward(self, x):
        features = self.encoder(x)  # [B, 512, 7, 7]
        keypoints = self.spatial_softmax(features)  # [B, 1024]
        return keypoints
```

---

## 3. SEER: Stored Embeddings for Efficient RL

### 핵심 기법 (NeurIPS)
1. **CNN 하위 레이어 고정**: 초기 학습 후 freeze (빠른 수렴)
2. **Latent Vector 저장**: Replay buffer에 이미지 대신 특징 벡터 저장
   - 메모리 절약: 84×84×4 → 512차원
   - 더 큰 buffer 사용 가능

### 효과
- 메모리 사용량 **90% 감소**
- 학습 속도 **2~3배 향상**

---

## 4. Object-Centric World Models

### Model-Based RL 접근
1. **객체 세그멘테이션**: 큐브에 마스크 적용
2. **Pretrained Vision 모델로 특징 추출**
3. **World Model**: 다음 상태 예측
4. **Policy 학습**: 상상된 trajectory에서 학습

### 장점
- 작은 객체, 동적 요소에 효과적
- Sample efficiency 극대화

---

## 5. SAC + CNN 최적화 팁

### 하이퍼파라미터
| 파라미터 | 권장값 | 설명 |
|----------|--------|------|
| `ent_coef` | auto 또는 0.2 | 높으면 탐색 증가 |
| `tau` | 0.005 | Target network 업데이트 속도 |
| `learning_rate` | 3e-4 | Adam optimizer |
| `buffer_size` | 1M+ | Vision RL은 큰 버퍼 필요 |

### CNN 아키텍처 팁
1. **Nature CNN**: 기본, 단순한 작업에 적합
2. **ResNet**: 복잡한 시각 환경
3. **Self-Attention 추가**: 객체 위치 파악 향상

### 초기 탐색 강화
```python
# 처음 10K steps는 random action
if total_steps < 10000:
    action = env.action_space.sample()
else:
    action, _ = model.predict(obs)
```

---

## 6. Domain Randomization (Sim-to-Real)

### Isaac Sim에서 적용
```python
# 텍스처, 조명, 카메라 위치 랜덤화
rep.randomizer.register(
    rep.randomize.texture([cube_prim]),
    rep.randomize.rotation(cube_prim),
    rep.randomize.light_intensity(light)
)
```

### 효과
- 다양한 시각 조건에서 일반화
- Sim ↔ Real 전이 개선

---

## 7. NVidia Isaac 관련 도구

### Foundation Models
| 모델 | 용도 |
|------|------|
| **FoundationPose** | 6D 자세 추정 |
| **FoundationStereo** | 깊이 추정 |
| **SyntheticaDETR** | 객체 탐지 |
| **COMPASS** | 비전 기반 이동 |
| **GR00T N1.5** | 휴머노이드 조작 |

### Isaac Manipulator
- AI 기반 로봇 팔 개발 라이브러리
- 인식, 이해, 상호작용 지원

---

## 8. 권장 학습 전략

### Option A: 장기 End-to-End 학습
- 현재 환경 유지
- **1M+ steps** 학습 (6~12시간)
- 장점: 구현 간단

### Option B: Pretrained Encoder 적용
1. ResNet18 pretrained 로드
2. Encoder freeze
3. Policy만 학습
- 장점: 빠른 수렴, 적은 데이터

### Option C: Curriculum Learning
1. Stage 1: 큐브만 화면에 (단순)
2. Stage 2: 로봇 추가
3. Stage 3: 배경 복잡화
- 장점: 점진적 난이도 증가

---

## 참고 자료

1. [arxiv.org] Object-Centric Model-Based RL
2. [openreview.net] Pretrained Visual Representations for RL
3. [NeurIPS] SEER: Stored Embeddings for Efficient RL
4. [NVIDIA Isaac Sim] Visual RL Pipeline Documentation
5. [CleanRL] SAC Implementation Best Practices

---

*작성일: 2024-12-27*
*출처: Web Search 기반 종합 정리*
