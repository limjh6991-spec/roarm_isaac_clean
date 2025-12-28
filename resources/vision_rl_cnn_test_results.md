# Vision RL CNN 테스트 결과 - 2025-12-27

## 테스트 요약

| 테스트 | 해상도 | 카메라 | Red Pixels | CNN 오차 | 결과 |
|--------|--------|--------|------------|----------|------|
| v1 | 84x84 | 기본 | 0.0% | - | 큐브 안 보임 |
| v2 | 84x84 | 수정 (prim path) | 1.8% | 9.03cm | 평균 예측 |
| v3 | 224x224 | 기본 | ~2% | 9.10cm | 평균 예측 |
| **v4** | **224x224** | **줌** | **7.2%** | **9.24cm** | **평균 예측** |

## 핵심 발견

### 1. 카메라 문제 해결됨 ✅
- `clipping_range` + prim path string 사용으로 카메라 수정
- 줌 카메라로 red pixels 7.2% 달성

### 2. CNN이 학습 못함 ❌
- 7.2% red pixels로도 CNN이 평균값만 예측
- 모델 크기 증가 (912K → 19M 파라미터)로도 개선 없음

### 3. 문제 원인 추정

| 원인 | 설명 |
|------|------|
| **CNN 아키텍처** | NatureCNN은 Atari 게임용, 정밀 위치 예측에 부적합 |
| **Loss function** | MSE가 평균값 예측을 장려 |
| **피처 추출 실패** | 큐브의 2D 위치 → 3D 좌표 매핑 학습 어려움 |

---

## 다음 해결책 (우선순위)

### 1. Spatial Softmax 레이어 추가
- 2D 키포인트 추출 → 더 정확한 위치 학습
- 구현 난이도: 중간

### 2. Pretrained ResNet + Fine-tuning
- ImageNet pretrained encoder + small head
- 더 강력한 피처 추출

### 3. Auxiliary Loss 추가
- 큐브 heatmap 예측 보조 태스크
- L1 loss (MAE) 시도

### 4. 2D Detection → 3D Regression
- 먼저 이미지에서 큐브 2D 바운딩박스 예측
- 그 다음 3D 좌표로 변환

---

## 테스트 환경

- Isaac Sim 5.1
- PyTorch 2.7.0+cu128
- RTX 5090
- 10,000 샘플 데이터

---

*작성: 2025-12-27 19:10*
