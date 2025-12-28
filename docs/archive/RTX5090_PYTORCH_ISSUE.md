# RTX 5090 PyTorch 호환성 문제 및 해결 방안

**작성일**: 2025-11-03  
**확인일**: 2025-11-11  
**상태**: 🔴 CRITICAL - GPU 학습 불가

---

## 🔴 문제 요약

### 핵심 이슈
```
NVIDIA GeForce RTX 5090 with CUDA capability sm_120 is not compatible 
with the current PyTorch installation.

Error: CUDA error: no kernel image is available for execution on the device
```

### 기술적 원인
- **GPU**: NVIDIA RTX 5090 (Blackwell 아키텍처, **sm_120**)
- **PyTorch**: 2.2.2+cu121 (지원: sm_50~sm_90, **sm_120 미포함**)
- **Isaac Sim**: 4.2.0 (PyTorch 2.2.2 고정 의존성)

### 영향 범위
| 컴포넌트 | 상태 | GPU 사용 | 비고 |
|---------|------|---------|------|
| Isaac Sim 4.2.0 | ✅ 정상 | ✅ 2949MB | PhysX는 CUDA 직접 사용 |
| PyTorch 연산 | ❌ 실패 | ❌ Kernel 없음 | CPU fallback 가능 |
| SAC 트레이닝 | ❌ 중단 | ❌ 환경 생성 실패 | stable-baselines3 |
| Vision 전처리 | ⚠️ 위험 | ❌ torch 사용 시 | NumPy 사용 권장 |

---

## 📊 검증 결과

### 1. PyTorch CUDA 테스트
```python
import torch
x = torch.randn(1000, 1000, device='cuda')
# ❌ RuntimeError: CUDA error: no kernel image is available for execution
```

**결과**: GPU 텐서 연산 완전 불가

### 2. Isaac Sim GPU 사용
```bash
nvidia-smi
# ✅ /isaac-sim/kit/kit: 2949MiB VRAM 사용 중
```

**결과**: PhysX 시뮬레이션은 정상 작동

### 3. SAC 트레이닝 시도
```
1. Creating environments...
[Warning] RTX 5090 sm_120 not compatible...
[8.055s] Simulation App Shutting Down
```

**결과**: 환경 생성 중 PyTorch 에러로 중단

---

## 🎯 해결 방안

### ✅ 방안 1: CPU 모드 (즉시 적용 가능)

**장점**:
- ✅ 즉시 작동
- ✅ Isaac Sim 호환성 유지
- ✅ 안정성 보장

**단점**:
- ❌ 학습 속도 10-50배 느림
- ❌ 대규모 학습 비현실적

**구현**:
```python
# scripts/train/train_vision_sac.py
model = SAC(
    "CnnPolicy",
    train_env,
    device="cpu",  # ← CPU 모드
    buffer_size=50_000,  # ← 메모리 절약
    batch_size=128,      # ← 배치 크기 감소
    verbose=1,
)
```

**예상 성능**:
- 50K timesteps: ~8-12시간 (GPU: ~1-2시간)
- Proof-of-concept 가능

---

### ⚠️ 방안 2: PyTorch 2.5+ 업그레이드 (실험적)

**PyTorch 2.5.0+ sm_120 지원**:
- ✅ RTX 5090 완전 지원
- ✅ GPU 가속 복원
- ⚠️ Isaac Sim 4.2.0 호환성 미검증

**위험 요소**:
```
Isaac Sim 4.2.0 요구사항:
- PyTorch == 2.2.2
- torchvision == 0.17.2
- 기타 CUDA 라이브러리 버전 고정

PyTorch 2.5.0 변경사항:
- CUDA 12.4+ 권장
- cuDNN 9.x 필요
- API 변경 가능성
```

**테스트 계획**:
1. **별도 Docker 컨테이너** 생성
2. PyTorch 2.5.1 설치
3. Isaac Sim 부팅 테스트
4. 실패 시 즉시 롤백

---

### 🔮 방안 3: Isaac Sim 업그레이드 대기 (장기)

**Isaac Sim 4.3/5.0 기대사항**:
- PyTorch 2.4+ 지원 가능성
- Blackwell (sm_120) 공식 지원
- 최적화된 의존성

**현실성**:
- 릴리스 시기 미정
- 6개월+ 소요 가능

---

## ✅ TODO 체크리스트

### 🔧 즉시 조치 (금일/내일)

- [ ] **CPU 모드 SAC 트레이닝 테스트**
  - [ ] `train_vision_sac.py` 수정 (`device="cpu"`)
  - [ ] 10K timesteps 테스트 실행 (~2시간)
  - [ ] 학습 곡선 확인 (TensorBoard)
  - [ ] 성능 측정 및 기록

- [ ] **우분투용 최신 NVIDIA 드라이버 검색**
  - [ ] RTX 5090 최적화 드라이버 확인
  - [ ] 현재: 580.95.05, CUDA 13.0
  - [ ] 최신: [NVIDIA 드라이버 페이지](https://www.nvidia.com/Download/index.aspx) 확인
  - [ ] 업데이트 필요 여부 판단

### 🧪 실험적 작업 (이번 주)

- [ ] **PyTorch 2.5+ 호환성 테스트**
  - [ ] 별도 Docker 컨테이너 생성
    ```bash
    docker run -it --gpus all nvidia/cuda:12.4.0-devel-ubuntu22.04
    ```
  - [ ] PyTorch 2.5.1 설치
    ```bash
    pip install torch==2.5.1 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124
    ```
  - [ ] sm_120 지원 확인
    ```python
    import torch
    print(torch.cuda.get_arch_list())  # sm_120 포함 여부
    x = torch.randn(1000, 1000, device='cuda')
    y = x @ x  # 연산 테스트
    ```
  - [ ] Isaac Sim 설치 및 부팅 테스트
  - [ ] 의존성 충돌 기록
  - [ ] 결과 문서화

### 📚 모니터링 (지속적)

- [ ] **Isaac Sim 릴리스 노트 모니터링**
  - [ ] [NVIDIA Isaac Sim 문서](https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html) 주간 확인
  - [ ] PyTorch 버전 요구사항 변경 추적
  - [ ] RTX 5090/Blackwell 관련 공지사항 확인

- [ ] **PyTorch 릴리스 추적**
  - [ ] [PyTorch GitHub Releases](https://github.com/pytorch/pytorch/releases) 확인
  - [ ] sm_120 지원 버전 체크
  - [ ] Isaac Sim 호환 가능 버전 파악

- [ ] **커뮤니티 피드백 수집**
  - [ ] NVIDIA Developer Forums 검색
  - [ ] Isaac Sim Discord/Slack 확인
  - [ ] 유사 사례 솔루션 조사

---

## 🔬 검증 명령어 모음

### PyTorch CUDA 지원 확인
```bash
docker exec zen_yonath /isaac-sim/python.sh -c "
import torch
print('PyTorch:', torch.__version__)
print('CUDA available:', torch.cuda.is_available())
print('Device capability:', torch.cuda.get_device_capability(0))
print('Supported archs:', torch.cuda.get_arch_list())

# 실제 연산 테스트
try:
    x = torch.randn(100, 100, device='cuda')
    y = x @ x
    print('✅ GPU computation: SUCCESS')
except Exception as e:
    print(f'❌ GPU computation: FAILED - {e}')
"
```

### GPU 사용 현황
```bash
docker exec zen_yonath nvidia-smi
```

### SAC 로그 확인
```bash
docker exec zen_yonath tail -f /workspace/roarm_project/logs/sac_training_*.log
```

---

## 📁 관련 파일

### 수정이 필요한 파일
```
scripts/train/train_vision_sac.py  # device="cpu" 설정
envs/simple_vision_env.py          # torch 사용 최소화
models/cnn_extractor.py            # CPU 호환 확인
```

### 로그 위치
```
logs/sac_training_*.log            # 트레이닝 로그
output/train_vision_sac/           # 모델 체크포인트
```

---

## 💡 권장 진행 순서

### Phase 1: 긴급 대응 (1-2일)
1. ✅ CPU 모드로 SAC 테스트
2. ✅ 학습 가능 여부 확인
3. ✅ 드라이버 최신 버전 확인

### Phase 2: 실험적 솔루션 (1주)
1. ⚠️ 별도 환경에서 PyTorch 2.5 테스트
2. ⚠️ Isaac Sim 호환성 검증
3. ⚠️ 성능 벤치마크

### Phase 3: 장기 모니터링 (지속)
1. 🔮 Isaac Sim 업데이트 추적
2. 🔮 NVIDIA 공식 솔루션 대기
3. 🔮 커뮤니티 피드백 수집

---

## 🚨 긴급 연락처

### NVIDIA 지원
- **Developer Forums**: https://forums.developer.nvidia.com/c/omniverse/simulation/69
- **Isaac Sim Issues**: https://github.com/NVIDIA-Omniverse/IsaacGymEnvs/issues
- **Technical Support**: https://developer.nvidia.com/nvidia-omniverse-developer-support

### PyTorch 커뮤니티
- **Forums**: https://discuss.pytorch.org/
- **GitHub Issues**: https://github.com/pytorch/pytorch/issues
- **Slack**: https://pytorch.slack.com/

---

## 📌 최종 상태

**날짜**: 2025-11-11  
**문제**: RTX 5090 (sm_120) PyTorch 2.2.2 비호환  
**상태**: 🔴 CRITICAL  
**다음 단계**: CPU 모드 테스트 → PyTorch 2.5 실험 → 공식 업데이트 대기  

**예상 해결 시기**:
- 단기 (CPU): ✅ 즉시
- 중기 (PyTorch 2.5): ⚠️ 1-2주 (실험 필요)
- 장기 (Isaac Sim 업그레이드): 🔮 6개월+

---

**작성자**: GitHub Copilot  
**최종 수정**: 2025-11-11
