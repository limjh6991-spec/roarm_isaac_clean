# 프로젝트 구조 (Restructured)

## 📁 새로운 디렉토리 구조

```
roarm_isaac_clean/
│
├── 1_urdf_workspace/              # URDF 개발 및 테스트 워크스페이스
│   ├── assets/
│   │   └── roarm_m3/
│   │       ├── urdf/              # URDF 파일들
│   │       ├── meshes/            # 메시 파일들
│   │       └── usd/               # USD 변환 파일들
│   ├── scripts/
│   │   ├── verify_urdf.py         # URDF 검증
│   │   ├── convert_urdf_to_usd.py # URDF → USD 변환
│   │   └── test_joints.py         # 조인트 테스트
│   ├── tests/
│   │   └── urdf_tests/            # URDF 단위 테스트
│   ├── docs/
│   │   ├── URDF_DESIGN.md         # URDF 설계 문서
│   │   └── URDF_CHANGELOG.md      # URDF 변경 이력
│   └── README.md
│
├── 2_rl_workspace/                # 강화학습 워크스페이스
│   ├── envs/                      # RL 환경 정의
│   │   ├── __init__.py
│   │   ├── roarm_pick_place_env.py
│   │   └── configs/               # 환경 설정 파일들
│   ├── scripts/
│   │   ├── train/                 # 학습 스크립트
│   │   │   ├── train_ppo.py
│   │   │   └── train_config.yaml
│   │   ├── test/                  # 테스트 스크립트
│   │   │   ├── test_trained_model.py
│   │   │   └── visualize_policy.py
│   │   └── utils/                 # 유틸리티
│   │       ├── callbacks.py
│   │       └── logger.py
│   ├── logs/                      # 학습 로그
│   │   ├── checkpoints/
│   │   ├── tensorboard/
│   │   └── videos/
│   ├── tests/                     # RL 환경 테스트
│   │   └── env_tests/
│   ├── docs/
│   │   ├── TRAINING_LOG.md        # 학습 로그
│   │   ├── REWARD_DESIGN.md       # 보상 설계
│   │   └── HYPERPARAMETERS.md     # 하이퍼파라미터
│   └── README.md
│
├── docs/                          # 프로젝트 전체 문서
│   ├── PROJECT_STRUCTURE.md       # 이 파일
│   ├── CODING_RULES.md            # 코딩 규칙
│   ├── API_CHECKLIST.md           # API 버전 체크리스트
│   ├── DEVELOPMENT_WORKFLOW.md    # 개발 워크플로우
│   └── TROUBLESHOOTING.md         # 문제 해결 가이드
│
├── resources/                     # 참고 자료
│   ├── isaac_sim/                 # Isaac Sim 레퍼런스
│   ├── rl_references/             # RL 레퍼런스
│   └── roarm_m3/                  # RoArm M3 스펙
│
├── devops/                        # DevOps 스크립트
│   ├── setup_env.sh               # 환경 설정
│   └── backup.sh                  # 백업 스크립트
│
├── tests/                         # 통합 테스트
│   └── integration_tests/
│
├── .gitignore
├── README.md                      # 프로젝트 메인 문서
└── CHANGELOG.md                   # 전체 변경 이력
```

## 🎯 구조화 원칙

### 1. 관심사의 분리 (Separation of Concerns)
- **URDF 워크스페이스**: 로봇 모델링, 물리 파라미터, 메시 관리
- **RL 워크스페이스**: 환경 정의, 학습, 평가

### 2. 독립적 개발
- 각 워크스페이스는 독립적으로 개발 가능
- URDF 변경 → RL 환경 재테스트
- RL 환경 변경 → URDF는 영향 없음

### 3. 명확한 인터페이스
```python
# 2_rl_workspace/envs/roarm_pick_place_env.py
URDF_PATH = "../../1_urdf_workspace/assets/roarm_m3/urdf/roarm_m3_multiprim.urdf"
```

### 4. 버전 관리
- 각 워크스페이스는 독립적인 버전 관리
- URDF: v1.0, v1.1, v2.0 등
- RL 환경: Phase 0, Phase 1, Phase 2 등

## 📋 마이그레이션 계획

### Phase 1: 백업 (완료)
```bash
cp -r ~/roarm_isaac_clean ~/roarm_isaac_clean_backup_20251020
```

### Phase 2: URDF 워크스페이스 생성
```bash
mkdir -p 1_urdf_workspace/{assets/roarm_m3,scripts,tests,docs}
mv assets/roarm_m3/* 1_urdf_workspace/assets/roarm_m3/
```

### Phase 3: RL 워크스페이스 생성
```bash
mkdir -p 2_rl_workspace/{envs,scripts/{train,test,utils},logs,tests,docs}
mv envs/* 2_rl_workspace/envs/
mv logs/* 2_rl_workspace/logs/
```

### Phase 4: 문서 재구조화
```bash
# 각 워크스페이스별 README 및 문서 생성
# 전체 프로젝트 docs 폴더 정리
```

### Phase 5: 경로 수정
```bash
# 모든 스크립트의 import 경로 수정
# URDF 참조 경로 수정
```

## 🔄 개발 워크플로우

### URDF 개발
1. `1_urdf_workspace/`에서 작업
2. `scripts/verify_urdf.py`로 검증
3. Isaac Sim에서 임포트 테스트
4. 통과 시 버전 태깅 (v1.2 등)

### RL 개발
1. `2_rl_workspace/`에서 작업
2. 최신 URDF 버전 참조 확인
3. 환경 테스트 → 학습 → 평가
4. 체크포인트 저장 및 문서화

## 🎨 확장 가능성

### 미래 추가 가능 워크스페이스
```
3_hardware_interface/    # 실제 하드웨어 제어
4_deployment/            # 배포 및 모니터링
5_experiments/           # 실험 및 연구
```

## 📝 주의사항

### 절대 경로 vs 상대 경로
- **절대 경로**: 환경 설정, 로그 저장
- **상대 경로**: 워크스페이스 내부 참조

### 공유 리소스
- `resources/`: 모든 워크스페이스가 참조 가능
- `docs/`: 프로젝트 전체 문서

### 백업 정책
- 구조 변경 전 항상 백업
- `_backup_YYYYMMDD/` 형식으로 저장
