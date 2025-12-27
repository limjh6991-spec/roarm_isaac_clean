# RoArm-M3 Isaac Sim RL Training

RoArm-M3 로봇팔의 Pick and Place 작업을 위한 강화학습 환경

## 📊 현재 상태

**Isaac Sim**: 5.1.0 GA  
**RL Algorithm**: SAC (권장) / PPO  
**Phase**: Vision RL (VRL) 전환 중

### 최근 업데이트 (2025-12-27)
- ✅ Isaac Sim 5.1 API 마이그레이션 완료
- ✅ 프로젝트 구조 리팩토링 (configs, scene 분리)
- ✅ `SingleArticulation` API 적용
- 🔄 VRL 학습 준비 완료

## 🏗️ 프로젝트 구조

```
roarm_isaac_clean/
├── configs/                    # ✨ 설정 파일 (YAML)
│   ├── base.yaml              # 환경/로봇/리워드 설정
│   ├── vision_rl.yaml         # VRL 전용 설정
│   └── config_loader.py       # 설정 로더
├── envs/                       # RL 환경
│   ├── scene/                 # Scene 유틸리티
│   ├── robot/                 # 로봇 제어
│   ├── reward/                # 리워드 계산
│   ├── observation/           # 관측 생성
│   ├── roarm_pick_place_env.py     # 메인 환경
│   └── simple_vision_env_v2.py     # Vision RL 환경
├── scripts/                    # 학습/테스트 스크립트
│   ├── train/                 # 학습 스크립트
│   ├── test/                  # 테스트 스크립트
│   └── launch_vision_rl.sh    # VRL 실행
├── assets/roarm_m3/           # URDF, USD 모델
├── resources/                  # 문서
│   ├── PROJECT_STATUS.md      # 📌 AI 컨텍스트
│   └── isaac_sim_5_1_*.md     # API 가이드
└── logs/                       # 학습 로그
```

## 🚀 빠른 시작

### 1. Config 테스트
```bash
python configs/config_loader.py
```

### 2. 환경 테스트 (GUI)
```bash
python envs/simple_vision_env_v2.py
```

### 3. VRL 학습 실행
```bash
./scripts/launch_vision_rl.sh
```

## 🔧 기술 스택

| 항목 | 버전 |
|------|------|
| Isaac Sim | 5.1.0 GA |
| RL Library | Stable-Baselines3 |
| Algorithm | SAC (VRL) / PPO |
| Python | 3.11 |
| GPU | RTX 5090 (Blackwell) |

## 📝 주요 파일

| 파일 | 설명 |
|------|------|
| `envs/roarm_pick_place_env.py` | 메인 RL 환경 (1546줄) |
| `envs/simple_vision_env_v2.py` | Vision RL 환경 (457줄) |
| `configs/config_loader.py` | 설정 관리 시스템 |
| `resources/PROJECT_STATUS.md` | AI 컨텍스트 프롬프트 |

## 📚 문서

- [PROJECT_STATUS.md](resources/PROJECT_STATUS.md) - 현재 상태 및 다음 작업
- [Isaac Sim 5.1 API Guide](resources/isaac_sim_5_1_api_guide.md)
- [VRL Algorithm Recommendations](resources/vrl_algorithm_recommendations.md)
- [Isaac Sim 5.1 Cheatsheet](resources/isaac_sim_5_1_cheatsheet.md)

## 👥 기여자

- limjh6991-spec

---

**Last Updated**: 2025-12-27
