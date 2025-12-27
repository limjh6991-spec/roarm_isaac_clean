# 🤖 RoArm Isaac Clean - AI 컨텍스트 프롬프트

> **마지막 업데이트**: 2025-12-27 14:30
> **이 파일을 읽고 현재 프로젝트 상태를 파악하세요.**

---

## 📍 프로젝트 개요

**프로젝트명**: RoArm-M3 Isaac Sim RL  
**목표**: RoArm-M3 로봇 암으로 Pick and Place 작업을 Vision RL로 학습  
**시뮬레이터**: NVIDIA Isaac Sim 5.1.0 GA  
**RL 프레임워크**: Stable-Baselines3 (SAC 알고리즘)  
**하드웨어**: RTX 5090 (Blackwell)

---

## ✅ 완료된 작업 (2025-12-27)

### 1. 프로젝트 구조 리팩토링
- **configs 시스템 구축**: `configs/base.yaml`, `configs/vision_rl.yaml`, `configs/config_loader.py`
- **Scene 유틸 분리**: `envs/scene/scene_builder.py` (DynamicCuboidWrapper)

### 2. Isaac Sim 5.1 API 호환성 업데이트
| 파일 | 변경 내용 |
|------|----------|
| `envs/roarm_pick_place_env.py` | `Articulation` → `SingleArticulation` |
| `envs/robot/robot_controller.py` | `ArticulationAction` import 경로 |
| `envs/roarm_pickplace_isaac_assets.py` | 5개 import 수정 |

### 3. 디스크 관리 정책 구현 ⭐ (신규)
| 항목 | 변경 내용 |
|------|----------|
| Replay buffer 저장 | `True` → `False` (각 22GB 방지) |
| 체크포인트 간격 | 10K → 50K 스텝 |
| 500K 학습 예상 용량 | ~1.1TB → **~660MB** |

---

## 📂 핵심 파일 구조

```
roarm_isaac_clean/
├── configs/                    # ✨ 설정 파일
│   ├── base.yaml              # 환경/로봇/리워드 설정
│   ├── vision_rl.yaml         # VRL 전용 설정
│   └── config_loader.py       # 설정 로더
├── envs/
│   ├── scene/                 # ✨ Scene 유틸
│   │   └── scene_builder.py   # DynamicCuboidWrapper
│   ├── robot/robot_controller.py
│   ├── reward/reward_calculator.py
│   ├── roarm_pick_place_env.py      # 메인 RL 환경 (1546줄)
│   └── simple_vision_env_v2.py      # Vision RL 환경 (457줄)
├── scripts/
│   ├── train/train_vision_sac.py    # VRL 학습 스크립트
│   └── launch_vision_rl.sh          # 학습 실행 스크립트
├── assets/roarm_m3/                 # URDF, USD 모델
└── resources/                       # 문서 및 가이드
    ├── PROJECT_STATUS.md            # 📌 이 파일
    ├── isaac_sim_5_1_api_guide.md   # API 마이그레이션 가이드
    └── vrl_algorithm_recommendations.md
```

---

## 🎯 다음 작업 (우선순위 순)

### 1. VRL 학습 재개 (최우선)
```bash
cd /home/roarm_m3/roarm_isaac_clean
./scripts/launch_vision_rl.sh
```
- `simple_vision_env_v2.py` 사용
- SAC 알고리즘 + NatureCNN extractor

### 2. 테스트 스크립트 API 업데이트 (선택)
- `scripts/test/` 폴더의 deprecated API 수정
- 약 15개 파일에 `omni.isaac.core` 잔존

### 3. 거대 파일 추가 분할 (선택)
- `roarm_pick_place_env.py`: `_calculate_reward` (272줄) 분리 가능
- 현재 기능 동작에는 문제 없음

---

## ⚠️ 알려진 이슈

1. **Headless 학습 초기화 지연**: 첫 reset() 시 10-20초 소요
2. **Joint 안정성**: 이미 해결됨 (Drive nullification + Gravity compensation)
3. **테스트 스크립트 deprecated API**: 핵심 기능에 영향 없음

---

## 🔧 빠른 명령어

```bash
# Config loader 테스트
python configs/config_loader.py

# Vision 환경 테스트 (GUI)
python envs/simple_vision_env_v2.py

# VRL 학습 시작
./scripts/launch_vision_rl.sh
```

---

## 📚 참고 문서

- [Isaac Sim 5.1 API Guide](file:///home/roarm_m3/roarm_isaac_clean/resources/isaac_sim_5_1_api_guide.md)
- [VRL Algorithm Recommendations](file:///home/roarm_m3/roarm_isaac_clean/resources/vrl_algorithm_recommendations.md)
- [Isaac Sim 5.1 Cheatsheet](file:///home/roarm_m3/roarm_isaac_clean/resources/isaac_sim_5_1_cheatsheet.md)
