# 🔍 Isaac Sim 시각화 문제 분석 보고서

**작성일**: 2025-10-30  
**문제**: Isaac Sim GUI 창이 열리지만 로봇이 움직이지 않고 빠르게 종료됨

---

## 📊 문제 증상

1. **Isaac Sim 창 열림**: GUI는 정상적으로 표시됨
2. **초기 화면 로드**: 환경이 로드되는 것처럼 보임
3. **갑자기 확대된 화면**: 마지막 프레임으로 보이는 화면이 확대되어 표시
4. **빠른 종료**: 로봇 동작을 관찰할 수 없이 종료
5. **로그는 정상**: 터미널 로그에는 600 스텝이 완료된 것으로 표시

---

## 🔬 원인 분석: IsaacLab 공식 코드와 비교

### ❌ **우리 코드의 문제점**

```python
# test_v4.1_visual.py (문제가 있는 코드)

# 1. env.step() 호출만 있음
while not done:
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)  # ❌ 렌더링 없음!
    
    # 진행 상황 출력
    if step_count % 50 == 0:
        print(f"Step {step_count}/600")
    
    # ❌ 렌더링 호출이 없음!
    time.sleep(0.1)  # 단순 대기만
```

**문제점**:
- `env.step()`: 물리 시뮬레이션만 실행
- **렌더링이 호출되지 않음**: GUI 업데이트 X
- `time.sleep()`: 프로그램만 대기, 화면 업데이트 X
- 결과: 600 스텝이 **백그라운드에서 빠르게 실행**되고, 마지막 프레임만 표시된 채로 종료

---

### ✅ **IsaacLab 공식 코드 (올바른 방법)**

#### 1️⃣ **play.py** (Stable Baselines3 시각화)
```python
# scripts/reinforcement_learning/sb3/play.py (IsaacLab)

# simulate environment
while simulation_app.is_running():
    start_time = time.time()
    
    with torch.inference_mode():
        actions, _ = agent.predict(obs, deterministic=True)
        obs, _, _, _ = env.step(actions)  # 물리 시뮬레이션
    
    # ✅ 실시간 렌더링 속도 제어 (중요!)
    sleep_time = dt - (time.time() - start_time)
    if args_cli.real_time and sleep_time > 0:
        time.sleep(sleep_time)
```

**핵심**:
- `env.step()` 내부에서 **자동 렌더링** 처리
- `dt` (환경 time-step): 실제 시간과 동기화
- `sleep_time`: **실제 시간 기준** 대기 (프레임당 시간 맞춤)

#### 2️⃣ **DirectRLEnv.step()** (환경 내부 렌더링)
```python
# source/isaaclab/isaaclab/envs/direct_rl_env.py

def step(self, action: torch.Tensor) -> VecEnvStepReturn:
    # ...
    
    # check if we need to do rendering within the physics loop
    is_rendering = self.sim.has_gui() or self.sim.has_rtx_sensors()
    
    # perform physics stepping
    for _ in range(self.cfg.decimation):
        self._sim_step_counter += 1
        
        # set actions into buffers
        self._apply_action()
        self.scene.write_data_to_sim()
        
        # ✅ 물리 시뮬레이션
        self.sim.step(render=False)  # 물리만 먼저
        
        # ✅ 렌더링 (render_interval마다)
        if self._sim_step_counter % self.cfg.sim.render_interval == 0 and is_rendering:
            self.sim.render()  # ← 여기서 화면 업데이트!
        
        # update buffers
        self.scene.update(dt=self.physics_dt)
```

**핵심**:
- `self.sim.step(render=False)`: 물리만 먼저 실행
- `self.sim.render()`: **명시적 렌더링 호출**
- `render_interval`: 렌더링 주기 (성능 조절)
- `is_rendering`: GUI 또는 센서가 있을 때만 렌더링

#### 3️⃣ **SimulationContext.step()** (시뮬레이션 단계)
```python
# source/isaaclab/isaaclab/sim/simulation_context.py

def step(self, render: bool = True):
    """Steps the simulation."""
    
    # check if we need to raise an exception
    if builtins.ISAACLAB_CALLBACK_EXCEPTION is not None:
        exception_to_raise = builtins.ISAACLAB_CALLBACK_EXCEPTION
        builtins.ISAACLAB_CALLBACK_EXCEPTION = None
        raise exception_to_raise
    
    # check if the simulation timeline is paused
    if not self.is_playing():
        # step the simulator (but not the physics) to have UI still active
        while not self.is_playing():
            self.render()  # ✅ pause 중에도 UI 유지
            if self.is_stopped():
                break
        self.app.update()  # ✅ UI 업데이트
    
    # ✅ 물리 시뮬레이션 실행
    super().step(render=render)
    
    # app.update() may be changing the cuda device
    if "cuda" in self.device:
        torch.cuda.set_device(self.device)
```

**핵심**:
- `self.render()`: pause 중에도 UI 유지
- `self.app.update()`: **앱 전체 업데이트** (GUI 반응성)
- `super().step(render=render)`: 부모 클래스의 step 호출

#### 4️⃣ **SimulationContext.render()** (렌더링 메서드)
```python
# source/isaaclab/isaaclab/sim/simulation_context.py

def render(self, mode: RenderMode | None = None):
    """Render the scene."""
    
    if mode is not None:
        self.set_render_mode(mode)
    
    # render based on the render mode
    if self.render_mode == self.RenderMode.NO_GUI_OR_RENDERING:
        pass  # headless, no rendering
    
    elif self.render_mode == self.RenderMode.NO_RENDERING:
        # throttle the rendering frequency to keep the UI responsive
        self._render_throttle_counter += 1
        if self._render_throttle_counter % self._render_throttle_period == 0:
            self._render_throttle_counter = 0
            
            # ✅ UI 업데이트 (물리 없이)
            self.set_setting("/app/player/playSimulations", False)
            self._app.update()  # ← GUI 반응성 유지!
            self.set_setting("/app/player/playSimulations", True)
    
    else:
        # ✅ 전체 렌더링 (viewport + GUI)
        super().render()
```

**핵심**:
- `self._app.update()`: **GUI 이벤트 처리** (마우스, 키보드 등)
- `render_throttle`: 렌더링 빈도 조절 (성능 vs 반응성)
- `super().render()`: 부모 클래스의 렌더링 (실제 화면 그리기)

---

## 🎯 **핵심 차이점 정리**

| 항목 | ❌ 우리 코드 | ✅ IsaacLab 공식 |
|------|-------------|------------------|
| **렌더링 호출** | 없음 | `self.sim.render()` 명시적 호출 |
| **GUI 업데이트** | 없음 | `self._app.update()` 호출 |
| **시간 동기화** | `time.sleep(0.1)` 고정 | `dt - elapsed_time` 계산 |
| **렌더링 위치** | 없음 | `env.step()` 내부 자동 처리 |
| **렌더링 주기** | 없음 | `render_interval` 설정 |
| **pause 처리** | 없음 | `is_playing()` 체크 |
| **실시간 제어** | 없음 | `real_time` 플래그 + sleep 계산 |

---

## 💡 **문제 해결 방안**

### 방법 1: **환경 설정에 render_interval 추가** ⭐ (권장)
```python
# envs/roarm_pick_place_env.py

@configclass
class RoArmPickPlaceEnvCfg(BaseEnvCfg):
    # ...
    sim: SimulationCfg = SimulationCfg(
        dt=1/60.0,
        render_interval=1,  # ✅ 매 스텝마다 렌더링
        # render_interval=2,  # 또는 2 스텝마다 (성능 향상)
    )
```

### 방법 2: **test 스크립트에서 수동 렌더링** (임시 해결)
```python
# scripts/rl/test_v4.1_visual.py

while not done:
    action, _states = model.predict(obs, deterministic=True)
    
    # ✅ 환경 스텝 (render=False)
    obs, reward, done, info = env.step(action)
    
    # ✅ 수동 렌더링 호출
    if hasattr(env.unwrapped.env, 'sim'):
        env.unwrapped.env.sim.render()
    
    # ✅ 실시간 속도 제어
    elapsed_time = time.time() - start_time
    sleep_time = dt - elapsed_time
    if sleep_time > 0:
        time.sleep(sleep_time)
```

### 방법 3: **환경 클래스에 render 메서드 추가** (올바른 해결)
```python
# envs/roarm_pick_place_env.py

class RoArmPickPlaceEnv:
    def __init__(self, cfg: RoArmPickPlaceEnvCfg):
        # ...
        self.render_interval = cfg.sim.render_interval
        self._render_counter = 0
    
    def step(self, action):
        # 기존 step 로직...
        
        # ✅ 렌더링 추가
        self._render_counter += 1
        if self._render_counter % self.render_interval == 0:
            if self.world and (self.sim.has_gui() or self.sim.has_rtx_sensors()):
                self.sim.render()
        
        return obs, reward, done, info
```

---

## 🔧 **즉시 적용 가능한 수정**

### 수정 1: `SimulationCfg`에 `render_interval` 추가
```python
# envs/roarm_pick_place_env.py

from omni.isaac.lab.sim import SimulationCfg

@configclass
class RoArmPickPlaceEnvCfg(BaseEnvCfg):
    sim: SimulationCfg = SimulationCfg(
        dt=1/60.0,
        render_interval=1,  # ← 추가!
    )
```

### 수정 2: `test_v4.1_visual.py` 실시간 렌더링
```python
# scripts/rl/test_v4.1_visual.py

dt = 1/60.0  # 환경 time-step (60 FPS)

while not done:
    start_time = time.time()
    
    # 모델 예측
    action, _states = model.predict(obs, deterministic=True)
    
    # 환경 스텝
    obs, reward, done, info = env.step(action)
    
    # ✅ 실시간 속도 제어
    elapsed_time = time.time() - start_time
    sleep_time = dt - elapsed_time
    if sleep_time > 0:
        time.sleep(sleep_time)
```

---

## 📝 **요약**

### **문제의 핵심**
- Isaac Sim은 **`env.step()`만으로는 화면이 업데이트되지 않음**
- **`sim.render()` 또는 `app.update()`를 명시적으로 호출**해야 GUI가 반응함
- IsaacLab은 이를 **환경 내부에서 자동 처리**하도록 설계됨

### **우리 코드의 문제**
- `env.step()`만 호출하고 렌더링은 없음
- `time.sleep(0.1)`: 단순 대기만 하고 GUI 업데이트 X
- 결과: 600 스텝이 **백그라운드에서 빠르게 완료**되고, 마지막 프레임만 표시

### **해결 방법**
1. **환경 설정**: `render_interval=1` 추가
2. **환경 클래스**: `step()` 내부에 `self.sim.render()` 추가
3. **테스트 스크립트**: 실시간 속도 제어 (`dt - elapsed_time`)

---

## 🎯 **다음 단계**

1. ✅ **즉시 적용**: `test_v4.1_visual.py`에 실시간 렌더링 추가
2. ✅ **환경 수정**: `RoArmPickPlaceEnv.step()`에 렌더링 로직 추가
3. ✅ **설정 추가**: `SimulationCfg(render_interval=1)` 설정
4. ✅ **재테스트**: 로봇 동작을 **실시간으로** 관찰

---

**참고 자료**:
- IsaacLab: `scripts/reinforcement_learning/sb3/play.py`
- IsaacLab: `source/isaaclab/isaaclab/envs/direct_rl_env.py`
- IsaacLab: `source/isaaclab/isaaclab/sim/simulation_context.py`
