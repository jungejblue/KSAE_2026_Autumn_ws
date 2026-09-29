# 설정과 API 참고

[README](../README.md) · [설치·실행 설명서](usage.md)

[설정 파일](#configs) · [결과 JSON](#pairs) · [fallback API](#fallback) · [제어 유틸리티](#control) · [코드 역할](#files) · [인용](#citation)

<a id="configs"></a>

## 설정 파일과 명령 형식

| 파일 | 소비하는 기능 | 사용 범위 |
|---|---|---|
| `configs/tfpp.yaml` | `run_tfpp.py --config` | 외부 경로, route, 서버 주소, 반복·timeout |
| `configs/dual_scenario.json` | 듀얼·paired replay | 시나리오, route·fallback 경로, 고정 프로토콜 |
| `configs/dual_lead_stop.xml` | 듀얼·paired replay의 평가기 | Town01 선행 차량 정지 route |
| `configs/fallback.json` | `fallback_control.Config` | 제어기·backend 설정 |
| `configs/pairs.example.json` | `evaluate_pairs.py` | 오프라인 결과 집계 입력 예제 |

`python tools/evaluate_pairs.py configs/pairs.example.json`의 마지막 인자는 실행 설정이 아니라 **집계할 결과 데이터**입니다. `run_tfpp.py --config configs/tfpp.yaml`의 YAML은 실행 설정입니다.

`run_telemetry.py run`, `run_violations.py run`, `run_paired_replay.py run`의 `run`은 CLI의 하위 명령입니다. 특히 paired replay는 새 주행을 하는 `run`과 저장 결과를 읽는 `review`를 구분합니다. `run_tfpp.py`, `run_dual_scenario.py`에는 `run`을 붙이지 않습니다.

모든 옵션은 각 도구의 `--help`로 확인합니다. 하위 명령이 있으면 `run --help` 또는 `review --help`를 사용합니다.

### 평가 구간과 fault 구분

| 실행 | fault | 평가 구간 |
|---|---|---|
| `run_violations.py run --delay-ms …` | 명령 FIFO 지연 | 사용자가 지정한 지연 구간·route 기록 |
| `run_dual_scenario.py` | 마지막 제출 명령 3초 유지 | 후보 이후 5초 + 복귀·후속 주행 |
| `run_paired_replay.py run` | 같은 명령 유지 조건 | 후보 이후 3초의 독립 분기 비교 |
| `evaluate_pairs.py --horizon-seconds …` | 주행을 실행하지 않음 | 입력 JSON의 구간 일치 여부·결과 집계 |

`--horizon-seconds`는 오프라인 집계에만 적용됩니다. 듀얼·paired replay의 구간을 바꾸는 옵션이 아닙니다. paired 실행은 공유 JSON의 `horizon_ticks=100`을 60으로 적용하며, 실행·분석 코드도 3초를 전제로 합니다.

`review.json`·`comparison.json`은 `evaluate_pairs.py`의 입력 배열 형식과 다릅니다. 이 파일들을 그대로 넘기지 마세요. 직접 결과 배열을 구성한다면 비교 유효성, 구간 길이, 위반 여부와 개입 판단을 명시해야 합니다.


<a id="pairs"></a>

## 직접 수집한 결과 비교

입력은 다음 레코드들을 담은 JSON 배열입니다. `branch_e`는 주 제어기,
`branch_f`는 대체 제어기의 결과를 뜻합니다.

```json
[
  {
    "event_id": "pair_001",
    "valid_pair": true,
    "decision_intervene": true,
    "branch_e": {"duration_s": 3.0, "collision": true, "lane": false},
    "branch_f": {"duration_s": 3.0, "collision": false, "lane": false}
  }
]
```

| 필드 | 의미 |
|---|---|
| `event_id` | 중복되지 않는 문자열 ID |
| `valid_pair` | 두 실행이 비교 가능한 조건으로 수집됐는지 나타내는 boolean |
| `decision_intervene` | 해당 시점에 대체 제어기로 전환한다고 판단했으면 `true` |
| `duration_s` | 각 실행의 평가 구간 길이. 필수 입력이며 기본 명령에서는 3.0초 |
| `collision` | 평가 구간에 충돌이 발생했으면 `true` |
| `lane` | 평가 구간에 허용 주행 영역 이탈이 발생했으면 `true` |

boolean은 문자열 `"true"`가 아닌 JSON의 `true`/`false`로 입력합니다.
두 실행은 같은 시작 조건과 평가 구간을 사용해야 합니다. 이 도구는 저장된 값을 집계하며,
시뮬레이터를 실행하거나 원본 주행의 비교 가능성을 자동 검증하지 않습니다.

```bash
python tools/evaluate_pairs.py /path/to/pairs.json --output outputs/summary.json
```

평가 구간이 5초라면 모든 유효 레코드의 `duration_s`를 5.0으로 기록하고
`--horizon-seconds 5`를 추가합니다. 길이가 다른 결과는 한 번에 집계하지 않습니다.
불완전한 실행은 다음처럼 표시할 수 있습니다.

```json
{"event_id": "pair_002", "valid_pair": false, "invalid_reason": "incomplete_run"}
```

이 레코드에는 두 branch와 `decision_intervene`가 필요하지 않습니다.
제외된 레코드는 `invalid_pairs`와 `invalid_reasons`에 집계됩니다.

충돌 또는 영역 이탈 중 하나라도 발생하면 해당 branch의 위반으로 분류합니다.

| 주 제어기 위반 | 대체 제어기 위반 | 출력 분류 |
|---|---|---|
| 없음 | 없음 | `UNNECESSARY` |
| 있음 | 없음 | `NECESSARY_EFFECTIVE` |
| 있음 | 있음 | `NECESSARY_INEFFECTIVE` |
| 없음 | 있음 | `HARMFUL` |

`NECESSARY_EFFECTIVE`를 positive로 두고 `decision_intervene`와 비교하여 TP/TN/FP/FN을 계산합니다.
precision은 `TP / (TP + FP)`, recall은 `TP / (TP + FN)`이며, 분모가 0이면 `null`입니다.
전체 출력에는 분류별 개수(`outcomes`)와 레코드별 결과(`events`)도 포함됩니다.

<a id="fallback"></a>

## 규칙 기반 fallback 제어기

`fallback_control`은 로컬 경로 후보 생성, CBF·CLF 조건을 사용하는 MPC, CARLA 제어 명령
변환을 제공하는 Python 라이브러리입니다. 기본 `sampled` backend는 NumPy로 후보를 계산합니다.
CARLA adapter는 차량·보행자·신호등 등의 시뮬레이터 정답 상태를 사용합니다.
TF++와 연결한 고정 전환·복귀 예제는 [듀얼 시나리오 실행기](usage.md#dual)에서 제공합니다.
이 라이브러리 자체는 전환 시점을 결정하지 않습니다.

### 설치와 설정

저장소 루트의 Python 3.10 환경에서 실행합니다. CARLA를 사용하지 않는 제어 계산에도
NumPy가 필요합니다. 아래 명령은 NumPy 1.26.4를 함께 설치합니다.

```bash
python -m pip install -e '.[fallback]'
```

`configs/fallback.json`은 기본 sampled 제어 설정입니다. JSON에 없는 값은 `Config`의
기본값을 사용합니다. 차량·노면이 달라지면 설정의 적합성을 별도로 평가해야 합니다.

| 설정 | 기본 배포값과 의미 |
|---|---|
| `backend` | `sampled`. 제한된 후보 집합에서 제약과 비용을 평가 |
| `control_dt` | 0.05초. CARLA 제어 tick과 일치해야 함 |
| `dt`, `horizon` | 예측 간격 0.15초, horizon 20. 예측 시간 격자는 `Config.times` 확인 |
| `solver_budget_s` | 0.045초. solver의 처리 예산 |
| `enforce_tick_deadline` | `true`. 반환 뒤 시간 초과를 검출하면 비상 명령으로 전환 |
| `max_obstacles` | 6. 초과 시 장애물을 조용히 버리지 않고 비상 명령 반환 |
| `terminal_coast_speed` | 2.0m/s. 일반 저속 감속의 타력 모드 진입 기준 |
| `terminal_coast_decel` | 0.2m/s². 타력 거리 예측에 가정하는 감속도 |

선택적인 `ipopt` backend는 `mpc.py`에서 제공합니다. 이를 사용하려면
`python -m pip install -e '.[ipopt]'` 후 설정의 `backend`를 `ipopt`로 바꿉니다.
CasADi/Ipopt는 기본 sampled 실행에 필요하지 않습니다. [듀얼 시나리오](usage.md#dual)에서 사용하는
주행 범위는 sampled backend에 해당하며 Ipopt에 그대로 적용할 수 없습니다.

### CARLA 없이 제어 명령 계산

다음 예시는 CARLA 서버 없이 합성 직선 경로와 차량 형상으로 제어 명령을 한 번
계산합니다. 이 API로 사용자 실행기에 제어기를 연결할 수 있습니다.

```bash
python - <<'PY'
import json
from pathlib import Path

import numpy as np
from fallback_control import Config, EgoState, FallbackController, Geometry, Route

config = Config(**json.loads(Path("configs/fallback.json").read_text()))
xy = np.column_stack((np.linspace(0.0, 150.0, 301), np.zeros(301)))
route = Route(xy, np.full(301, 3.6))
geometry = Geometry(
    wheelbase=2.85, rear_axle_x=-1.4,
    front=3.8, rear=1.0, half_width=0.95, max_steer=0.6,
    wheels=((0.0, -0.8), (0.0, 0.8), (2.85, -0.8), (2.85, 0.8)),
)
controller = FallbackController(route, geometry, config)
command = controller.step(EgoState(x=5.0, y=0.2, yaw=0.0, speed=3.0), [], speed_limit=3.0)
print(command)
PY
```

코어의 좌표는 오른손 XY, yaw·steer는 rad, 길이는 m, 속도는 m/s입니다. ego 위치는
후륜축 중심입니다. `Geometry.wheels`는 후륜축 기준 고정 바퀴 중심 좌표이며, 예시 형상을
실제 MKZ 측정값으로 사용하지 마세요. 실제 CARLA에서는 adapter가 차량 형상을 구성합니다.
`Command`에는 가속도, 조향각, 목표 속도, `emergency`, 진단 정보가 담깁니다.
직접 연결하는 실행기는 `emergency`를 반드시 처리해야 합니다.

### CARLA 실행기에 연결

CARLA 0.9.15의 `vehicle.lincoln.mkz_2020`과 동기식 0.05초 간격을 사용합니다.
다음은 **이미 차량과 전체 경로를 준비한 실행기 안에 삽입하는 코드**입니다.

```python
import json
from pathlib import Path

from fallback_control import Config
from fallback_control.carla_adapter import CarlaFallback

config = Config(**json.loads(Path("configs/fallback.json").read_text()))
fallback = CarlaFallback(ego_vehicle, full_world_plan, config=config)

# 실행기의 각 제어 tick에서 현재 snapshot을 전달합니다.
control = fallback.run_step(world.get_snapshot())
ego_vehicle.apply_control(control)
# world.tick() 호출은 기존 실행기가 한 번만 수행합니다.
```

- `ego_vehicle`: 실행기가 생성한 CARLA Vehicle. autopilot 또는 다른 제어기가 동시에
  같은 차량에 명령을 제출하지 않도록 실행기에서 제어권을 관리합니다.
- `full_world_plan`: `(carla.Transform, command)` 목록. 최소 3개 지점이 필요합니다.
  차선 중심을 따르는 연속된 전체 경로를 사용하며 지점 간 간격은 3m 이하여야 합니다.
  Garage에서는 downsample되기 전 `set_global_plan`의 전체 world 경로를 보존합니다.
- `run_step`: 제어 명령을 반환합니다. adapter 자체는 `world.tick()` 또는
  `apply_control()`을 호출하지 않습니다. 같은 프레임을 재호출하면 이전 반환값을 사용합니다.
- 경로·world·ego가 바뀌면 새 `CarlaFallback`을 생성합니다. `FallbackController.reset()`은
  코어 상태 초기화이며 adapter 전체나 CARLA world 복원을 대신하지 않습니다.

일반 경로에 연결할 때는 실행기에서 서버·차량·route 수명 주기를 관리합니다.
[듀얼 시나리오 명령](usage.md#dual)은 포함된 Town01 경로에 대해 이 과정을 수행합니다.
기존 `run_tfpp.py`·`run_violations.py`는 자동으로 이 제어기를 사용하지 않습니다.
차선 변경 경로, 역주행, 급경사 등 지원하지 않는 조건은 adapter에서 거부합니다.
맵 구조물 중 actor로 제공되지 않는 물체는 자동으로 모두 관측되는 것이 아니므로 필요한
정적 장애물은 `extra_static_obstacles`에 같은 오른손 좌표계의 `Obstacle` 목록으로 전달합니다.

### 동작 범위와 해석

확인된 CARLA 단독 주행 범위는 Town01의 저속 직선·곡선·좌우 경로 복귀·정적 차량·횡단
보행자 조건입니다. 다른 맵·노면·속도의 동작까지 보장하지 않습니다. TF++와의 전환 예제는
[설명서의 고정 시나리오 범위](usage.md#dual)로 사용합니다.
이미 영역 밖에서 시작한 복귀 시나리오는 이탈 기록이 남아도 복귀 동작의 성공을 별도로
판정할 수 있습니다. 이는 충돌 OR 영역 이탈이라는 위반 정의를 바꾸는 것이 아닙니다.

시간 제한 처리는 연산이 반환된 뒤 초과 여부를 판단하는 방식입니다. 50ms 이내 실행을
강제로 선점·보장하는 기능이 아닙니다. CBF·CLF 조건도 후보 집합·예측 모델·관측 범위에
의존하며 실제 폐루프 안전이나 안정성을 무조건 증명하지 않습니다.
입력 정합성 검사는 유효하지 않은 상태를 검출하며, `diagnostics`는 명령 계산 시간과
비상 명령의 원인 등 실행 중 진단 정보를 제공합니다.

<a id="control"></a>

## Python에서 제어 유틸리티 사용

`ActionDelayFIFO`는 입력 명령을 지정한 호출 횟수만큼 늦춰 반환하는 기본 유틸리티입니다.
[실제 주행 지연 실행기](usage.md#delay)는 별도의 `action_delay.py`에 있는 `ActionDelay`를 사용합니다.

```python
from ksae_2026_autumn.control import ActionDelayFIFO

delay = ActionDelayFIFO(delay_ticks=2, initial_action="STOP")
print(delay.step("A"))  # STOP
print(delay.step("B"))  # STOP
print(delay.step("C"))  # A
```

실제 명령에는 `throttle`, `steer`, `brake`를 담은 Python 딕셔너리를 사용할 수 있습니다.
`reset()`은 초기 큐로 되돌리고, `snapshot()`/`restore()`는 큐 상태를 저장·복원합니다.
이 큐에는 지연을 적용하려는 명령만 전달합니다.

`baseline_decision()`은 충돌 위험 또는 이탈 위험이 참이면 `True`를 반환합니다.
`proposed_decision()`은 명령 지연, 위험, 대체 제어의 이득, 조건 지속 여부를 조합합니다.
두 함수는 외부에서 계산한 boolean을 입력받는 판단 함수입니다.

```python
from ksae_2026_autumn.control import proposed_decision

switch = proposed_decision(
    action_age_gate=True,
    predicted_e2e_risk=True,
    fallback_benefit_gate=True,
    persistence_gate=True,
)
print(switch)  # True
```

<a id="files"></a>

## 파일 구성

| 위치 | 역할 |
|---|---|
| `configs/` | 실행 설정, 결과 입력 예제, fallback 설정, 듀얼 시나리오 JSON·route XML |
| `src/fallback_control/` | fallback 제어 코어와 CARLA adapter. 아래 파일별 설명 참조 |
| `src/ksae_2026_autumn/control.py` | 명령 지연과 전환 판단 함수 |
| `src/ksae_2026_autumn/evaluation.py` | 위반 판정, 결과 분류, 지표 집계 |
| `src/ksae_2026_autumn/tfpp.py` | Garage 평가기 실행과 결과 경로 관리 |
| `src/ksae_2026_autumn/logging_agent.py` | TransFuser++에 로깅을 연결하는 agent |
| `src/ksae_2026_autumn/telemetry.py` | 프레임별 상태·명령 기록과 route 집계 |
| `src/ksae_2026_autumn/telemetry_runtime.py` | 평가기의 route 수명 주기와 제어 제출 지점 연결 |
| `src/ksae_2026_autumn/telemetry_cli.py` | 로깅 실행 옵션, 외부 경로, 실행 정보 관리 |
| `src/ksae_2026_autumn/route_corridor.py` | route 기반 허용 영역 |
| `src/ksae_2026_autumn/violation_geometry.py` | MKZ 바퀴 참조점 좌표 변환 |
| `src/ksae_2026_autumn/violations.py` | 충돌·이탈 결합 |
| `src/ksae_2026_autumn/violation_runtime.py` | 주행 중 검출 및 저장 |
| `src/ksae_2026_autumn/violation_launcher.py` | 위반 검출 주행 실행과 선택적 지연 옵션 |
| `src/ksae_2026_autumn/action_delay.py` | 구간별 명령 지연 FIFO와 버퍼 상태 저장·복원 |
| `src/ksae_2026_autumn/latency_runtime.py` | TF++ 반환 뒤 지연 채널 연결과 메타데이터 기록 |
| `src/ksae_2026_autumn/run_status.py` | 실행 종료 오류 전달 |
| `src/ksae_2026_autumn/safety_metrics.py` | TTC·TTLC·충돌 직전 속도 계산 |
| `src/ksae_2026_autumn/safety_metrics_io.py` | 기록 입력, 정합성 검사, CSV·JSON 출력 |
| `src/ksae_2026_autumn/dual_control.py` | 명령 유지·전환·복귀 조건과 prefix 상태 비교 |
| `src/ksae_2026_autumn/dual_runtime.py` | 선행 차량 시나리오와 TF++·fallback 제어 연결 |
| `src/ksae_2026_autumn/dual_cli.py` | 세 모드 순차 실행·설정·출력 관리 |
| `src/ksae_2026_autumn/dual_results.py` | 전환·위반·복귀·계산 시간 결과 비교 |
| `tools/run_dual_scenario.py` | 듀얼 시나리오 실행 명령 |
| `tools/compare_dual_runs.py` | 저장된 듀얼 시나리오 결과 집계 명령 |
| `tools/extract_safety_metrics.py` | 오프라인 안전 지표 추출 명령 |
| `tools/run_paired_replay.py` | 공통 후보 입력 기반 쌍대 실행·결과 재집계 |
| `src/ksae_2026_autumn/replay_checkpoint.py`, `replay_candidate_input.py` | 후보 상태 복원과 입력 공유 |
| `src/ksae_2026_autumn/paired_replay_runtime.py`, `paired_replay_cli.py`, `paired_replay_review.py` | 분기 주행·실행 관리·결과 비교 |
| `tools/` | 실행·결과 처리 명령 |

### fallback_control 파일별 역할

| 파일 | 역할 |
|---|---|
| `__init__.py`, `types.py` | 공개 API, 설정과 입출력 자료형 |
| `controller.py` | planner·MPC 연결과 명령·비상 상태 반환 |
| `planner.py`, `route.py` | 로컬 경로 후보·속도 계획, 경로 투영과 영역 계산 |
| `sampling_mpc.py` | 기본 sampled backend의 예측·제약·비용 평가 |
| `mpc.py` | 선택적 CasADi/Ipopt backend |
| `actuation.py` | 가속도 목표를 throttle/brake로 변환, 저속 타력 모드 |
| `steering.py` | 실제 바퀴 조향각과 정규화 제어량 변환 |
| `recovery.py` | 영역 밖 초기 상태의 복귀 허용 범위 계산 |
| `carla_adapter.py` | CARLA 좌표·관측·차량 형상·신호등과 제어 API 연결 |

<a id="citation"></a>

## 소프트웨어 인용

이 저장소의 소프트웨어를 사용했다면 [CITATION.cff](../CITATION.cff)를 참고합니다. 현재는 소프트웨어 저장소를 인용하며, 확인되지 않은 논문 제목·저자 순서·학회 게재·DOI를 지정하지 않습니다.

아래는 이 설명서의 대상 소스에 대한 BibTeX 예시입니다. 다른 commit으로 실행했다면 `note`의 commit을 실제 값으로 바꾸세요.

```bibtex
@misc{ksae_carla_tools,
  author = {{KSAE\_2026\_Autumn\_ws contributors}},
  title = {{KSAE\_2026\_Autumn\_ws}: CARLA E2E and fallback evaluation tools},
  year = {2026},
  howpublished = {\url{https://github.com/jungejblue/KSAE_2026_Autumn_ws}},
  note = {Source commit 547ba1ff1d8ea794d5a06d3af2ddf052b88cbfdc}
}
```

실행 조건을 전달할 때는 저장소 commit, Garage commit, 모델·config hash, CARLA 버전 및 실제 적용 설정을 함께 기록합니다. 결과 폴더에 저장된 실행 정보를 사용할 수 있습니다.

사용한 외부 구성요소도 해당 프로젝트의 인용 안내를 따릅니다.

- [CARLA 인용 안내](https://github.com/carla-simulator/carla/tree/0.9.15#citing-carla)
- [CARLA Garage·TransFuser++ 및 통합 Bench2Drive 인용 안내](https://github.com/autonomousvision/carla_garage/tree/72f39a63423a5edef6904b1487e0360a64bcf445#citations)
- [Bench2Drive 공식 저장소](https://github.com/Thinklab-SJTU/Bench2Drive)
