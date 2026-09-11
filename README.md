# KSAE_2026_Autumn_ws

**CARLA에서 TransFuser++를 실행하고, E2E·fallback 전환 시나리오와 주행 기록의 안전 결과를 비교하는 Python 도구 모음입니다.**

| 기능 | 사용 방법 | 출력 |
|---|---|---|
| 두 제어 모드의 결과 비교 | `tools/evaluate_pairs.py`에 결과 JSON 입력 | 결과 분류, 혼동행렬, precision/recall |
| TransFuser++ 주행 실행 | `tools/run_tfpp.py`에 설정 YAML 입력 | CARLA Garage 평가 결과와 실행 정보 |
| 주행 시계열 기록 | `tools/run_telemetry.py run` | 센서 프레임, 차량 상태, 제어 명령 JSONL |
| 충돌·이탈 기록 | `tools/run_violations.py run` | 바퀴 참조점, 충돌 이벤트, 위반 결과 |
| E2E 제어 명령 지연 주입 | `tools/run_violations.py run --delay-ms …` | 지연된 명령 제출과 프레임별 지연 기록 |
| 오프라인 안전 지표 추출 | `tools/extract_safety_metrics.py --input … --output …` | TTC·TTLC·충돌 직전 속도 CSV |
| E2E → fallback → E2E 시나리오 | `tools/run_dual_scenario.py` | 세 모드 주행, 전환·복귀 기록과 비교 결과 |
| 저장된 시나리오 결과 비교 | `tools/compare_dual_runs.py` | 상태 일치, 위반, 복귀·계산 시간 집계 |
| 규칙 기반 fallback 제어 | `fallback_control` Python API | 가속도·조향각 또는 CARLA VehicleControl |
| 제어 명령 지연·전환 판단 | Python에서 `control.py` 함수 사용 | 지연된 명령 또는 전환 여부 |

결과 비교와 제어 유틸리티는 Python 3.10만으로 사용할 수 있습니다.
TransFuser++ 실행에는 CARLA, CARLA Garage, 모델 파일과 NVIDIA GPU가 추가로 필요합니다.

## 빠른 시작: 예제 결과 집계

Ubuntu 22.04 / Python 3.10 터미널에서 실행합니다.
ZIP으로 받은 경우 압축을 풀고 저장소 폴더로 이동한 뒤 가상환경 생성부터 실행하세요.

```bash
git clone https://github.com/jungejblue/KSAE_2026_Autumn_ws.git
cd KSAE_2026_Autumn_ws
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python tools/evaluate_pairs.py configs/pairs.example.json
```

명령이 완료되면 다음 값을 포함하는 JSON이 터미널에 출력됩니다.

```json
{
  "total_pairs": 5,
  "valid_pairs": 4,
  "invalid_pairs": 1,
  "confusion": {"TP": 1, "TN": 2, "FP": 1, "FN": 0},
  "precision": 0.5,
  "recall": 1.0
}
```

위 내용은 전체 출력 중 일부입니다. `pairs.example.json`은 사용법 확인용 예제 데이터입니다.
결과를 파일로 저장하려면 새 파일 경로를 지정합니다.

```bash
python tools/evaluate_pairs.py configs/pairs.example.json --output outputs/example_summary.json
```

이후 명령은 저장소 루트에서 가상환경을 활성화한 상태로 실행합니다.

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

## TransFuser++ 실행

### 실행 환경 준비

지원 대상은 CARLA 0.9.15와 CARLA Garage의 `leaderboard_2` 코드입니다.
실행기는 Garage의 `leaderboard_evaluator_local.py`를 호출합니다.

| 준비할 항목 | 설치·다운로드 안내 |
|---|---|
| CARLA 0.9.15 서버와 PythonAPI | [CARLA 설치 안내](https://carla.readthedocs.io/en/0.9.15/start_quickstart/) |
| NVIDIA GPU 드라이버 | [NVIDIA 드라이버](https://www.nvidia.com/en-us/drivers/) |
| Docker Engine | [Ubuntu 설치 안내](https://docs.docker.com/engine/install/ubuntu/) |
| NVIDIA Container Toolkit | [설치 및 Docker 설정](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) |
| TransFuser++ 모델 | [CARLA Garage 모델 다운로드](https://github.com/autonomousvision/carla_garage/tree/leaderboard_2#pre-trained-models) |

호스트 터미널에서 Garage 코드를 준비합니다. 아래 명령은 연결 대상 코드의 버전을 고정합니다.

```bash
git clone --branch leaderboard_2 https://github.com/autonomousvision/carla_garage.git "$HOME/carla_garage"
git -C "$HOME/carla_garage" checkout 72f39a63423a5edef6904b1487e0360a64bcf445
```

모델 폴더에는 같은 배포 모델의 **`config.json`과 사용할 `.pth` 한 개**를 넣습니다.
서버·맵·Garage 원본·모델은 이 저장소 밖에 보관하고, 실제 경로를 설정합니다.

```bash
export CARLA_ROOT="$HOME/CARLA_0.9.15"
export CARLA_GARAGE_ROOT="$HOME/carla_garage"
export TFPP_MODEL="$HOME/models/tfpp"
export EXPERIMENT_OUTPUT_ROOT="$HOME/carla_outputs"
export GARAGE_IMAGE=leaderboard-user:latest
```

### Docker 환경 열기

저장소 루트의 **호스트 터미널**에서 실행합니다.
`build`는 [Garage 공식 Dockerfile](https://github.com/autonomousvision/carla_garage/blob/leaderboard_2/tools/Dockerfile.master)로
로컬 이미지를 만듭니다. 같은 Garage checkout으로 만든 이미지가 있으면 `build`를 생략합니다.
현재 사용자로 `docker info`가 실행되는 환경에서 사용하세요.

```bash
bash tools/docker.sh build
bash tools/docker.sh run
```

스크립트는 호스트의 CARLA PythonAPI·Garage·모델을 컨테이너에 읽기 전용으로 연결합니다.
코드와 출력 경로는 쓰기 가능하게 연결하며, 컨테이너의 `/outputs`는 호스트의
`EXPERIMENT_OUTPUT_ROOT`에 해당합니다. CARLA 서버는 호스트에서 실행합니다.

이어서 **컨테이너 내부**에서 설치합니다. `.venv-docker`는 이미지에 설치된 패키지를
재사용하며, 빠른 시작에서 만든 호스트의 `.venv`와 별도로 사용합니다.

```bash
python -m venv --system-site-packages .venv-docker
source .venv-docker/bin/activate
python -m pip install --no-cache-dir -e .
python -c "import carla, torch; print('CUDA available:', torch.cuda.is_available())"
```

마지막 명령에서 import 오류가 없고 `CUDA available: True`가 출력되는지 확인합니다.
다음 진입부터는 `source .venv-docker/bin/activate`로 활성화합니다.
이미지의 Python·의존성 구성이 바뀌면 가상환경도 새로 만듭니다.

이미 [Garage의 설치 안내](https://github.com/autonomousvision/carla_garage/tree/leaderboard_2#setup)로
호스트 Python 환경을 구성했다면 Docker 대신 그 환경에서 `python -m pip install -e .` 후
아래 실행 명령을 사용할 수 있습니다.

### 서버 시작과 주행 실행

**호스트의 별도 터미널**에서 실제 CARLA 설치 경로로 이동하고 서버를 시작합니다.

```bash
cd "$HOME/CARLA_0.9.15"
./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000
```

**컨테이너 내부의 저장소 루트**에서 경로를 점검하고 실행합니다.

```bash
python tools/run_tfpp.py --config configs/tfpp.yaml --dry-run
python tools/run_tfpp.py --config configs/tfpp.yaml
```

`--dry-run`은 파일 경로와 명령을 확인하며 서버 접속이나 모델 추론은 수행하지 않습니다.

| `configs/tfpp.yaml` 설정 | 의미 |
|---|---|
| `garage_root`, `carla_root`, `model_dir` | 앞에서 설정한 환경변수로 외부 파일 연결 |
| `routes` | route XML. 상대 경로는 Garage 루트 기준 |
| `output_dir` | 출력 폴더. 상대 경로는 명령 실행 위치 기준 |
| `host`, `port` | CARLA 서버 주소와 RPC 포트 |
| `traffic_manager_port`, `traffic_manager_seed` | Traffic Manager 포트와 난수 seed |
| `repetitions`, `timeout`, `debug` | route 반복 횟수, 평가기에 전달하는 timeout(초), 디버그 수준 |

기본 route는 Garage의 `leaderboard/data/debug.xml`입니다. 실행 전 해당 XML에 지정된 맵이
CARLA 설치에 포함되어 있는지 확인하세요. Bench2Drive 실행은
[Garage의 별도 평가 안내](https://github.com/autonomousvision/carla_garage/tree/leaderboard_2#bench2drive)를 따릅니다.

기본 출력 폴더는 `${EXPERIMENT_OUTPUT_ROOT}/tfpp_run`입니다.
출력 폴더가 이미 존재하면 중단합니다. 다시 실행할 때는 새 경로를 지정합니다.

```bash
python tools/run_tfpp.py --config configs/tfpp.yaml --output-dir /outputs/tfpp_run_02
```

| 출력 파일 | 내용 |
|---|---|
| `result.json` | Garage가 기록하는 route 상태·점수·위반 결과 |
| `run.json` | 실행 명령, 설정, Garage commit, 모델·route hash, 프로세스 종료 코드 |

`result.json`이 생성됐는지만 확인하지 말고 그 안의 route 상태와 완주·위반 결과를 확인하세요.
컨테이너는 `exit`, CARLA 서버는 실행 터미널에서 `Ctrl+C`로 종료합니다. 연결한 결과 파일은 유지됩니다.

## 주행 시계열 기록

`run_tfpp.py`는 YAML 기반 기본 평가 실행기입니다. 프레임별 상태와 제어 명령까지
필요하면 `run_telemetry.py run`을 사용합니다. 두 명령을 동시에 실행하지 마세요.
텔레메트리 실행기는 명령행 옵션을 사용하며 `configs/tfpp.yaml`을 읽지 않습니다.
소스와 도구 경로를 함께 사용하므로 저장소를 유지하고 `pip install -e .`로 설치합니다.

### 기존 서버에 연결하여 기록

앞의 환경 준비와 서버 시작을 마친 다음, 컨테이너 내부에서 실행합니다.
호스트 Garage Python 환경에서도 같은 명령을 사용할 수 있습니다.

```bash
python tools/run_telemetry.py run \
  --evaluator local --logging on \
  --garage "$CARLA_GARAGE_ROOT" --carla "$CARLA_ROOT" \
  --model "$TFPP_MODEL" \
  --routes "$CARLA_GARAGE_ROOT/leaderboard/data/debug.xml" \
  --seed 100 --gpu 0 \
  --output "$EXPERIMENT_OUTPUT_ROOT/telemetry_local_001"
```

`local`은 XML의 모든 route를 한 번씩 실행합니다. 일부만 실행하려면 별도의 route XML을
준비합니다. `--routes-subset`은 `b2d` 모드에서만 사용합니다.
출력 폴더는 매번 새 경로여야 합니다.

### Bench2Drive route 기록

이 모드는 평가기가 CARLA 서버를 시작하므로 **기존 서버를 종료한 뒤** 실행합니다.
현재 `tools/docker.sh run`은 PythonAPI만 연결하므로 아래 명령은 전체 CARLA 설치와
Garage 의존성이 있는 **호스트 Python 3.10 환경**에서 실행합니다.
Garage의 `Bench2Drive` 코드와 XML에서 요구하는 맵도 준비되어 있어야 합니다.

```bash
python tools/run_telemetry.py run \
  --evaluator b2d --logging on \
  --garage "$CARLA_GARAGE_ROOT" --carla "$CARLA_ROOT" \
  --model "$TFPP_MODEL" \
  --routes "$CARLA_GARAGE_ROOT/Bench2Drive/leaderboard/data/bench2drive220.xml" \
  --routes-subset 24211 --seed 100 --gpu 0 \
  --output "$EXPERIMENT_OUTPUT_ROOT/telemetry_b2d_001"
```

`24211`은 선택 예시입니다. 사용할 XML에 있는 ID로 바꾸며 여러 개는 쉼표로 구분합니다.
`--garage`, `--carla`, `--model`을 생략하면 각각 `CARLA_GARAGE_ROOT`, `CARLA_ROOT`,
`TFPP_MODEL`을 사용합니다. `--host`, `--port`, `--tm-port`, `--timeout`도 지정할 수 있습니다.
전체 옵션은 `python tools/run_telemetry.py run --help`로 확인합니다.

### 저장 내용과 지원 범위

| 실행 폴더 내 경로 | 내용 |
|---|---|
| `run.json` | 명령, seed, 모델·코드 hash, 제어 환경변수, 실행 환경, 종료 상태 |
| `result.json` | 평가기의 route 점수와 위반 결과 |
| `run.log`, `exit_code.txt` | 평가기 출력과 프로세스 종료 코드 |
| `routes/<route_id>/route.json` | route 식별자, 센서 구성, 시뮬레이션 설정 |
| `routes/<route_id>/ticks.jsonl` | 프레임별 상태·센서 프레임·제어 명령·처리 시간 |
| `routes/<route_id>/summary.json` | 호출·행 개수, 시간 통계, 로거 종료 상태 |
| `source/` | 실행에 사용한 코드·route·모델 설정 사본과 Git 상태 |

`--logging off`는 프레임별 JSONL 기록을 끄고 호출 집계와 실행 정보를 남깁니다.
출력은 저장소 밖의 `EXPERIMENT_OUTPUT_ROOT`에 보관하세요. 모델 가중치는 복사하지 않습니다.

현재 로깅·검사는 **동기식 0.05초 간격, 지연 주입 없는 E2E 주행**을 대상으로 합니다.
Garage의 핵심 소스 hash가 지원 버전과 다르면 실행을 중단합니다.
로거는 원본 agent 호출과 제어 제출 지점에 연결되며 Garage 파일 자체를 수정하지 않습니다.
`agent_step_wall_ms`는 agent 콜백의 경과 시간으로, GPU 모델 연산만 분리한 시간이 아닙니다.
`submitted_control`은 제어 제출 함수 호출 후 기록한 명령이며 차량의 물리적 적용 완료를
확인하는 신호는 아닙니다. `control.py`의 지연·전환 함수는 이 실행기에 자동 연결되지 않습니다.
실행기는 `telemetry_cli.py`의 `CONTROL_ENV` 값을 적용하며 실제 값을 `run.json`에 남깁니다.

## 충돌·주행 영역 이탈 기록

`run_violations.py run`은 TF++ 주행에 텔레메트리와 위반 검출을 함께 연결합니다.
현재 지원 차량은 `vehicle.lincoln.mkz_2020`이며 동기식 0.05초 간격으로 실행합니다.
모델 폴더와 Garage 버전은 위의 TF++ 실행 조건과 같습니다.

### Local 평가

호스트에서 CARLA 서버를 먼저 시작한 뒤, 주행용 Python 환경의 저장소 루트에서 실행합니다.
기존 Docker 도구는 local 평가에 사용할 수 있습니다.

```bash
python tools/run_violations.py run \
  --evaluator local \
  --garage "$CARLA_GARAGE_ROOT" --carla "$CARLA_ROOT" \
  --model "$TFPP_MODEL" \
  --routes "$CARLA_GARAGE_ROOT/leaderboard/data/debug.xml" \
  --seed 100 --gpu 0 \
  --output "$EXPERIMENT_OUTPUT_ROOT/violations_local_001"
```

### Bench2Drive 평가

기존 CARLA 서버를 종료한 뒤 전체 CARLA 설치와 Garage 의존성이 있는 호스트
Python 3.10 환경에서 실행합니다. 현재 Docker 도구는 CARLA 서버 전체를 연결하지 않습니다.

```bash
python tools/run_violations.py run \
  --evaluator b2d \
  --garage "$CARLA_GARAGE_ROOT" --carla "$CARLA_ROOT" \
  --model "$TFPP_MODEL" \
  --routes "$CARLA_GARAGE_ROOT/Bench2Drive/leaderboard/data/bench2drive220.xml" \
  --routes-subset 24211 --seed 100 --gpu 0 \
  --output "$EXPERIMENT_OUTPUT_ROOT/violations_b2d_001"
```

route ID는 XML에 있는 값으로 선택합니다. 출력 폴더는 매번 새 경로여야 합니다.
위반 검출에는 차량 상태가 필요하므로 로깅은 항상 켜져 있습니다.
전체 옵션: `python tools/run_violations.py run --help`.

### 판정 기준과 출력

평가기의 route를 따라 차선 가장자리를 연결한 고정 영역을 구성하고, MKZ 2020의
차량 기준 바퀴 중심 참조점 4개를 차량 자세로 월드 좌표에 변환합니다.
하나라도 영역 밖에 있으면 이탈로 판정합니다. 경계 위의 점은 내부로 취급합니다.
이는 실제 타이어 접지면·타이어 외곽을 측정하는 방식이 아닙니다.
교차로·차선 변경·지도 샘플링에 따라 영역 표현에 한계가 있으므로 모든 도로 형상에서
정확한 판정을 보장하지 않습니다. 다른 차량에는 새로운 바퀴 프로필이 필요합니다.

충돌은 collision sensor의 원본 이벤트 프레임에 결합하며, 충돌 또는 이탈 중 하나가
참이면 위반입니다. 센서 관측 전 구간이나 유효하지 않은 관측은 안전으로 간주하지 않습니다.
이벤트 수신 종료 시 대기 시간은 0.25초이며, 모든 콜백 전달을 증명하는 신호는 아닙니다.

기본 텔레메트리 파일에 더해 `routes/<route_id>/`에 다음을 저장합니다.

| 파일 | 내용 |
|---|---|
| `detector_config.json` | 차량 프로필, 좌표·판정 기준, 센서 관측 범위 |
| `corridor.json` | route로 만든 허용 영역 |
| `wheel_observations.jsonl` | 프레임별 차량 상태·바퀴 참조점·이탈 여부 |
| `collision_events.jsonl` | 원본 충돌 이벤트 |
| `violations.jsonl` | 충돌과 이탈을 결합한 프레임별 결과 |
| `violation_summary.json` | 종료 상태·오류·충돌 개수·위반 누적 여부 |

`violation_at_frame`은 해당 프레임, `violation_seen`은 시작 이후 누적 위반 여부입니다.
`true`는 위반, `false`는 관측된 범위에서 위반 없음, `null`은 판정 불가입니다.
`coverage_valid`도 함께 확인하세요. 실행 성공은 무충돌·무이탈을 의미하지 않습니다.

실행기는 요청 route의 완료와 로거·검출기의 종료 오류를 확인하고 실패 시 0이 아닌 코드를
반환합니다. 원본 평가기 종료 코드는
`exit_code.txt`에 남으며, 종료 상태 처리 오류는 `run.json`의 `completion_error`에 남습니다.
로그 전체의 의미적 정합성을 자동 보증하는 기능은 아닙니다.

위반 로그는 `evaluate_pairs.py` 입력과 다릅니다. 동일 조건에서 비교할 두 실행의 평가
구간을 선택하고 충돌·이탈을 집계하여 앞의 `pairs.json` 형식으로 준비해야 합니다.
이 위반 기록 명령은 E2E 주행만 수행합니다. 제어 전환과 복귀를 비교하려면
아래의 `run_dual_scenario.py` 시나리오 실행 명령을 사용하세요.

## E2E 제어 명령 지연 주입

`run_violations.py run`에 지연 옵션을 추가하면 TF++가 반환한 제어 명령을 FIFO에
저장하고, 준비된 명령을 평가기에 전달합니다. 센서 입력과 TF++ 호출은 계속 진행하며,
충돌·주행 영역 이탈 기록도 함께 저장합니다. 별도의 실행 스크립트는 필요하지 않습니다.

이는 **시뮬레이션 프레임 기준의 추가 제어 명령 지연**입니다. GPU 추론 시간을 늘리거나
프로세스를 `sleep`시키는 기능이 아닙니다. 안전 제어기로 전환하는 기능도 포함하지 않습니다.

### 지연을 지정하여 실행

앞의 위반 기록 환경을 준비한 뒤 실행합니다. Local 모드는 CARLA 서버를 먼저 켭니다.
아래 예시는 각 route에서 첫 기록 프레임으로부터 5초 후, 2초 동안 생성한 명령에
200ms 지연을 지정합니다. 출력 폴더는 매번 새 경로를 사용하세요.

```bash
python tools/run_violations.py run \
  --evaluator local \
  --garage "$CARLA_GARAGE_ROOT" --carla "$CARLA_ROOT" \
  --model "$TFPP_MODEL" \
  --routes "$CARLA_GARAGE_ROOT/leaderboard/data/debug.xml" \
  --seed 100 --gpu 0 \
  --delay-ms 200 --delay-onset-s 5 --delay-duration-s 2 \
  --output "$EXPERIMENT_OUTPUT_ROOT/violations_delay_200ms_001"
```

Bench2Drive에서는 기존 CARLA 서버를 종료하고, 전체 CARLA 설치와 Garage 의존성이
있는 호스트 Python 환경에서 실행합니다. 현재 Docker 도구는 local 실행용입니다.

```bash
python tools/run_violations.py run \
  --evaluator b2d \
  --garage "$CARLA_GARAGE_ROOT" --carla "$CARLA_ROOT" \
  --model "$TFPP_MODEL" \
  --routes "$CARLA_GARAGE_ROOT/Bench2Drive/leaderboard/data/bench2drive220.xml" \
  --routes-subset 24211 --seed 100 --gpu 0 \
  --delay-ms 200 --delay-onset-s 5 --delay-duration-s 2 \
  --output "$EXPERIMENT_OUTPUT_ROOT/violations_b2d_delay_200ms_001"
```

route ID는 사용할 XML에 있는 값으로 바꿉니다.

| 옵션 | 기본값과 의미 |
|---|---|
| `--delay-ms` | 생략하면 지연 채널을 연결하지 않음. 지정 가능한 값은 `0`, `100`, `200`, `500`ms |
| `--delay-onset-s` | `5.0`. 각 route의 첫 기록 프레임을 기준으로 지연 지정 시작 시각 |
| `--delay-duration-s` | `2.0`. 지연을 지정할 명령의 생성 구간 길이 |

동기식 0.05초 간격에서 100/200/500ms는 2/4/10tick입니다. 시작 시각은 0 이상,
길이는 0 초과여야 하며 둘 다 0.05초의 배수로 지정합니다. 시작·길이 옵션은
`--delay-ms`를 지정한 경우에만 사용됩니다. 시뮬레이션 시간 기준이며 실제 경과 시간이 아닙니다.

`--delay-ms 0`은 지연 채널을 통과하면서 현재 명령을 그대로 전달하고 지연 정보를
기록합니다. 옵션 생략과 구분하여 비교 조건에 사용하세요. 기본 설정에서는 route tick
100부터 139까지 생성한 40개 명령이 지정 구간에 해당합니다. route가 먼저 끝나면
구간이 실행되지 않거나 일부만 기록될 수 있습니다.

### FIFO 동작과 저장 필드

명령의 준비 프레임은 `생성 프레임 + 지정한 지연 tick`입니다. 아직 준비되지 않은
선두 명령을 뒤의 명령이 추월하지 않습니다. 새로 전달할 명령이 없으면 이전 전달 명령을
유지하며, 첫 호출부터 지연이 시작되어 이전 명령도 없으면 초기 제동 명령을 사용합니다.
동시에 여러 명령이 준비되면 그중 가장 최근 명령을 전달하고 나머지를 기록합니다.
route가 바뀌면 버퍼와 시작 프레임도 새로 초기화됩니다.

지정 구간이 끝나도 남은 버퍼 때문에 최대 `지연 tick - 1`프레임 동안 과거 명령이
전달될 수 있습니다. 따라서 새 명령에 지정한 지연과 실제 전달된 명령의 나이를 구분합니다.

설정은 `run.json`과 `routes/<route_id>/route.json`의 `action_delay`에 저장됩니다.
다음 필드는 `ticks.jsonl`에 저장됩니다. 기존 차량 상태와 별도의 충돌·이탈 기록을
프레임 기준으로 함께 해석합니다.

| 필드 | 의미 |
|---|---|
| `e2e_control` | 현재 TF++ 호출이 새로 생성한 원본 명령 |
| `selected_control`, `submitted_control` | 평가기가 선택하고 제출 함수에 전달한 명령 |
| `action_delay.route_tick` | 첫 기록 프레임을 0으로 둔 route 내 tick |
| `action_delay.episode_active` | 현재 생성 명령이 지정 구간에 포함되는지 여부 |
| `action_delay.requested_delay_ticks` | 현재 생성 명령에 지정한 지연 |
| `injected_delay_ticks` | 실제 선택 명령이 생성된 후 지난 tick. 초기 명령 부재 시 `null` |
| `action_delay.selected_generation_frame` | 선택 명령의 생성 프레임 |
| `action_delay.selected_source_frame` | 선택 명령이 참조한 센서 기준 프레임 |
| `current_e2e_action_source_frame` | 현재 새로 생성한 E2E 명령의 센서 기준 프레임 |
| `action_source_frame`, `action_observation_age_s` | 선택 명령의 센서 기준 프레임과 관측 나이 |
| `action_delay.held`, `action_delay.initial_hold` | 이전 명령 유지 여부와 초기 명령 부재 여부 |
| `action_delay.queue_depth` | 남아 있는 버퍼 명령 개수 |
| `action_delay.released_frames`, `action_delay.superseded_frames` | 준비되어 꺼낸 명령과 그중 제출을 생략한 명령의 생성 프레임 |
| `delay_wall_ms` | 버퍼 처리·VehicleControl 구성 등에 걸린 시간. TF++ 추론 시간 제외 |

`action_observation_age_s`에는 센서의 기존 프레임 차이도 포함될 수 있어 주입한 명령
지연과 같다고 가정하면 안 됩니다. 제출 기록은 함수 호출 근거이며 물리 서버 처리 완료
응답을 뜻하지 않습니다. `ActionDelay.snapshot()`과 `restore()`는 지연 버퍼 상태만
저장·복원하며 CARLA world, 다른 차량, 모델 내부 상태까지 복원하지 않습니다.

### 지연 주행의 안전 지표 추출

앞에서 생성한 실행 폴더를 그대로 입력합니다. 실제 주행이 끝난 뒤 실행하세요.

```bash
python tools/extract_safety_metrics.py \
  --input "$EXPERIMENT_OUTPUT_ROOT/violations_delay_200ms_001" \
  --output "$EXPERIMENT_OUTPUT_ROOT/metrics_delay_200ms_001"
```

Bench2Drive 결과에는 해당 실행 폴더를 지정합니다. 아래 안전 지표 설명에서 입력 조건과
계산 가정을 확인하세요. 지연 설정·출력 디렉터리 이외의 모델, route, seed 등 비교 조건은
동일하게 맞춥니다. 같은 seed만으로 서로 다른 실행의 궤적이 완전히 같아지는 것은 아닙니다.

## 기록된 주행에서 안전 지표 추출

`extract_safety_metrics.py`는 `run_violations.py run`으로 저장한 주행 기록에서
TTC, TTLC, 충돌 직전 속도 대푯값을 계산합니다. 이 과정은 Python 3.10과 저장된
입력 파일만 사용하며, CARLA 서버·Garage·GPU를 실행할 필요가 없습니다.
일반 `run_tfpp.py` 또는 `run_telemetry.py` 출력만으로는 필요한 위반 기록이 부족합니다.

### 입력 폴더 선택

앞의 위반 검출 실행에서 지정한 출력 폴더를 `--input`으로 전달합니다.
해당 폴더 바로 아래에 `routes/<route_id>/`가 있어야 하며, 각 route에는 다음 파일이 필요합니다.

- `ticks.jsonl`, `violations.jsonl`, `collision_events.jsonl`
- `corridor.json`, `detector_config.json`, `violation_summary.json`

입력은 스키마 버전 2, 지원되는 MKZ 2020 고정 바퀴 프로필, 연속된 0.05초 프레임,
유효한 관측 범위, 오류 없이 종료된 검출 세션이어야 합니다. 입력이 이 조건을
만족하지 않으면 원인을 출력하고 중단합니다.

### 추출 실행

저장소 루트에서 Python 환경을 활성화한 뒤 실행합니다. 아래 입력 경로를 실제 위반
기록 폴더로 바꾸세요. Docker에서 생성한 기록도 호스트의 출력 경로로 읽을 수 있습니다.

```bash
python tools/extract_safety_metrics.py \
  --input "$HOME/carla_outputs/violations_local_001" \
  --output "$HOME/carla_outputs/safety_metrics_001"
```

| 옵션 | 의미 |
|---|---|
| `--input` | `routes/`가 바로 들어 있는 실행 폴더. 필수 |
| `--output` | 아직 존재하지 않는 결과 폴더. 입력 폴더 내부는 사용할 수 없음. 필수 |
| `--help` | 사용법 출력 |

이 명령에는 `run` 하위 명령이 없습니다. 상위 실험 폴더나 개별 route 폴더를 입력하지
마세요. 특정 이름의 하위 폴더를 자동으로 선택하지 않으며, 입력의 `routes/*/ticks.jsonl`로
발견되는 모든 route를 처리합니다. 다시 추출할 때는 새로운 출력 경로를 사용합니다.

### 출력과 해석

| 출력 폴더 내 경로 | 내용 |
|---|---|
| `analysis.json` | 처리 상태, 입력·코드 hash, 예측 범위와 계산 가정 |
| `summary.json` | route별 요약 목록 |
| `routes/<route_id>/metrics.csv` | 프레임별 TTC·TTLC, 상태, 충돌·이탈 여부 |
| `routes/<route_id>/collision_speeds.csv` | 충돌 이벤트별 직전 프레임 속도와 접촉 episode |
| `routes/<route_id>/summary.json` | 최솟값, 상태별 개수, 충돌 개수·요약 |
| `inputs/<route_id>/` | 계산에 사용한 입력 6개 파일의 사본 |

- **TTC**: 기록된 차량·보행자가 현재 전역 속도와 자세를 유지할 때 bounding box의
  XY 투영과 수직 범위가 겹치기까지의 시간입니다. 정적 지도 물체는 대상에서 제외됩니다.
- **TTLC**: 현재 전역 속도·자세로 이동할 때 고정 바퀴 참조점 중 하나가 허용 영역을
  처음 벗어나는 샘플 시점입니다. 이미 밖에 있으면 0초입니다.
- **충돌 직전 속도**: 충돌 이벤트 직전 프레임의 ego 속도입니다. 0.05초 전의 대푯값이며
  실제 접촉 순간 속도가 아닙니다. 직전 프레임이 없으면 계산 불가로 표시합니다.

CLI의 예측 범위는 3.0초, TTLC 샘플 간격은 0.05초로 고정되어 있습니다.
현재 제어 명령이나 이후 제어 변화는 예측에 반영하지 않습니다.
JSON의 `null`과 CSV의 빈 값은 0이 아닙니다. 반드시 상태 필드와 함께 해석하세요.

| 상태 값 | 의미 |
|---|---|
| `no_overlap_within_horizon` | 예측 범위 내 겹침 없음 |
| `unknown_actor_state` | 누락된 actor 상태로 TTC를 결정할 수 없음 |
| `no_exit_on_prediction_grid` | 예측 샘플에서 이탈 없음 |
| `route_endpoint_censored` | route 끝을 넘어 TTLC 판단이 제한됨 |
| `missing_previous_frame` | 충돌 직전 속도를 계산할 프레임 없음 |

`analysis.json`의 `PASS`는 추출 완료 및 코드가 정의한 결측 문제 없음이라는 뜻입니다.
무충돌·무이탈이나 예측 정확도를 보증하지 않습니다. `REVIEW`는 TTC actor 상태 또는
충돌 직전 속도에 결측 문제가 있다는 뜻이며 route 요약의 `issues`를 확인합니다.
`PASS`는 종료 코드 0, `REVIEW`와 처리 오류는 0이 아닌 종료 코드를 반환합니다.
출력 생성 후 계산 중 오류가 나면 `analysis.json`에 `FAIL`과 오류를 기록합니다.
입력·출력 사전 검사에서 중단되면 이 파일이 생성되지 않을 수 있습니다.

전체 입력 사본과 결과는 저장소 밖에 보관합니다. 이 CSV는 `evaluate_pairs.py`의
입력 JSON과 다르며 자동으로 제어 전환이나 쌍대 재실행을 수행하지 않습니다.

## 규칙 기반 fallback 제어기

`fallback_control`은 로컬 경로 후보 생성, CBF·CLF 조건을 사용하는 MPC, CARLA 제어 명령
변환을 제공하는 Python 라이브러리입니다. 기본 `sampled` backend는 NumPy로 후보를 계산합니다.
CARLA adapter는 차량·보행자·신호등 등의 시뮬레이터 정답 상태를 사용합니다.
TF++와 연결한 고정 전환·복귀 예제는 아래의 `run_dual_scenario.py`에서 제공합니다.
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
CasADi/Ipopt는 기본 sampled 실행에 필요하지 않습니다. 아래에서 설명하는 확인된
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
아래의 듀얼 시나리오 명령은 포함된 Town01 경로에 대해 이 과정을 수행합니다.
기존 `run_tfpp.py`·`run_violations.py`는 자동으로 이 제어기를 사용하지 않습니다.
차선 변경 경로, 역주행, 급경사 등 지원하지 않는 조건은 adapter에서 거부합니다.
맵 구조물 중 actor로 제공되지 않는 물체는 자동으로 모두 관측되는 것이 아니므로 필요한
정적 장애물은 `extra_static_obstacles`에 같은 오른손 좌표계의 `Obstacle` 목록으로 전달합니다.

### 동작 범위와 해석

확인된 CARLA 단독 주행 범위는 Town01의 저속 직선·곡선·좌우 경로 복귀·정적 차량·횡단
보행자 조건입니다. 다른 맵·노면·속도의 동작까지 보장하지 않습니다. TF++와의 전환 예제는
아래에 설명한 고정 시나리오 범위로 사용합니다.
이미 영역 밖에서 시작한 복귀 시나리오는 이탈 기록이 남아도 복귀 동작의 성공을 별도로
판정할 수 있습니다. 이는 충돌 OR 영역 이탈이라는 위반 정의를 바꾸는 것이 아닙니다.

시간 제한 처리는 연산이 반환된 뒤 초과 여부를 판단하는 방식입니다. 50ms 이내 실행을
강제로 선점·보장하는 기능이 아닙니다. CBF·CLF 조건도 후보 집합·예측 모델·관측 범위에
의존하며 실제 폐루프 안전이나 안정성을 무조건 증명하지 않습니다.
입력 정합성 검사는 유효하지 않은 상태를 검출하며, `diagnostics`는 명령 계산 시간과
비상 명령의 원인 등 실행 중 진단 정보를 제공합니다.

## 시나리오로 E2E → fallback → E2E 복귀 비교

`run_dual_scenario.py`는 Town01의 정지 선행 차량 시나리오를 세 가지 모드로 실행합니다.
차량 생성, TF++ 호출, 제어권 전환, E2E 복귀와 결과 저장을 함께 수행하므로 별도 실행기
코드를 작성할 필요가 없습니다. 주행 종료 후 `comparison.json`과 `metrics.csv`를 비교합니다.

| 모드 | E2E 명령 채널 | 실제 ego 제어 |
|---|---|---|
| `clean` | 현재 명령 사용 | E2E 유지 |
| `delayed` | fault 시작 직전 실제 명령을 3초간 유지 | E2E 유지 |
| `takeover` | delayed와 같은 명령 유지 조건 | fault 시작 0.75초 후 fallback, 복귀 조건 충족 후 E2E |

여기서 지연은 **마지막 명령 유지로 구현한 연산 정지 모사**입니다. TF++는 계속 계산하지만
fault 구간의 새 출력을 명령 채널에서 버립니다. `run_violations.py --delay-ms`의 FIFO와는
다른 조건이며, 실제 GPU 추론 프로세스를 정지시키는 기능은 아닙니다.
전환 시점은 시나리오에 고정된 시험 입력이며 위험 기반 자동 전환 기준은 아닙니다.

### 1. 실행 환경과 경로 준비

앞의 TransFuser++ 환경 준비를 마친 **호스트 Python 3.10 환경**에서 실행합니다.
CARLA Garage commit `72f39a63423a5edef6904b1487e0360a64bcf445`, 전체 CARLA 0.9.15 설치,
Town01 맵, Lincoln MKZ 2020, NVIDIA GPU가 필요합니다. 모델 폴더에는 서로 대응하는
`config.json`과 사용할 `.pth` 파일 하나를 준비합니다. 모델이 달라지면 fault가 발생하지
않거나 비교 결과가 달라질 수 있습니다.

```bash
cd "$HOME/KSAE_2026_Autumn_ws"

# CARLA Garage 의존성이 설치된 Python 환경을 활성화합니다.
conda activate garage_2
python -m pip install -e '.[fallback]'

export CARLA_GARAGE_ROOT="$HOME/carla_garage"
export CARLA_ROOT="$HOME/e2e_carla_ws"
export TFPP_MODEL="$HOME/models/tfpp"
export EXPERIMENT_OUTPUT_ROOT="$HOME/carla_outputs"
```

`garage_2`는 환경 이름 예시입니다. 실제 설치 경로와 환경 이름에 맞게 바꾸세요.
같은 모델 조건으로 비교하려면 다음 기준 SHA-256과 준비한 파일을 대조할 수 있습니다.

| 기준 파일 | SHA-256 |
|---|---|
| TF++ 가중치 한 개 | `d6fbdc28f7398354beadc7cf6765d866457c957f7b470c88ba206e73311a3b44` |
| 대응하는 `config.json` | `895e3e9704ceda443169ca32aaef2712b1becf2d42473d7273071ec6ceda113e` |

```bash
sha256sum "$TFPP_MODEL"/*.pth "$TFPP_MODEL/config.json"
```

다른 모델도 실행할 수 있지만 위 모델 조건의 결과를 그대로 기대하면 안 됩니다.
전체 CARLA 설치 폴더에는 `CarlaUE4.sh`와 `PythonAPI/`가 있어야 합니다.
기존 `tools/docker.sh`는 PythonAPI만 연결하므로 이 시나리오는 위 호스트 환경에서 실행합니다.

**실행 중인 CARLA 서버가 있다면 해당 터미널에서 `Ctrl+C`로 종료합니다.** 이 명령은
각 모드에서 서버를 자동 시작합니다. 수동 주행이나 다른 평가기를 동시에 실행하지 마세요.

### 2. 화면으로 전환과 복귀 확인

다음 명령은 Clean → Delayed → Takeover를 한 번씩, 총 세 번 주행합니다.
출력 경로는 아직 존재하지 않는 디렉터리로 지정합니다.

```bash
python tools/run_dual_scenario.py \
  --config configs/dual_scenario.json \
  --repetitions 1 \
  --windowed \
  --output "$EXPERIMENT_OUTPUT_ROOT/dual_visual_001"
```

CARLA 창은 ego를 따라가는 상공 시점으로 표시됩니다. 터미널에는 다음 이벤트가 출력됩니다.
Clean의 `fault_onset`은 비교용 기준 시점이며 실제 명령 유지는 발생하지 않습니다.

| 이벤트 | 확인할 동작 |
|---|---|
| `fault_onset` | 정지 선행 차량에 접근하면서 명령 유지 조건 시작 |
| `takeover_forced` | Takeover 모드의 실제 제어권이 fallback으로 전환 |
| `recovery` | 선행 차량 출발 후 간격·명령 신선도 조건을 만족하여 E2E로 복귀 |

fallback은 같은 차선에서 감속·정지하며 선행 차량이 출발하면 다시 진행합니다.
옆 차선으로 추월하는 시나리오는 아닙니다. fault 조건에 도달하지 못하거나 복귀가
발생하지 않았다면 결과의 `attention`을 확인합니다. 화면만 보고 성공으로 판단하지 마세요.

### 3. 같은 설정으로 반복 비교

화면 출력 없이 세 번 반복하면 총 아홉 번 주행합니다.

```bash
python tools/run_dual_scenario.py \
  --config configs/dual_scenario.json \
  --repetitions 3 \
  --output "$EXPERIMENT_OUTPUT_ROOT/dual_compare_001"
```

| 옵션 | 의미 |
|---|---|
| `--config` | 시나리오 JSON. 기본값은 저장소의 `configs/dual_scenario.json` |
| `--repetitions` | 세 모드를 실행할 반복 횟수. 1~3, 기본 1 |
| `--windowed` | CARLA 화면 표시. 생략하면 offscreen |
| `--output` | 새 실행 폴더. 레포·Garage·CARLA·모델 폴더 밖에 저장 |
| `--garage`, `--carla`, `--model` | 환경변수 대신 지정할 실제 경로 |
| `--gpu` | GPU 인덱스. 기본 0 |
| `--port`, `--tm-port` | CARLA RPC·Traffic Manager 포트. 기본 2000·8000 |
| `--timeout` | 평가기 timeout(초). 기본 600 |

명령에 `run` 하위 명령은 붙이지 않습니다. 이 실행은 seed 100과 고정된 전환·복귀 조건을
사용합니다. `dual_scenario.json`은 route XML, 선행 차량 위치, fallback 설정과 이 조건을
기록하며, 지원하지 않는 프로토콜 변경은 실행 전에 거부합니다.

### 4. 결과 읽기와 다시 집계

주행 명령은 실행 폴더에 결과를 자동 집계합니다.

```bash
python -m json.tool "$EXPERIMENT_OUTPUT_ROOT/dual_compare_001/comparison.json"
cat "$EXPERIMENT_OUTPUT_ROOT/dual_compare_001/metrics.csv"
```

CARLA를 실행하지 않고 저장된 로그를 다시 집계하려면 새 결과 폴더를 지정합니다.
이 도구는 시나리오의 전환·복귀와 안전 결과를 계산하는 분석 도구입니다.

```bash
python tools/compare_dual_runs.py \
  --input "$EXPERIMENT_OUTPUT_ROOT/dual_compare_001" \
  --output "$EXPERIMENT_OUTPUT_ROOT/dual_analysis_001"
```

| 출력 | 내용 |
|---|---|
| `comparison.json` | 반복별 상태 일치, 위반 결과, 이벤트, 비교 가능 여부 |
| `metrics.csv` | 모드별 이탈 프레임·충돌 콜백·fallback 유지시간·복귀 후 이동량·계산 시간 |
| `execution.json`, `suite_config.json`, `config/` | 실제 실행 목록·종료 코드·고정 설정 사본 |
| `rep_00/{clean,delayed,takeover}/` | 각 실행의 원본 평가 결과·로그·소스와 설정 hash |
| 각 실행의 `routes/RouteScenario_70001_rep0/ticks.jsonl` | 프레임별 `controller`, `dual`, 명령 나이와 실제 제출 명령 |
| 같은 route의 `fault.json`, `violations.jsonl` | 명령 유지 조건과 충돌·이탈 기록 |

`rep_01`, `rep_02`도 같은 구조입니다. 로그·소스 사본·분석 출력은 공개 레포에 커밋하지 않습니다.
다시 실행할 때는 새 출력 경로를 사용합니다. 실행 오류가 나면 이후 모드를 중단하고 기록을
보존합니다. 오류 없는 주행은 비교 효과가 없더라도 요청한 반복을 계속 수행합니다.

### 안전상 이득을 판단하는 기준

`comparison.json`의 `repetitions["0"].comparison`부터 확인합니다.

| 필드 | 해석 |
|---|---|
| `configuration_match` | 세 실행의 모델·route·seed·코드·제어 설정 일치 |
| `state_match` | Delayed와 Takeover의 전환 이전 ego·선행 차량 운동 상태가 허용 오차 이내 |
| `V_E_H`, `V_F_H` | 전환 후보 시점부터 5초 동안 각각 위반이 발생했는지 여부 |
| `stress_benefit_demonstrated` | 비교 조건을 만족하고 Delayed의 위반을 Takeover에서 방지했는지 |
| `benefit_supported_pairs` | 전체 반복 중 위 조건으로 안전상 이득이 확인된 비교 수 |

`benefit_supported_pairs`는 JSON 최상위 필드입니다. 위반은 **충돌 OR 고정 바퀴 참조점의
허용 주행 영역 이탈**입니다. `V_E_H=true`, `V_F_H=false`에 더해 상태·설정 일치,
전환 전 위반 없음, 유효한 Clean 기준 주행과 Takeover 복귀·후속 주행이 확인되어야
이 시나리오에서 안전상 이득이 확인됐다고 해석합니다.

Takeover의 `events`에 전환과 복귀가 있고 `post_recovery_frames`가 60 이상이며,
`post_recovery_progress_m`가 5m 이상인지도 확인합니다. 복귀는 최소 fallback 유지 5초,
선행 차량 출발, 간격 25m 이상, 최신 E2E 명령 및 유효한 fallback 명령 3프레임 연속을
요구합니다. 선행 차량은 fault 시작 18초 후 출발합니다.

최상위 `status=COMPLETE`와 명령 종료 코드 0은 요청된 주행 기록이 모두 분석됐다는 뜻입니다.
안전상 이득은 별도로 확인해야 합니다. 개별 `PASS`는 해당 조건 충족, `REVIEW`는 해석에 필요한
조건 미충족, `FAIL`은 실행·기록 오류, `BLOCKED`는 비교할 자료 부족을 뜻합니다.
Delayed도 안전했다면 이 조건에서는 개입의 이득이 입증되지 않은 것이며 fallback 실패는 아닙니다.
누락된 결과를 안전으로 간주하지 않습니다.

Takeover는 Delayed의 전환 이전 제출 명령을 재생하고 관측된 운동 상태를 비교합니다.
허용 오차는 위치 0.05m, 속도 벡터 0.1m/s, yaw 0.5도, 각속도 벡터 0.05rad/s,
가속도 벡터 0.5m/s²입니다. 이는 완전한 CARLA 내부 상태·모델 이력·난수 상태 복원이 아닙니다.

배경 교통은 없고 신호등은 녹색으로 고정됩니다. 원본 Garage 파일은 수정하지 않으며
실행 사본에서 사용자 정의 route의 평가 결과 이름과 선택적 화면 옵션을 맞춥니다.
평가기의 MinSpeed를 포함한 모든 위반은 결과에 남지만, 이 무교통 시나리오의 전환·복귀
성공 조건에는 MinSpeed를 사용하지 않습니다. 사용자 정의 route 점수는 공식 Bench2Drive DS가 아닙니다.

이 비교는 **해당 시나리오의 안전성**을 평가합니다. 모든 도로에서의 안전성, 제어 안정성의
수학적 보장, 위험 기반 전환 기준의 정확도를 입증하지 않습니다. TF++와 fallback은 순차로
계산되므로 `fallback_max_ms`와 `integration_max_ms`를 구분하세요. fallback만 50ms 이내여도
전체 계산은 50ms를 넘을 수 있으며, 화면 표시 여부와 장비도 처리 시간에 영향을 줍니다.


## Python에서 제어 유틸리티 사용

`ActionDelayFIFO`는 입력 명령을 지정한 호출 횟수만큼 늦춰 반환하는 기본 유틸리티입니다.
위의 실제 주행 지연 실행기는 별도의 `action_delay.py`에 있는 `ActionDelay`를 사용합니다.

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

## 문제 해결 및 코드 검사

| 증상 | 확인할 사항 |
|---|---|
| `No module named ksae_2026_autumn` | 현재 Python 환경에서 `python -m pip install -e .` 실행 |
| 가상환경 생성 시 `ensurepip` 오류 | Ubuntu 22.04에서 `sudo apt install python3.10-venv` 후 재시도 |
| Docker socket의 `permission denied` | [Docker 사용자 권한 설정](https://docs.docker.com/engine/install/linux-postinstall/) 확인 |
| 환경변수 또는 파일 경로 오류 | `configs/tfpp.yaml`과 호스트의 export 경로 확인 |
| `.pth` 개수 오류 | 모델 폴더에 `config.json`과 사용할 가중치 한 개만 배치 |
| 출력 경로가 이미 존재함 | 기본 TF++은 `--output-dir`, 텔레메트리와 결과 집계는 `--output`에 새 경로 지정 |
| `Unsupported Garage file` | 위의 지원 commit과 핵심 파일 일치 여부 확인 |
| B2D에서 CARLA 포트 사용 중 | 기존 서버를 종료하고 새 출력 경로로 재실행 |
| CARLA 연결 timeout | 서버 실행 여부, 설정한 RPC 포트, 맵 로딩 상태 확인 |
| `CUDA available: False` 또는 GPU 실행 오류 | 호스트의 `nvidia-smi`와 Container Toolkit 설정 확인 |

코드 검사는 다음 명령으로 실행합니다. GitHub Actions에서도 같은 검사를 실행합니다.

```bash
python -m pip install -e '.[dev]'
python -m ruff check .
bash -n tools/docker.sh
```
