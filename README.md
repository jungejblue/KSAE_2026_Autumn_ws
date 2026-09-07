# KSAE_2026_Autumn_ws

**CARLA에서 TransFuser++를 실행하고, 주행 제어 결과를 비교·집계하는 Python 도구 모음입니다.**

| 기능 | 사용 방법 | 출력 |
|---|---|---|
| 두 제어 모드의 결과 비교 | `tools/evaluate_pairs.py`에 결과 JSON 입력 | 결과 분류, 혼동행렬, precision/recall |
| TransFuser++ 주행 실행 | `tools/run_tfpp.py`에 설정 YAML 입력 | CARLA Garage 평가 결과와 실행 정보 |
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

## Python에서 제어 유틸리티 사용

`ActionDelayFIFO`는 입력 명령을 지정한 호출 횟수만큼 늦춰 반환합니다.

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
| `configs/` | 실행 설정과 결과 입력 예제 |
| `src/ksae_2026_autumn/control.py` | 명령 지연과 전환 판단 함수 |
| `src/ksae_2026_autumn/evaluation.py` | 위반 판정, 결과 분류, 지표 집계 |
| `src/ksae_2026_autumn/tfpp.py` | Garage 평가기 실행과 결과 경로 관리 |
| `tools/` | 실행·결과 처리 명령 |
| `tests/` | 자동 검사 |

## 문제 해결 및 코드 검사

| 증상 | 확인할 사항 |
|---|---|
| `No module named ksae_2026_autumn` | 현재 Python 환경에서 `python -m pip install -e .` 실행 |
| 가상환경 생성 시 `ensurepip` 오류 | Ubuntu 22.04에서 `sudo apt install python3.10-venv` 후 재시도 |
| Docker socket의 `permission denied` | [Docker 사용자 권한 설정](https://docs.docker.com/engine/install/linux-postinstall/) 확인 |
| 환경변수 또는 파일 경로 오류 | `configs/tfpp.yaml`과 호스트의 export 경로 확인 |
| `.pth` 개수 오류 | 모델 폴더에 `config.json`과 사용할 가중치 한 개만 배치 |
| 출력 경로가 이미 존재함 | TF++은 `--output-dir`, 결과 집계는 `--output`에 새 경로 지정 |
| CARLA 연결 timeout | 서버 실행 여부, 설정한 RPC 포트, 맵 로딩 상태 확인 |
| `CUDA available: False` 또는 GPU 실행 오류 | 호스트의 `nvidia-smi`와 Container Toolkit 설정 확인 |

코드 검사는 다음 명령으로 실행합니다. GitHub Actions에서도 같은 검사를 실행합니다.

```bash
python -m pip install -e '.[dev]'
python -m ruff check .
python -m unittest discover -s tests -v
bash -n tools/docker.sh
```
