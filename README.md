# KSAE_2026_Autumn_ws

CARLA Garage의 TransFuser++(TF++)를 실행하고, E2E와 규칙 기반 제어기의 쌍대 실행 결과로
개입 필요성과 유효성을 평가하는 Python 도구 모음입니다.

제공 기능은 **TF++ 로컬 평가 실행, 제어 지연 FIFO, 개입 판단 조건 조합, E/F 결과 채점**입니다.
실제 fallback 차량 제어, TTC/TTLC 예측, CARLA 상태 복원 및 쌍대 주행 수집기는 이 저장소에서
제공하지 않습니다. 결과 채점 도구에는 외부 수집기로 만든 E/F 결과를 입력합니다.

## 구조

```text
configs/                       실행 설정과 입력 예제
src/ksae_2026_autumn/
  control.py                   제어 지연, 후보 시점, 개입 조건
  evaluation.py                위반·개입 결과 분류 및 집계
  tfpp.py                      CARLA Garage 로컬 평가기 연결
tools/
  docker.sh                    공식 Garage 이미지 빌드/실행
  run_tfpp.py                  TF++ 실행 명령
  evaluate_pairs.py            저장된 E/F 결과 채점 명령
tests/                         CARLA 없이 실행하는 자동 검사
```

새 시각화·분석·변환 도구도 `tools/`에 파일로 추가합니다. 여러 실행에서 재사용하는 제어·평가
함수는 `src/ksae_2026_autumn/`에 추가합니다. 출력 파일은 실행 시 `outputs/` 또는 지정한 외부
경로에 생성하며 Git에서 제외합니다.

## 1. 외부 파일 준비

Linux/Ubuntu 호스트, NVIDIA GPU/driver, Docker와 NVIDIA Container Toolkit이 필요합니다.
호스트 Ubuntu 22.04에서 Garage의 Ubuntu 20.04 컨테이너를 사용할 수 있습니다.

```bash
git clone https://github.com/jungejblue/KSAE_2026_Autumn_ws.git
git clone --branch leaderboard_2 https://github.com/autonomousvision/carla_garage.git
git -C carla_garage checkout 72f39a63423a5edef6904b1487e0360a64bcf445
cd KSAE_2026_Autumn_ws
```

별도 경로에 CARLA 0.9.15와 TF++ pretrained model을 준비합니다.
[CARLA 설치](https://carla.readthedocs.io/en/0.9.15/start_quickstart/)와
[Garage 모델 안내](https://github.com/autonomousvision/carla_garage/tree/leaderboard_2#pre-trained-models)를
참고하세요. 모델 폴더에는 `config.json`과 사용할 **`.pth` 한 개**를 둡니다. Garage는 여러 가중치가
있으면 ensemble로 실행하므로, 이 저장소의 단일 모델 실행기는 가중치가 여러 개면 중단합니다.
배포 모델의 `args.txt`가 있다면 같이 보관합니다.

호스트 터미널에서 자신의 실제 경로를 설정합니다. 아래 경로는 예시입니다.

```bash
export CARLA_ROOT="$HOME/e2e_carla_ws"
export CARLA_GARAGE_ROOT="$HOME/carla_garage"
export TFPP_MODEL="$HOME/e2e_checkpoints/tfpp_single"
export EXPERIMENT_OUTPUT_ROOT="$HOME/e2e_experiment_outputs"
```

CARLA 서버·맵·Garage 원본·모델 가중치는 이 저장소에 복사하지 않습니다.

## 2. CARLA Garage Docker 이미지 사용

공식 Garage README는 공개 pull 태그 대신
[`tools/Dockerfile.master`](https://github.com/autonomousvision/carla_garage/blob/leaderboard_2/tools/Dockerfile.master)와
[`make_docker.sh`](https://github.com/autonomousvision/carla_garage/blob/leaderboard_2/tools/make_docker.sh)로
`leaderboard-user` 이미지를 만드는 방법을 안내합니다. `leaderboard-user:latest`는 이 과정에서
생기는 **로컬 이미지 이름**이며 공식 공개 이미지 주소로 가정하면 안 됩니다.

아래 명령은 외부 Garage의 Dockerfile을 수정 없이 사용합니다. 빌드에 필요한 PythonAPI와
Garage 코드를 임시 디렉터리에 모으고 가중치는 제외합니다. 이 저장소는 별도 Dockerfile이나
Compose를 관리하지 않습니다.

```bash
bash tools/docker.sh build
bash tools/docker.sh run
```

동일 Garage checkout으로 만든 이미지가 이미 있다면 `build`를 생략합니다. 이미지 이름을
직접 지정할 수도 있습니다.

```bash
export GARAGE_IMAGE=leaderboard-user:latest
bash tools/docker.sh run
```

기본 이미지 설정은 Garage의 Ubuntu 20.04, CUDA 11.7.1/cuDNN 8, Python 3.10입니다.
upstream 설치 절차의 네트워크/의존성 호환성까지 이 저장소가 보장하지는 않습니다.
이미지와 mount하는 Garage 코드의 버전을 맞춰 사용하세요.

컨테이너 내부에서 Garage의 Python 패키지를 재사용하는 가상환경을 만들고 연구 코드를 설치합니다.

```bash
python -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --no-cache-dir -e .
python -c "import carla, torch; print(torch.__version__); print(torch.cuda.is_available())"
```

다음 컨테이너 진입부터는 `source .venv/bin/activate`만 실행합니다. Garage 이미지의 Python·패키지
구성이 바뀌면 해당 이미지에 맞는 가상환경을 새로 만드세요. `exit`로 셸을 종료하면 컨테이너는
삭제되고, mount한 코드·가상환경·결과 파일은 유지됩니다.

Docker를 사용하지 않는 경우에도 Garage의 `garage_2` 환경에서 `python -m pip install -e .` 후
동일 Python 명령을 사용할 수 있습니다. 필요한 설치 절차는
[Garage Setup](https://github.com/autonomousvision/carla_garage/tree/leaderboard_2#setup)을 따릅니다.

## 3. TF++ 로컬 실행

**호스트의 별도 터미널**에서 CARLA 서버를 실행합니다.

```bash
cd "$CARLA_ROOT"
./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000
```

**컨테이너 안의 저장소 루트**에서 먼저 설정과 실행 명령을 확인한 뒤 실행합니다.

```bash
python tools/run_tfpp.py --config configs/tfpp.yaml --dry-run
python tools/run_tfpp.py --config configs/tfpp.yaml
```

`configs/tfpp.yaml`에서 포트, seed, route와 출력 경로를 변경합니다. `garage_root`, `carla_root`,
`model_dir`는 위 환경변수를 읽으며 Docker 진입 시 컨테이너 경로로 자동 연결됩니다.
상대 `routes`는 Garage 루트 기준, 상대 `output_dir`는 명령을 실행한 디렉터리 기준입니다.
기본 출력은 `${EXPERIMENT_OUTPUT_ROOT}/tfpp_debug`입니다.

실행기는 기존 출력 폴더를 덮어쓰지 않습니다. 다시 실행할 때는 `output_dir`를 새 이름으로
바꾸세요. 결과 폴더에는 Garage의 `result.json`과 실행 설정·명령·Garage commit·모델/route hash를
담은 `run.json`이 저장됩니다. `result.json` 생성이나 프로세스 종료 코드만으로 주행 성공을
판정하지 말고 route 상태·완주율·위반 항목을 확인하세요.

### Bench2Drive 평가

위 명령은 Garage의 `leaderboard_evaluator_local.py`와 `debug.xml`을 사용하는 연결 확인용
실행입니다. **공식 Bench2Drive 점수를 생성하는 실행과 다릅니다.**

Bench2Drive는 Garage의 `Bench2Drive/leaderboard/scripts/run_evaluation_tf++.sh`를 사용합니다.
이 스크립트는 경로·GPU 수 조정이 필요하며 CARLA 서버도 직접 실행합니다. 이 저장소의
Docker 실행기는 호스트 서버에 접속하는 Python 클라이언트용이므로, 여기서 benchmark 스크립트를
그대로 실행하지 마세요. Garage의 완전한 CARLA 환경에서
[공식 Bench2Drive 실행 안내](https://github.com/autonomousvision/carla_garage/tree/leaderboard_2#bench2drive)를
따르세요. 확인한 Garage checkout은 `72f39a63423a5edef6904b1487e0360a64bcf445`이며, 그 README는
내장 Bench2Drive를 v0.0.3으로 표기합니다. 별도 v0.0.4 환경을 사용한다면 같은 버전으로 간주하지
말고 평가 코드·routes 버전을 함께 맞춰야 합니다.

## 4. 제어 코드에서 재사용

20 Hz에서 100 ms 지연은 2 ticks입니다. FIFO에는 복사 가능한 Python 자료형을 넣고,
실제 `carla.VehicleControl` 변환은 호출 측에서 수행합니다.
지연 조건 0/100/200/500 ms는 각각 0/2/4/10 ticks로 설정합니다.

```python
from ksae_2026_autumn.control import ActionDelayFIFO, candidate_time, proposed_decision

fifo = ActionDelayFIFO(2, {"throttle": 0.0, "steer": 0.0, "brake": 1.0})
applied = fifo.step({"throttle": 0.3, "steer": 0.0, "brake": 0.0})
t_candidate = candidate_time(onset_s=10.0, latency_ms=100)  # 10.6 s

intervene = proposed_decision(
    action_age_gate=True,
    predicted_e2e_risk=True,
    fallback_benefit_gate=True,
    persistence_gate=True,
)
```

FIFO 초기 구간에는 전달한 초기 제어가 적용됩니다. 지연 episode의 시작·종료에서 FIFO를
어떻게 채울지는 호출 측에서 정해야 합니다. fallback 제어는 이 FIFO를 통과시키지 않습니다.
`proposed_decision`의 입력은 외부에서 계산한 판단값이며 이 함수가 TTC/TTLC를 예측하지는 않습니다.

## 5. E/F 결과 채점

동일 후보 상태에서 E는 지연된 E2E를, F는 fallback을 3초 동안 적용하여 각각 결과를 수집합니다.
F는 이 구간 중 E2E로 복귀하지 않습니다. 충돌 또는 허용 주행 영역 이탈을 위반으로 판정합니다.
바퀴 네 개 중 하나의 지면 투영점이라도 허용 주행 영역을 벗어나면 이탈 후보이며, 수치 오차를
줄이기 위해 연속 2 ticks 확인 후 `lane: true`로 기록합니다.
`any_wheel_outside_corridor()`는 한 프레임의 판정만 수행합니다. 충돌 이벤트는 원인과 무관하게
포함하며, 충돌·바퀴 좌표 검출과 상태 복원 재현성 검사는 수집기에서 수행해야 합니다.

| E 위반 | F 위반 | 결과 |
|---|---|---|
| 없음 | 없음 | UNNECESSARY |
| 있음 | 없음 | NECESSARY_EFFECTIVE |
| 있음 | 있음 | NECESSARY_INEFFECTIVE |
| 없음 | 있음 | HARMFUL |

`NECESSARY_EFFECTIVE`만 이진 분류의 positive입니다. `decision_intervene`와 조합해 TP/TN/FP/FN을
집계합니다. `valid_pair: false`는 분류 집계에서 제외하고 `invalid_reason`별 개수를 따로 셉니다.

```bash
python tools/evaluate_pairs.py configs/pairs.example.json
python tools/evaluate_pairs.py /outputs/pairs.json --output /outputs/pairs_summary.json
```

`configs/pairs.example.json`은 실제 실험 결과가 아닌 사용법 예제입니다. 예제를 실행하면 유효
4건, 제외 1건, TP=1/TN=2/FP=1/FN=0이 나옵니다. 외부 수집기 출력도 같은 JSON 구조로 저장하세요.
각 branch의 `duration_s`는 3.0이어야 합니다. 일찍 종료된 불완전 branch를 3초 결과로 채우지 말고
invalid pair로 표시하세요. 필요하면 `--horizon-seconds`로 다른 길이를 지정할 수 있으며 서로
다른 horizon의 결과는 섞어 집계하지 않습니다. 기존 결과 파일은 덮어쓰지 않습니다.

## 코드 검사

Python 3.10 환경에서 다음을 실행합니다. GitHub Actions도 같은 검사를 실행하며 CARLA/GPU는
사용하지 않습니다.

```bash
python -m pip install -e '.[dev]'
python -m ruff check .
python -m unittest discover -s tests -v
bash -n tools/docker.sh
python tools/evaluate_pairs.py configs/pairs.example.json
```
