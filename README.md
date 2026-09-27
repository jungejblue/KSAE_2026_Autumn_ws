# KSAE_2026_Autumn_ws

**CARLA에서 TransFuser++와 fallback 제어를 연결하고, 제어권 전환·복귀 및 두 제어기의 안전 결과를 비교하는 Python 도구 모음입니다.**

정지 선행 차량 시나리오의 E2E → fallback → E2E 복귀를 실행하거나, 공통 후보 입력에서 E2E와 fallback의 독립 주행을 비교할 수 있습니다. 주행 기록, 명령 지연 주입, 오프라인 안전 지표 추출도 제공합니다.

[설치·실행 설명서](docs/usage.md) · [설정·API 참고](docs/reference.md) · [인용](CITATION.cff) · [라이선스 상태](LICENSE.md)

## 기능 선택

| 하고 싶은 일 | 진입점 | 설명서 |
|---|---|---|
| 지연 E2E → fallback → E2E 복귀 실행 | `tools/run_dual_scenario.py` | [듀얼 시나리오](docs/usage.md#dual) |
| 후보 시점에서 E2E와 fallback 독립 주행 비교 | `tools/run_paired_replay.py run` | [Paired replay](docs/usage.md#paired) |
| TransFuser++ 기본 주행 | `tools/run_tfpp.py` | [기본 실행](docs/usage.md#tfpp) |
| 차량 상태·명령 기록 | `tools/run_telemetry.py run` | [주행 기록](docs/usage.md#telemetry) |
| 충돌·영역 이탈 기록 및 FIFO 지연 주입 | `tools/run_violations.py run` | [위반 기록](docs/usage.md#violations) · [지연 주입](docs/usage.md#delay) |
| TTC·TTLC·충돌 직전 속도 추출 | `tools/extract_safety_metrics.py` | [안전 지표](docs/usage.md#metrics) |
| 저장된 결과 재집계 | `tools/compare_dual_runs.py`, `tools/run_paired_replay.py review` | [듀얼](docs/usage.md#dual) · [Paired](docs/usage.md#paired) |
| 직접 수집한 쌍대 결과 JSON 집계 | `tools/evaluate_pairs.py` | [입력 형식·분류](docs/reference.md#pairs) |
| fallback 제어기를 다른 실행기에 연결 | `fallback_control` | [Python API](docs/reference.md#fallback) |

## 실행 환경

| 사용 범위 | 필요한 환경 |
|---|---|
| 예제 결과 JSON 집계 | Python 3.10; CARLA·GPU 불필요 |
| fallback 코어, paired replay 결과 재집계 | Python 3.10 + `pip install -e '.[fallback]'` |
| TF++ 기본 주행·Local 기록 | CARLA 0.9.15 서버, Garage, 대응 모델, NVIDIA GPU; 호스트 또는 Garage Docker 환경 |
| 듀얼·paired replay·Bench2Drive 실행 | **전체 CARLA 설치와 Garage 의존성을 갖춘 호스트 Python 환경** |

호스트 설치 안내는 Ubuntu 22.04 기준입니다. Garage는 `leaderboard_2`의 commit `72f39a63423a5edef6904b1487e0360a64bcf445`를 사용합니다. 모델 폴더에는 서로 대응하는 `config.json`과 `.pth` 한 개가 필요합니다.

`tools/docker.sh`는 Garage 공식 Dockerfile로 이미지를 빌드합니다. Compose는 사용하지 않습니다. 현재 컨테이너에는 CARLA PythonAPI만 연결하므로, CARLA 서버를 자동 시작하는 듀얼·paired replay는 호스트에서 실행합니다.

## 설치와 첫 시나리오 실행

먼저 [환경 준비](docs/usage.md#setup)에 따라 CARLA·Garage·모델과 `garage_2` 환경을 준비합니다. 이미 준비했다면 아래부터 실행하세요. 경로는 실제 설치 위치로 바꿉니다.

```bash
git clone https://github.com/jungejblue/KSAE_2026_Autumn_ws.git "$HOME/KSAE_2026_Autumn_ws"
cd "$HOME/KSAE_2026_Autumn_ws"
conda activate garage_2
python -m pip install -e '.[fallback]'

export CARLA_ROOT="$HOME/CARLA_0.9.15"
export CARLA_GARAGE_ROOT="$HOME/carla_garage"
export TFPP_MODEL="$HOME/models/tfpp"
export EXPERIMENT_OUTPUT_ROOT="$HOME/carla_outputs"
```

이미 저장소를 받았다면 `git clone`은 생략합니다. 이후 명령은 항상 저장소 루트에서 실행합니다. 듀얼·paired replay를 시작하기 전에는 실행 중인 CARLA 서버를 해당 터미널의 `Ctrl+C`로 종료하세요. 실행기가 분기마다 서버를 시작합니다.

### E2E → fallback → E2E 복귀

```bash
python tools/run_dual_scenario.py \
  --config configs/dual_scenario.json \
  --repetitions 1 --windowed \
  --output "$EXPERIMENT_OUTPUT_ROOT/dual_visual_001"

python -m json.tool "$EXPERIMENT_OUTPUT_ROOT/dual_visual_001/comparison.json"
```

Clean·Delayed·Takeover를 한 번씩 실행합니다. `--windowed`를 생략하면 offscreen으로 실행합니다. fallback은 같은 차선에서 감속·정지하고, 선행 차량이 출발한 뒤 복귀 조건을 만족하면 E2E로 돌아갑니다.

`comparison.json`의 `benefit_supported_pairs`와 각 반복의 비교 조건·복귀 결과를 함께 확인합니다. `status=COMPLETE`만으로 안전상 이득을 판단하지 않습니다. [옵션·출력 해석](docs/usage.md#dual)

### 공통 후보 입력에서 쌍대 비교

```bash
python tools/run_paired_replay.py run \
  --repetitions 1 --seed 100 \
  --output "$EXPERIMENT_OUTPUT_ROOT/paired_001"

python -m json.tool "$EXPERIMENT_OUTPUT_ROOT/paired_001/review.json"
```

기준 분기 E0와 E2E 분기 E1·E2, fallback 분기 F1·F2를 실행합니다. 후보 이후 3초의 위반을 비교하며, 후보 이후에는 각 분기의 실시간 센서 입력을 사용합니다.

`review.json`의 `PASS`는 비교 조건 충족을 뜻합니다. 유효한 비교에서 `observed_V_E=true`, `observed_V_F=false`일 때 해당 구간의 위반 방지 효과를 해석합니다. [옵션·복원 범위·출력 해석](docs/usage.md#paired)

출력 경로는 **매번 새로운 디렉터리**로 지정합니다. 듀얼·paired replay 결과는 저장소·Garage·CARLA·모델 폴더 밖에 저장합니다.

## CARLA 없이 결과 집계 체험

시뮬레이터 설치 전에는 예제 JSON으로 집계 기능을 사용할 수 있습니다.

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python tools/evaluate_pairs.py configs/pairs.example.json
```

예제 출력에는 `total_pairs=5`, `valid_pairs=4`, `invalid_pairs=1`, `precision=0.5`, `recall=1.0`이 포함됩니다. 이는 사용법 확인용 입력이며 시뮬레이터에서 측정한 성능 결과가 아닙니다. GPU 주행으로 돌아갈 때는 이 가상환경을 종료하고 Garage 환경을 활성화합니다.

## 결과 해석 범위

- 듀얼 시나리오는 고정 시점의 전환·복귀를 실행하며 후보 이후 5초의 위반을 평가합니다. Paired replay는 후보 이후 3초의 독립 주행을 비교합니다.
- 시나리오의 fault는 마지막 명령 유지입니다. `run_violations.py --delay-ms`의 FIFO 지연과 구분합니다.
- 비교 가능성 검사와 위반 결과를 함께 해석합니다. 누락된 실행은 안전한 결과로 처리하지 않습니다.
- 위험 기반 자동 전환, 완전한 CARLA 물리 snapshot 복원, 모든 도로에서의 안전성 및 전체 시스템 실시간 성능을 보장하지 않습니다.

## 저장소 구성

| 경로 | 역할 |
|---|---|
| `configs/` | 모델 실행 설정, 시나리오·route, fallback 설정, 결과 입력 예제 |
| `src/ksae_2026_autumn/` | 실행, 주행 기록, 지연·전환, 쌍대 비교, 결과 분석 |
| `src/fallback_control/` | fallback 제어 코어와 CARLA adapter |
| `tools/` | 사용자 실행 명령과 Docker 도구 |
| `docs/usage.md` | 환경 준비, 실행 명령, 출력 해석, 문제 해결 |
| `docs/reference.md` | 설정 파일, JSON 형식, Python API, 코드 역할 |

CARLA 바이너리·맵·Garage 원본·모델·전체 실행 로그는 이 저장소에 포함하지 않습니다. 파일별 역할은 [참고 문서](docs/reference.md#files)에 있습니다.

## 문의·인용·라이선스

오류 보고는 [Issues](https://github.com/jungejblue/KSAE_2026_Autumn_ws/issues)에 사용한 commit, 명령어, 관련 설정과 오류 메시지를 포함해 작성하세요. 개인 경로·인증정보·모델 가중치는 제외합니다.

소프트웨어 인용은 [CITATION.cff](CITATION.cff)와 [인용 안내](docs/reference.md#citation)를 참고하세요. 실행에 사용한 commit과 모델 조건도 함께 남기는 것이 좋습니다.

이 저장소 자체의 사용 허락은 [LICENSE.md](LICENSE.md)를 확인하세요. 현재 명시적 오픈소스 라이선스는 선언하지 않았습니다. CARLA·Garage·모델에는 각각의 조건이 적용되며 [외부 구성요소 안내](THIRD_PARTY_NOTICES.md)에 원문을 연결했습니다.
