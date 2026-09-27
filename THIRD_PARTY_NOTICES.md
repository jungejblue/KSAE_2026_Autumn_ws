# External components

이 저장소는 아래 외부 구성요소를 실행 환경에서 사용합니다. CARLA 배포 파일·맵·Garage checkout·모델 가중치는 저장소에 포함하지 않습니다. 아래 목록은 실행에 중요한 구성요소 안내이며 전체 전이 의존성의 라이선스 목록을 대체하지 않습니다.

| 구성요소 | 용도 | 원문·조건 |
|---|---|---|
| CARLA 0.9.15 | 시뮬레이터·Python API | [코드 LICENSE: MIT](https://github.com/carla-simulator/carla/blob/0.9.15/LICENSE). 서버 배포물·맵·UE 구성요소는 각 배포물의 고지도 확인 |
| CARLA Garage | TransFuser++·평가기·공식 Dockerfile | [지원 commit의 LICENSE: MIT](https://github.com/autonomousvision/carla_garage/blob/72f39a63423a5edef6904b1487e0360a64bcf445/LICENSE) |
| Garage pretrained models | TransFuser++ 추론 | [모델 배포 안내: CC BY 4.0](https://github.com/autonomousvision/carla_garage/tree/72f39a63423a5edef6904b1487e0360a64bcf445#pre-trained-models) |
| Garage 통합 Leaderboard·ScenarioRunner·Bench2Drive | route 평가·시나리오 실행 | 사용한 Garage checkout 내부 각 구성요소의 원래 고지와 [상위 저장소 안내](https://github.com/autonomousvision/carla_garage/tree/72f39a63423a5edef6904b1487e0360a64bcf445) |
| Python·CUDA·PyTorch·NumPy 및 기타 환경 의존성 | 실행 환경·수치 계산 | 설치한 버전·이미지에 포함된 각 라이선스 |
| CasADi/Ipopt | 선택적 최적화 backend | 해당 배포물의 라이선스. 기본 sampled backend에는 이 선택 의존성 불필요 |

Garage LICENSE의 저작권 표기는 Bernhard Jaeger, Julian Zimmerlin, Jens Beißwenger, Kashyap Chitta, Andreas Geiger (2024)입니다. CARLA LICENSE의 표기는 Computer Vision Center (CVC), Universitat Autonoma de Barcelona (UAB) (2017)입니다. 원문 링크에서 전체 조건을 확인할 수 있습니다.

외부 코드·이미지·모델을 별도로 재배포할 때는 해당 파일의 원래 라이선스와 고지를 보존해야 합니다. 이 안내는 외부 구성요소를 이 저장소의 라이선스로 변경하지 않습니다. 실행 결과 폴더에 저장된 외부 소스 사본도 외부 코드의 조건을 따릅니다.

이 저장소 자체의 라이선스 상태는 [LICENSE.md](LICENSE.md), 소프트웨어와 외부 연구의 인용 안내는 [docs/reference.md](docs/reference.md#citation)에 있습니다.
