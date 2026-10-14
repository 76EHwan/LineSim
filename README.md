# LineSim

[BLDC-LINETRACER](https://github.com/76EHwan/BLDC-LINETRACER) 펌웨어를 **수정하지 않고 그대로** PC에서 돌리는 MuJoCo 기반 SIL(Software-in-the-Loop) 시뮬레이터입니다.
시뮬레이터에서 튜닝한 코드와 파라미터를 그대로 STM32CubeIDE로 빌드해 로봇에 넣는 것이 목표입니다.

```
┌──────────────── linesim.exe ─────────────────────────────────────────────┐
│  firmware fiber                         simulator fiber                    │
│  ┌──────────────────────────┐           ┌──────────────────────────────┐  │
│  │ Main/Src/*.c (원본 그대로)│  HAL shim │ 가상 시간 스케줄러            │  │
│  │ drive / sensor / motor / │◄────────►│  TIM7 25µs → ADC3 (IR 스캔)   │  │
│  │ foc / menu / sd_ui       │ 레지스터  │  TIM13/14 2kHz (속도/조향)    │  │
│  │ BSP: button, buzzer,     │ GPIO, I2C │  SysTick 1kHz → IMU I2C DMA   │  │
│  │ lsm6ds3, SDcard          │ FatFs     │  MuJoCo 10kHz 물리 스텝       │  │
│  └──────────────────────────┘           └──────────────────────────────┘  │
│                                           로봇(MuJoCo) · 모터 · IR · IMU · 트랙 │
└────────────────────────────────────────────────────────────────────────────┘
```

## 빌드

필요한 것: Visual Studio 2022 Build Tools(C++ 워크로드, CMake 포함). MuJoCo 3.14 바이너리는 `third_party/mujoco`에 들어 있습니다.

```bash
git submodule update --init
build.bat
```

결과물: `build/Release/linesim.exe`

## VS Code

`E:\kth59\LineSim` 폴더를 열면 됩니다 (C/C++ 확장 `ms-vscode.cpptools` 필요). 실행하면 모두 3D 창이 뜹니다.

| 하고 싶은 것 | 방법 |
|---|---|
| 빌드 | `Ctrl+Shift+B` |
| **계속 주행하며 튜닝** | `F5` (무작위 코스) 또는 작업 **LineSim: 3D session** → 코스 선택(`random` / figure8 / oval) → 기본 속도, 최고 속도, 가속도, 감속도 입력 |
| 시나리오 그대로 보기 | 작업 **LineSim: 3D scenario** |
| 펌웨어 메뉴를 키보드로 | 작업 **LineSim: 3D interactive firmware menu** |
| 무작위 코스 파일 만들기 | 작업 **LineSim: Generate random course** (`tracks/random_<seed>.json`) |
| 창 없이 빠르게 | 작업 **LineSim: Run scenario (no window)** → **Open results in browser** |
| 펌웨어 디버깅 | 실행 및 디버그에서 **Debug firmware** 선택 후 `F5`. `firmware/Main/Src/*.c`에 바로 브레이크포인트 |

결과는 `runs/latest`에 덮어씁니다(가상 SD 카드는 유지). 남길 결과는 `--out runs/이름`.

## 3D 세션 (계속 주행)

```bash
build\Release\linesim.exe view --session --track random --robot robots/full.json
```

펌웨어가 한 번 부팅한 뒤 창을 닫을 때까지 같은 전원 세션으로 계속 주행합니다. 1회차에서 기록한 마커 로그로 2~4회차를 반복할 수 있습니다.

| 키 | 동작 |
|---|---|
| `1` `2` `3` `4` | 해당 회차 주행 (`Drive_First` … `Drive_Fourth`). 로봇은 출발선에 자동으로 놓입니다 |
| `↑` `↓` / `←` `→` | 파라미터 선택 / 변경 (`Shift` = 10배, 속도는 0.01 m/s 단위): 기본·최고 속도, 가·감속, steer kp/kd, pos atten, target shift, pit-in. 다음 주행부터 적용. steer kd는 초 단위로 보이고 괄호 안이 로봇 메뉴에 넣을 값(`steer_gain_d` = kd ÷ 0.0005) |
| `Enter` → 숫자 → `Enter` | 선택한 파라미터 값을 직접 입력 (예: steer kp `18.5`). `Esc` 취소, `Backspace` 지우기. 입력 중에는 숫자·`-` 키가 주행·배속으로 쓰이지 않음 |
| `A` | **자동 튜닝(끝없이)**: 지금 코스에서 1회차 → 4회차(실전 주행)를 계속 돌리며 기본 속도, steer kp/kd, lat acc max, slalom ratio를 찾습니다. 아래 설명 참고. 다시 `A`(또는 `X`)를 누르면 멈추고 찾은 최고값을 남깁니다 |
| `N` | 무작위 코스 새로 생성 (`runs/latest/random_<seed>.json`에 저장). 새 코스에서는 1회차부터 |
| `X` | 주행 중인 로봇 들어 올리기 (펌웨어가 Line Lost로 종료) |
| `R` | 로봇을 출발선으로 |
| `V` | 오른쪽 아래 **센서 시점**(센서바 바로 위에서 내려다봄, 진행 방향이 위) 켜기/끄기. 로봇은 반투명, IR 센서 16개는 바닥의 점(노랑 = 임계값 초과) |
| `C` | 메인 카메라: 추적 → 센서 시점 → 위에서 전체 → 자유 |
| `Space` `+` `-` `S` `T` `P` `Esc` | 일시정지, 시뮬레이션 배율(×1/16 ~ ×256, 왼쪽 위 `sim speed`; 계산이 못 따라가면 `(running x4.1)`처럼 실제 배율 표시), 그림자, 주행 궤적 표시 켜기/끄기(끄면 지워짐, 주행마다 새로 그림), 스크린샷, 종료 |

왼쪽 위 `speed`는 차체 속도, `wheels`는 바퀴 속도, `slip`은 둘의 차이(곡선에서 마찰 한계에 걸리면 커짐: 엔코더 거리가 그만큼 길게 쌓임)입니다. 오른쪽 위 패널에 코스 이름, **정답 마커 순서**(코스 형상에서 자동 계산), 파라미터, 주행 기록(결과, 랩타임, 최고 속도, 마커 정답 여부: `OK` 또는 처음 틀린 위치)이 나옵니다.

## 무작위 코스

```bash
build\Release\linesim.exe gen --seed 7 --out tracks/my.json
```

기본은 **35–55 m 긴 코스**(예선판 코스의 약 3배)를 본선 판 **6.1 × 4.88 m**에 촘촘하게 만듭니다. 사각형 루프나 8자 루프(두 45° 대각선이 가운데서 직각 교차)에서 시작해 한 단계씩 모양을 바꾸고, 바꿀 때마다 아래 규정을 전부 검사합니다.
- 직선에 우회 구간 끼우기: 270° 원 여러 개가 이어진 뱀 모양, 열쇠 구멍 모양 벌브, 손가락, 시케인, 좌우 대칭 무작위 모양, 자기 직선을 직각으로 가로지르는 루프
- 두 직선 사이 원호 바꾸기: 모서리 깎기, 벌브 모서리, S자. 바뀐 위치 차이는 양쪽 직선 길이로 맞춥니다.

규정 외에 교차로 앞 직선은 40 cm 이상으로 둡니다. 급커브를 빠져나와 20 cm 만에 비스듬히 교차로를 지나면 엔드마크로 읽히기 때문입니다. 생성에는 1–12초 걸립니다. 예선판 크기의 짧은 코스는 `--board 4.88x4.88 --length 8,30`(예전 방식: 무작위 조각을 한 번에 이어 붙임).

ZETIN 규정을 지키는 닫힌 코스를 만듭니다. **교차로는 항상 1개 이상** 들어갑니다(`--crossings n`은 정확히 n개, `--min-crossings 0`은 교차 없는 코스도 허용): 원호 R 25 cm·45° 단위(45~270°), 같은 방향 원호는 연속되지 않음, 교차는 직각이고 교차 중심 앞뒤 20 cm 이상 직선, 출발·도착 엔드마크(도착 전 20 cm, 사이 30 cm 이상, 출발 후 20 cm 이상 직선), 다른 부분과 20 cm 이상 간격, 판 크기 안. 

## 로봇 모델: Full.step

`robots/full.json`은 `tools/step_import.py`가 `robots/Full.step`에서 만든 로봇입니다(시나리오 기본값). CAD 원본과 여기서 만든 메시·`full.json`은 저장소에 없습니다. `robots/Full.step`을 넣고 아래 명령으로 만드세요. 없으면 시뮬레이터가 경고를 띄우고 `robots/default.json`(바퀴 2개 단순 모델)로 실행합니다.

```bash
python -m venv tools\.venv
tools\.venv\Scripts\python.exe -m pip install cascadio trimesh numpy
tools\.venv\Scripts\python.exe tools\step_import.py robots\Full.step --total-mass 0.4
```

| CAD에서 가져오는 것 | 값 |
|---|---|
| 바퀴 | 4개(`Tire*`), 좌우 각 2개가 기어로 연동(모터 1개, 39T 2개). 반지름 11.5 mm, 폭 40 mm, 좌우 187 mm, 앞뒤 ±12.5 mm |
| IR 센서 16개(`ITR8307`) | 원호 배치: 가운데 132 mm, 양 끝 106 mm 앞. 좌우 ±76.4 mm(간격 10.9→8.9 mm), 지면 위 3.3 mm |
| 바닥에 닿을 수 있는 부분 | 플레이트, 센서보드, 센서 넥 (충돌 박스) |
| 외형 | 부품 그룹별 메시(`robots/mesh/*.stl`), 4 mm 미만 칩 부품 제외. 뷰어 속도를 위해 0.6 mm 격자로 단순화(약 38만 → 7.5만 면, 물리에는 영향 없음). `--view-cell 0.4`는 더 세밀하게, `--view-cell 0`은 원본 그대로 |

전체 질량은 실측 400 g입니다(`--total-mass 0.4`). STEP에 재질 정보가 없어서 부품별 분배(무게중심·관성)는 일반적인 부품 질량 비율로 추정합니다(`step_import.py`의 `ASSUMED_MASS`). 균형점을 재면 `robots/full.json`의 `com`을 바꿔 주세요.

펌웨어 상수와 CAD가 다른 점: `SENSOR_DIST_L` 0.13 m(CAD 0.106~0.132 m 원호), `SENSOR_HALF_WIDTH` 0.08 m(CAD 0.0764 m), `line_sensor_pos[]`는 균일 간격(CAD는 바깥으로 갈수록 좁음). 시뮬레이터의 센서는 CAD 위치를 쓰므로 이 차이가 위치 추정에 그대로 반영됩니다.

## 실제 주행 로그와 비교

펌웨어가 SD에 남긴 `save_slot_N.txt`(마커 종류, 직전 마커부터 오도메트리 거리, yaw)를 씁니다. `real_logs/`는 git에서 제외됩니다.

```bash
python tools/logs.py "real_logs/주행 로그" "real_logs/주행 로그/연습판 마커 정답.txt"
python tools/track_from_logs.py "real_logs/주행 로그" "real_logs/주행 로그/연습판 마커 정답.txt" tracks/practice_board.json --name practice_board
python tools/compare_logs.py "real_logs/주행 로그" "real_logs/주행 로그/연습판 마커 정답.txt" tracks/practice_board.json
```

- `logs.py`: 로그마다 파라미터, 정답과 처음 달라지는 마커. 정답과 일치한 로그의 구간별 거리 중앙값.
- `track_from_logs.py`: 규정(턴마크 의미, R 25 cm, 45° 단위) + 거리 + "닫힌 코스, 교차로는 직각 하나" 조건으로 코스를 재구성. 결과는 후보 중 가장 잘 맞는 것이며 일부 원호 각도는 틀릴 수 있습니다(연습판 도면이 있으면 직접 입력하는 것이 정확).
- `compare_logs.py`: 실제 로그의 파라미터 조합마다 시뮬레이터로 1회차를 돌려, 마커를 처음 틀리는 위치와 구간 거리를 비교. 로그를 남긴 시점의 펌웨어와 `firmware/` 버전이 다르면 결과도 달라집니다.
- `oscillation.py`: 직선 구간의 흔들림(yaw 진폭·주파수). 시뮬레이터 결과나 Vibe Test 로그(`vib_log_*.csv`)를 그대로 넣으면 됩니다.

## 자동 튜닝: 기본 속도 + 조향 PD

1회차는 학습 주행이라 **데이터가 깨끗해야** 하고(마커 전부 정확, 흔들림·미끄러짐 적게), 2회차가 그 데이터로 직선에서 가속합니다. 그래서 두 도구 모두 **1회차 → 2회차를 한 번의 시도**로 평가합니다.

- **통과**: 1·2회차 모두 완주, 라인에서 70 mm 이내, 마커 전부 정확.
- **점수**(낮을수록 좋음): 5 × 2회차 랩타임[s] + 1회차 깨끗함(라인 오차 rms[mm] + 흔들림[dps]/20 + 바퀴 미끄러짐[%]/2).
- 2회차의 최고 속도·가감속은 패널(또는 `--set max_mps=… accel=… decel=…`) 값 그대로이고, 찾는 것은 **기본 속도(곡선·1회차 속도)와 steer kp/kd**입니다.
- **모델 오차 스트레스**: 조향 지연(`sensor_delay` 7 ms)은 자이로 로그 하나로 맞춘 값이라 실제는 더 클 수 있습니다. 통과한 게인은 지연 ×1.4(≈10 ms)에서 1회차를 다시 돌려 라인을 지키고 흔들림이 1.5배 + 10 dps 이내여야 하고, ×1.85(≈13 ms)에서도 라인을 지켜야 합니다. 못 버티면 "불안정"으로 칩니다.
- 안정 경계 바로 옆 값은 피합니다(kp·kd 한 칸 옆이 라인을 놓치거나 불안정하면 선택하지 않음).
- 최고 점수의 1 % 안(2회차 약 0.1 s)이면 **더 순한 게인**(낮은 kp, 그다음 낮은 kd)을 고릅니다. 큰 kd는 실제 센서 위치의 계단형 변화를 키우는데 시뮬레이터는 이를 부드럽게 봅니다.

**3D 세션 (`A` 키, 끝없이)**: 연습판처럼 실제와 같은 코스를 열고(`--track tracks/practice_board.json`) `A`. 기본 속도, steer kp/kd, lat acc max, slalom ratio를 찾습니다.

- **코스 3개로 평가합니다(한 코스에 맞춰지는 것 방지).**
  - 지금 코스(c1)와 무작위 코스 2개(c2, c3)에서 시도마다 1회차 → 4회차를 돕니다. 나머지 파라미터는 패널 값 그대로입니다.
  - 모든 코스에서 완주하고 마커가 맞아야 통과입니다.
  - 기록은 코스 길이로 나눈 **페이스**(4회차 랩 / 길이, 초/10 m)의 평균입니다.
  - 랩 목록에는 `c2` 같은 코스 표시가 붙습니다.
- **새 BEST 없이 100번 주행하면** 무작위 코스(c2, c3)를 번갈아 하나씩 새 코스로 바꾸고, BEST 값에서 다시 시작합니다. 100번은 랩 목록의 `#번호`처럼 1회차, 4회차, 스트레스 주행을 모두 센 횟수입니다.
- **두 종류 라운드를 번갈아 돌립니다.**
  - 게인 라운드: kp × kd
  - 곡선 진입 라운드: lat acc max × slalom ratio. 직선 가속 뒤 곡선에서 밀리면(오버슛) 이 두 값으로 감속을 조절합니다.
- **한 라운드 = 3×3 블록**이고, 연속값으로 좁혀 갑니다.
  - 처음 간격: kp 10 %, kd 15 %(최소 0.01 s), lat 1 m/s², slalom 0.03
  - 점수가 나온 칸이 6개 이상이면 2차 곡면을 맞춰 칸 사이의 최적점을 찾고, 그 점을 10번째로 시험합니다(표 아래 `fit`).
  - 이긴 조합이 블록 가운데나 `fit` 점이면 간격을 반으로 줄입니다.
  - 가장자리이면 블록을 그쪽으로 옮깁니다.
  - 통과가 없어도 간격을 반으로 줄입니다.
  - 최소 간격: kp 0.25, kd 0.005 s, lat 0.25, slalom 0.01
- **점수**(낮을수록 좋음): 코스 평균 [정확도(1) + 정확도(4)] + 10 × 페이스. 1회차는 정확도만, 4회차는 정확도 + 기록입니다.
  - 정확도 = 곡선 구간 라인 오차 rms[mm] + 흔들림/20 + 바퀴 미끄러짐[%]/2
  - 직선은 어떤 게인이든 잘 따라가므로 라인 오차에서 뺍니다.
- **스트레스 주행:** 조향 지연 ×1.4, ×1.85에서 c1 1회차를 다시 돕니다.
- **통과해도 페이스가 BEST보다 느리면 버립니다**(표에 `s`). 그래서 통과가 나오면 항상 새 BEST입니다.
- **속도**는 게인 + 곡선 라운드 한 쌍마다 정합니다.
  - 그 속도에서 통과가 있었으면 +0.1 m/s를 시도합니다.
  - 없었으면 BEST 속도 + 절반 간격으로 돌아가며 0.01 m/s까지 좁힙니다.
- **패널**
  - 코스 목록
  - 이번 블록 표: `.` 대기, `x` 라인 놓침/불안정, `m` 마커 오독, `s` 통과했지만 BEST보다 느림, 숫자 = 점수
  - **BEST**: 속도, 네 값, 페이스와 코스별 랩
  - 최근 시도와 고정된 나머지 파라미터
- 시도마다 코스 3개 × (1·4회차)와 스트레스 주행이 있어 시간이 걸립니다. `+`로 배속을 올리세요.
- 다시 `A`를 누르면 BEST 값이 파라미터로 남고 c1 코스로 돌아갑니다. 빨리 결과만 필요하면 아래 격자 탐색(병렬)이 훨씬 빠릅니다.

**격자 탐색 (창 없이)**:

```bash
tools\.venv\Scripts\python.exe tools\autotune.py 2.0
tools\.venv\Scripts\python.exe tools\autotune.py 1.6 --max-speed
```

VS Code: **Run Task → LineSim: Auto-tune steering gains (target speed)**. 코스는 `tracks/practice_board.json`이 있으면 그것, 없으면 무작위 코스 4개(고정 시드), `--track`으로 지정 가능. kp 8–40 × kd 0–0.5 s(104조합)를 11개씩 병렬로 돌립니다. `--max-speed`는 통과하는 가장 높은 기본 속도를 찾습니다. 결과: `runs/tune/<속도>/result.txt`(kp × kd 표, 추천값, 3D 세션 명령), `runs/tune/max_speed.txt`.

**steer kd 단위**: 펌웨어의 `steer_pid`는 CMSIS `arm_pid_f32`이고 조향 주기(TIM14, `Core/Src/tim.c`에서 읽은 500 µs)마다 한 번 불립니다. Kd는 호출 사이 오차 변화에 곱해지므로 초 단위 kd를 쓰려면 **로봇 값 = kd ÷ 0.0005**(예: 0.3 s → 600)입니다. 시뮬레이터 화면과 도구는 초 단위로 보여주고 로봇 값을 함께 적습니다(`--set steer_kd_s=0.3`, 펌웨어 값 그대로는 `--set steer_kd=600`). 지금까지 쓰던 kd 0.2–1.3은 0.0001–0.00065 s라 미분이 거의 없습니다.

**접지 한계 감속 · 캐스케이드 조향** (펌웨어 브랜치 `feat/cascade-steer`): 자이로를 1.66 kHz로 바꾸고 1 ms마다 읽습니다(원래 104 Hz, 10 ms).

- `lat_acc_max` [m/s²]: 속도 × 자이로 요레이트(= 횡가속도)가 이 값을 넘으면 그만큼 감속(빨리 내리고 약 0.1 s에 걸쳐 회복). 곡선을 미끄러지지 않고 접지 한계 속도로 돌아 1회차 데이터(마커, 엔코더 거리)가 깨끗해집니다. 연습판 시뮬레이션: 기본 속도 2.4 m/s에서 1·2회차 통과 0/3 → 3/3(`lat_acc_max` 12), 1회차 미끄러짐 35 % → 9 %.
- `yaw_kp`, `yaw_ki`: 라인 PD 출력(곡률)을 목표 요레이트(속도 × 곡률)로 바꾸고 자이로로 추종하는 안쪽 루프. 0 / 0이면 기존 PD와 같습니다. 시뮬레이션에서는 kp 33 / kd 0의 직선 진동을 `yaw_kp` 2로 없애지만(kd와 같은 효과), 4 이상이나 적분을 넣으면 불안정하고, 마커 인식은 개선하지 못했습니다.
- 로봇 메뉴: `Yaw KP`, `Yaw KI`, `Lat Acc Max`. 시뮬레이터: `--set yaw_kp=… yaw_ki=… lat_acc_max=…`, 세션 패널.
- 자동 튜닝에 축으로 추가: `--lat 10,12,14,16`. 자동 튜닝은 접지 스트레스도 봅니다(타이어 마찰 ×0.85에서도 1회차가 마커까지 맞아야 통과). 실제 마찰계수를 정확히 모르기 때문입니다.
- 실제 로봇에 넣기 전: 왼쪽으로 돌릴 때 `Gyro_Z`가 +인지(부호), 1 ms I2C DMA 읽기가 매번 끝나는지 확인하세요.

## 실행

```bash
build\Release\linesim.exe run scenarios/figure8_all.json
```

```bash
build\Release\linesim.exe run --track tracks/oval.json --drives 1 --set base_mps=2.2 --set steer_kd=400
```

| 옵션 | 설명 |
|---|---|
| `--drives 1,2,3,4` | 실행할 주행 순서. 한 번의 전원 세션처럼 이어서 실행되므로 2~4회차는 1회차 로그를 사용 |
| `--set key=value` | `driveData` 덮어쓰기: `base_mps max_mps accel decel steer_kp steer_kd pos_atten_gain pit_in_m target_shift fan_en` |
| `--robot file` | 로봇 물리 파라미터 (`robots/default.json`) |
| `--seed n` | 센서 편차·노이즈 시드. 같은 시드면 결과가 완전히 같음 |
| `--max-time s` | 회차당 제한 시간. 넘으면 로봇을 "들어 올려" 펌웨어가 Line Lost로 끝내게 함 |
| `--out dir` | 결과 폴더 (기본 `runs/<날짜_시간>`) |

환경변수 `LINESIM_PROFILE=1`이면 회차별 실행 시간과 이벤트 수를 출력합니다.

### 결과 (`runs/<...>/`)

| 파일 | 내용 |
|---|---|
| `summary.json` | 회차별 결과(완주/Line Lost/시간초과), 펌웨어 LCD에 표시된 랩타임, 최대 속도, 라인 추종 오차, 마커 순서, LCD 화면 |
| `trajectory.svg` | 트랙과 회차별 주행 궤적(속도 색상) |
| `driveN.svg` | 속도(목표/램프/실제), 센서바 횡오차, 모터 iq |
| `driveN.csv` | 1 kHz 텔레메트리: 실제 자세·속도·횡오차 + 펌웨어 내부 변수(`g_current_base_mps`, `target_pos`, `omega`, `target_Iq`, …) |
| `driveN_markers.csv` | 펌웨어 `g_cross_log` (마커 종류, 직전 마커 거리, yaw) |
| `sdcard/` | 가상 SD 카드. 펌웨어가 직접 쓴 `Drive_Data/save_slot_N.txt`, `Sensor_Data/calibration_result.txt` |
| `robot.xml` | 생성된 MuJoCo 모델 (MuJoCo `simulate`로 열어볼 수 있음) |

## 펌웨어를 수정 없이 돌리는 방법

- **HAL shim** (`sim/fw_shim`): `stm32h7xx_hal.h`, `arm_math.h`, `ff.h`, LCD 헤더를 대체합니다. 레지스터는 메모리 구조체이고, `TIMx->CCR`, `LPTIMx->CNT`, `ADCx->JDR`, `GPIOx->BSRR`, `DWT->CYCCNT`를 시뮬레이터가 인터럽트 전후로 채우거나 읽습니다. 원래 `main.h`를 그대로 include하고, IR MUX 핀만 별도 포트로 옮깁니다(같은 포트에 연속으로 쓰는 BSRR 값을 구분하기 위해서).
- **메인 루프와 인터럽트**: 펌웨어 `main`은 Windows Fiber에서 돌아갑니다. `while` 조건을 확인할 때마다 시뮬레이터에 제어를 넘기는 매크로(`sim_fw_prelude.h`)를 강제 include하므로, `while (g_current_base_mps > 0.001f) {}` 같은 대기 루프도 그대로 동작합니다. 인터럽트는 MCU처럼 메인 루프 사이에 끼어들어 끝까지 실행됩니다.
- **MSVC 호환**: GCC만 허용하는 `default:` 바로 뒤 `}`(motor.c)는 빌드할 때 복사본에서만 고칩니다. `firmware/` 원본은 건드리지 않습니다.
- **SD 카드**: FatFs API를 `runs/.../sdcard/` 폴더에 연결합니다. 펌웨어의 `Sensor_Save_Calibration()`, `Save_MarkerLog_To_SD()`가 실제 파일을 씁니다.
- **센서 캘리브레이션**: 시뮬레이터가 센서 모델에서 white/black max 값을 계산해 펌웨어 변수에 넣고, 펌웨어 함수 `Sensor_Save_Calibration()`으로 저장합니다.
- **운영자**: 주행이 끝나면(TIM14 정지) K 버튼을 길게 눌러 세이브 슬롯 선택과 종료 대기를 넘깁니다. 각 회차 전에는 로봇을 출발선 뒤에 다시 놓습니다.

## 모델과 가정

| 항목 | 현재 모델 | 실제와 맞추려면 |
|---|---|---|
| 차체 | Full.step 형상, 질량·관성은 추정값 | 무게·균형점 실측 |
| 바퀴·노면 | 바퀴 4개(같은 쪽 기어 연동). 타이어는 쇼어 A30 우레탄 브러시 모델(접지면 변형·이완·마찰 한계): μ 1.3(실제 로그 46조합 중 41개 재현), 슬립 강성 8, 이완 거리 4 mm | 타이어 기울기 시험으로 μ 실측 |
| 조향 지연 | `sensor_delay` 7 ms: 센서가 7 ms 전의 바닥을 봄(모델에 없는 조향 경로 지연을 한 덩어리로 넣은 값). 2.0 m/s, kd 0에서 kp 33은 직선에서 계속 흔들리고(Vibe Test 자이로 로그, 약 10 Hz) kp 16.4는 흔들리지 않는 실제 결과를 재현(`tools/oscillation.py`). 바퀴 토크 지연(`motor.drive_lag`)으로는 펌웨어 속도 루프가 kp와 상관없이 흔들려서 쓰지 않음(0) | 다른 속도·kp의 Vibe Test 로그 |
| 모터 | 전류 루프는 이상화(1차 지연 0.13 ms, 역기전력에 따른 전압 한계 0.4·Vbus), Kt·R·J·기어비는 `foc.h`, `motor.h` 값 | 무부하 전류(마찰), 기어 효율 측정 |
| 속도 루프 | **펌웨어 `FOC_Speed_Loop` 그대로** (2 kHz, 엔코더 2048 CPR 양자화 포함) | — |
| FOC 전류 루프 | 실행 안 함 (`FOC_Execute_Loop`, 25 kHz) | 다음 단계: 3상 전기 모델 + JDR 에뮬레이션 |
| IR 센서 | CAD 위치(원호), 가우시안 스폿 σ=2.5 mm, 12비트, 센서별 게인 ±6 %, 노이즈 6 LSB, 높이 변화 무시 | 실제 raw 값 로그(`Sensor Raw` 메뉴)로 white/black/노이즈 보정 |
| IMU | LSM6DS3 레지스터 에뮬레이션, 104 Hz ODR 평균, 2000 dps 스케일, 바이어스 ±0.4 dps | — |
| 흡입팬 | `fan_force × duty`의 하방력 (기본 0 N) | 팬 duty별 하방력 측정 |
| 트랙 | ZETIN 규정 (R 25 cm, 2 cm 라인, 2×5 cm 마크, 수직 교차) | 실제 대회 맵 좌표 |

## 트랙 파일

[ZETIN 전국 라인 트레이서 대회 규정](https://zetin.uos.ac.kr/contest_limit) (2022-08-26)을 기본값으로 합니다.

| 규정 | 트랙 파일에서 |
|---|---|
| 주행선 폭 2 cm, 검은 바탕 흰 선, 직선·원호의 폐곡선 | `line_width` 0.02 (기본), `closed` |
| 원호 곡률반경 항상 25 cm, 중심각 45° 단위 45~270° | `{"arc": 각도}` (R 기본 0.25, 어기면 경고) |
| 마크: 주행선에 수직, 폭 2 cm × 길이 5 cm, 주행선 가장자리에서 4 cm | `mark` = `{gap 0.04, along 0.02, across 0.05}` (기본) |
| 턴마크: 곡선 시작점에 회전 방향 쪽, 곡선이 끝나고 직선이 시작되는 곳에 마지막 회전 방향 쪽, 방향이 연속으로 바뀌면 바뀔 때마다 | `auto_turn_marks` (기본 true) |
| 엔드마크(좌+우): 출발 지점과 도착 지점 | `start_s`, `goal_s` (필수) |
| 교차로: 주행선끼리 수직 교차, 중심에서 20 cm 이상 직선 | 코스가 실제로 자기 자신과 겹치게 설계 (별도 표시 없음) |

```jsonc
{
  "name": "figure8",
  "closed": true,
  "goal_s": 6.2633,          // 도착 엔드마크 (주행 방향으로 출발보다 앞)
  "start_s": 6.7633,         // 출발 엔드마크. 로봇은 둘 사이, 센서바가 출발선 8 cm 뒤에 놓임
  "segments": [ { "straight": 0.40 }, { "arc": 90 }, { "arc": -45 }, ... ]   // arc > 0: 좌회전
}
```

코스를 새로 만들 때는 닫힘, 교차각과 교차로 주변 직선 길이, 엔드마크 주변 직선 길이를 먼저 확인하세요.

```bash
python tools/course.py tracks/figure8.json
```

## 알려진 사항 (펌웨어)

- SD 카드에 `Drive_Data` 폴더가 없으면 `Delete_All_Marker_Logs()`가 `FR_NO_PATH`를 받아 "Del Fail"을 띄우고 500 ms씩 10번 대기합니다(부팅 5초 지연). 시뮬레이터는 폴더를 미리 만들어 둡니다.
- `figure8` 코스를 1.0 m/s로 달리면 펌웨어가 기록하는 마커 순서가 규정상 정답과 정확히 같습니다. 1.8 m/s에서는 R 25 cm 커브의 원심가속도가 1.3 g라서, 기본 타이어 μ=1.3이면 차체가 미끄러지며 횡오차가 50 mm 안팎까지 커지고, 곡선 끝 턴마크를 일부 놓칩니다. 실제 타이어 마찰을 측정해 `tire_mu`를 맞춰야 판단할 수 있습니다.
- 대회 규정은 흡착 팬을 금지합니다. 펌웨어의 `fan_en`은 기본 0입니다.
