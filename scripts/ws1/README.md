# ws1에서 LineSim 1차 주행 돌리기

ws1(Ubuntu)은 LineSim을 네이티브로 못 돌린다 — `CMakeLists.txt`가 Windows가 아니면
`FATAL_ERROR`를 내고(Windows Fibers, 고정 주소 flash 매핑), MuJoCo도 `mujoco.dll`뿐이며,
GPU 경로(`sim/gpu/gpu_sim.cpp`)는 DirectX 11 컴퓨트 셰이더다.
그래서 **MinGW 크로스컴파일 + Wine**으로 돌린다. (저장소의 `scripts/build_mingw.sh`,
`linesim` 래퍼가 원래 그 용도다.)

> **이 Wine 경로는 ws1 실기에서 아직 검증하지 않았다.** 아래 숫자 중 "실측"이라고
> 적은 것은 Windows PC에서 잰 값이다.

## 순서

```bash
# 1) 내 PC에서 ws1으로 복사 (git clone 가능하면 그쪽이 낫다)
scp -r LineSim ws1:~/

# 2) ws1 로그인 셸에서 한 번만 — 패키지 설치, Wine 프리픽스, 빌드, 스모크 테스트
ssh ws1
cd ~/LineSim && bash scripts/ws1/setup.sh

# 3) 1차 주행 병렬 (CPU, GPU 안 잡음)
sbatch scripts/ws1/first_run_array.sh              # 64회, 동시 8개
sbatch -a 0-199%16 scripts/ws1/first_run_array.sh  # 200회, 동시 16개

# 4) 결과 모으기
python3 scripts/ws1/collect.py ~/linesim-runs/<jobid> -o result.csv
```

GPU 배치(한 번의 dispatch로 `트랙수 × 후보수`를 동시에)를 쓰려면:

```bash
sbatch scripts/ws1/gpu_batch.sh
CANDIDATES=4096 MAPS=8 sbatch scripts/ws1/gpu_batch.sh
```

## 자원 점유

| | CPU 배열 잡 | GPU 배치 |
|---|---|---|
| Slurm 요청 | `-c 1 --mem=2G`, `--gres` 없음 | `--gres=shard:4 -c 4 --mem=8G` |
| 1회 비용 | 1코어 9.4초(54초 주행, 5.8× 실시간, 실측) | 1280 runs를 41.0초에(1130× 실시간, 실측) |
| 메모리 | 미측정 — `--mem=2G`로 잡아둠 | VRAM run당 약 1.2 KB → 1280개면 1.5 MB |
| 디스크 | 1회 **15 KB** (`--report light` 기본, 아래) | per-run 파일 0개, 셰이더 캐시 한 개뿐 |

- 노드 CPU는 20개, **Windows VM이 켜져 있으면 8개**로 준다(`-c`가 8 초과면 제출 거부).
  그래서 `-c 1`로 두고 `%N`으로 동시 개수를 조절한다.
- `shard:1~3`을 받아도 VRAM·연산은 안 나뉜다. 길게 돌릴 거면 `shard:4`로 한 장을
  통째로 잡고 빨리 반납하는 게 서로 낫다.
- GPU를 실제로 채우려면 **수천 runs**가 필요하다: 스레드 그룹이 64개 스레드라
  그룹 수 = `runs ÷ 64`다. 1280 runs = 20 그룹이면 NVIDIA SM 일부만 돈다.
- **후보를 늘리는 건 거의 공짜다.** 같은 PC에서 실측: 8 runs 24.9초 vs 1280 runs 41.0초 —
  작업량 160배에 시간 1.6배다. 벽시계 시간을 정하는 건 runs 수가 아니라 **dispatch 횟수**이고,
  이건 한 번에 50 ms씩 전진하니 `주행시간 ÷ 0.05초`로 고정된다(55초 주행 ≈ 1100회).
  그러니 `--candidates`는 아끼지 말고 크게 준다.

## 디스크 (실측, championship 트랙 · `--drives 1` · 54초 주행)

`linesim run` 에 **`--report`** 와 **`--csv-hz`** 를 넣었고, SVG 생성도 손봤다. 1회 출력량:

| 설정 | 용량 | 종전 | |
|---|---|---|---|
| `--report full` (기본값) | 9,299 KB | 10,388 KB | SVG 1,530 → 441 KB |
| `--report full --csv-hz 100` | 1,336 KB | 2,425 KB | |
| `--report full --csv-hz 20` | **648 KB** | 1,737 KB | 원래 `full` 대비 16× |
| **`--report light`** | **14.9 KB** | — | `summary.json` + 마커 로그 (**기본값, 698×**) |
| `--report none` | 11.2 KB | — | `summary.json` 만 |

`first_run_array.sh` 는 `REPORT=light` 가 기본이라 **무거운 파일을 애초에 안 쓴다** —
썼다가 지우는 게 아니라서 공용 디스크에 쓰기 자체가 안 일어난다. 그래서 동시 실행 개수와
무관하게 피크 사용량이 `횟수 × 15 KB`다. 200회 = 3 MB, 10,000회 = 150 MB.

플롯이 필요하면 `REPORT=full sbatch ...`, 절충안은 `REPORT=full CSV_HZ=100 sbatch ...` 다.
`--csv-hz` 는 시간 기준으로 솎아내되 **마커가 바뀌거나 line-lost 가 토글되는 행은 항상 남긴다** —
실측으로 44,974행 → 4,467행이 돼도 마커 전이 154개가 그대로였다.

SVG 쪽에서 줄인 것(그리는 내용은 그대로다):

- 같은 색 궤적 구간을 선분마다 `<line>` 하나씩이 아니라 `<polyline>` 하나로 묶고, 공통 속성은 `<g>` 로 뺐다.
- 좌표를 1 mm(궤적)·0.1 px(그래프) 자리에서 끊었다. 화면 배율이 138 px/m 이라 한 픽셀보다 훨씬 작다.
- 속도 색을 32단계로 양자화해 인접 구간이 같은 색을 공유하게 했다.
- 트랙 호 분할을 고정 5 mm 에서 화면 1.5 px 기준으로 바꿨다(현 배율에서 약 11 mm, 코드 오차 50 µm).
- 그래프 점을 가로 픽셀당 하나로 맞췄다.
- 덤으로 **`drive*.svg` 의 `&` 이스케이프 누락을 고쳤다** — 그동안 XML 파서에서 깨져서
  브라우저가 `index.html` 안의 그래프를 못 띄웠다(종전 파일도 똑같이 깨진 걸 확인했다).

`/home` 여유가 `MIN_FREE_GIB`(기본 20 GiB) 밑이면 잡이 스스로 거부한다.

## 알아둘 것

- **Wine 프리픽스는 하나를 공유한다**(`~/.wine-linesim`, `setup.sh`가 만든다).
  프리픽스 하나가 300 MB쯤이라 태스크마다 만들면 64개에 19 GB가 된다.
  동시 실행은 wineserver 하나가 처리하므로 공유해도 된다.
- **셰이더 컴파일**: Wine의 `d3dcompiler_47`은 `cs_5_0` HLSL 컴파일이 부실할 수 있다.
  `gpu_sim.cpp`가 바이트코드를 `build/gpu_cs_<hash>.cso`로 캐시하므로, Windows에서 만든
  `.cso`를 `scripts/ws1/shader-cache/`에 넣어뒀다 — `gpu_batch.sh`가 `build/`로 복사하면
  `D3DCompile`을 아예 안 탄다. (해시가 안 맞는 파일은 그냥 안 열리니 둘 다 넣어뒀다.)
- **DXVK는 Vulkan이라 `CUDA_VISIBLE_DEVICES`를 무시한다.** Slurm이 준 shard가 Vulkan
  디바이스를 고정해주지 않으므로, DXVK가 남의 잡이 도는 GPU를 잡을 수 있다. 두 GPU 모델명이
  다르면 `DXVK_FILTER_DEVICE_NAME`으로 고정하고, 같으면 관리자와 상의한다.
- GPU가 안 열리면 `D3D11CreateDevice failed (no DirectX 11 hardware GPU)`로 깔끔히 죽는다.
- 로그인 셸에서 GPU 작업을 직접 돌리지 않는다(가이드 §4). 빌드는 CPU만 쓰니 로그인 셸에서 해도 된다.
