# LIBERO-XE

LIBERO에서 로봇의 팔과 그리퍼를 다른 robosuite 로봇으로 바꿔, Panda로 학습한 정책(예: openpi `pi05_libero`)의 cross-embodiment 성능을 평가합니다. 장면과 task는 그대로 두고 로봇만 바꿉니다.

- 팔: `Panda`, `UR5e`, `IIWA`, `Kinova3`, `Jaco`, `Sawyer`
- 그리퍼: `default`(로봇 기본 그리퍼), `PandaGripper`, `RethinkGripper`, `Robotiq85Gripper`, `Robotiq140Gripper`, `RobotiqThreeFingerGripper`, `JacoThreeFingerGripper`

## 설치

openpi의 LIBERO 평가 환경이 있다는 전제입니다. 없다면 [openpi LIBERO 예제](https://github.com/Physical-Intelligence/openpi/tree/main/examples/libero)와 [LIBERO](https://github.com/Lifelong-Robot-Learning/LIBERO)를 참고해 먼저 설치하세요.

그 환경에서 이 저장소만 받아 경로에 추가하면 됩니다.
```bash
git clone https://github.com/yela318/LIBERO-XE.git
export PYTHONPATH=$PYTHONPATH:$PWD/LIBERO-XE
pip install h5py   # replay에만 필요
```

## 실행

**1. 정책 서버** (터미널 1, openpi 폴더)
```bash
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi05_libero --policy.dir=gs://openpi-assets/checkpoints/pi05_libero
```

**2. 평가** (터미널 2, LIBERO 환경에서)
```bash
python LIBERO-XE/eval_libero_xe.py --robot UR5e --gripper default --task-suite-name libero_object
```
화면이 없는 서버에서는 앞에 `MUJOCO_GL=egl`을 붙입니다.

| 옵션 | 설명 | 기본값 |
|---|---|---|
| `--robot` | 팔 | `Panda` |
| `--gripper` | 그리퍼 (`default` = 로봇 기본 그리퍼) | `default` |
| `--task-suite-name` | `libero_spatial`, `libero_object`, `libero_goal`, `libero_10` | `libero_spatial` |
| `--num-trials-per-task` | task당 회차 (LIBERO 공식 초기 상태 0번부터) | `50` |
| `--task-ids` | 일부 task만 (예: `--task-ids 0 1 2`) | 전체 |
| `--videos-per-task` | task당 저장할 영상 수. 각 task의 앞쪽 N회차를 저장 (0 = 저장 안 함) | `0` |
| `--out` | 결과 폴더 | `data/libero_xe` |
| `--host`, `--port` | 정책 서버 주소 | `127.0.0.1`, `8000` |

결과는 `<out>/<suite>/<로봇>_<그리퍼>/`에 `episodes.jsonl`(회차별 기록), `summary.json`(성공률), `videos/`로 저장됩니다.

**기준 설정 한 번에 돌리기**

아래 6개 설정 × 4개 suite를 차례로 평가하고, 끝나면 요약을 출력합니다(정책 서버는 켜 둔 상태).
```bash
bash LIBERO-XE/run_benchmark.sh
```
| 설정 | 역할 |
|---|---|
| Panda + 기본 그리퍼 | 기준 (학습한 로봇) |
| UR5e + 기본 그리퍼(Robotiq 85) | 주 설정: 팔 + 그리퍼 교체 |
| Panda + Robotiq 85 | 그리퍼만 교체 |
| Jaco + Panda 그리퍼 | 팔만 교체 |
| IIWA + 기본 그리퍼(Robotiq 140) | 팔 + 그리퍼 교체 |
| Kinova3 + 기본 그리퍼(Robotiq 85) | 팔 + 그리퍼 교체, 주 설정과 같은 그리퍼의 다른 로봇 |

명령 앞에 환경 변수를 붙여 설정을 바꿀 수 있습니다.

| 변수 | 기능 | 기본값 |
|---|---|---|
| `TRIALS` | task당 평가 회차. LIBERO 공식 초기 상태 0번부터 `TRIALS`개를 씁니다. suite당 회차는 10 task × `TRIALS`입니다. 빠르게 확인할 때는 줄이면 됩니다. | `50` (LIBERO 공식) |
| `SUITES` | 평가할 suite. 띄어쓰기로 여러 개 지정합니다. | `"libero_spatial libero_object libero_goal libero_10"` |
| `SETTINGS` | 평가할 `로봇:그리퍼` 조합. 띄어쓰기로 여러 개 지정합니다. 그리퍼 `default`는 로봇 기본 그리퍼입니다. 위 표 외의 조합도 지정할 수 있습니다. | 위 표의 6개 |
| `P` | 동시에 돌릴 평가 수. 모두 같은 정책 서버에 붙습니다. 늘리면 빨라지지만 평가마다 시뮬레이터 하나(RAM, 렌더링용 GPU)를 씁니다. | `1` |
| `OUT` | 결과 폴더. 실행별 로그는 `OUT/logs/`에 저장됩니다. 같은 폴더에 다시 돌리면 결과가 이어서 쌓이고, 요약에서는 같은 회차를 한 번만 셉니다. | `data/libero_xe` |
| `HOST`, `PORT` | 정책 서버 주소. 서버를 다른 포트나 다른 머신에서 띄웠을 때 바꿉니다. | `127.0.0.1`, `8000` |
| `VIDEOS` | task당 저장할 영상 수입니다. 각 task의 앞쪽 N회차를 저장합니다(예: `1`이면 task마다 1개, suite당 10개). `0`이면 저장하지 않습니다. | `0` |
| `PYTHON` | 평가에 쓸 Python. LIBERO 환경을 활성화하지 않고 경로로 지정할 때 씁니다. | `python` |

예시:
```bash
# 빠른 확인: task당 5회, Object/Goal만, 3개씩 동시에
TRIALS=5 P=3 SUITES="libero_object libero_goal" bash LIBERO-XE/run_benchmark.sh

# 주 설정만 공식 기준으로, task마다 영상 1개 저장
SETTINGS="UR5e:default" VIDEOS=1 bash LIBERO-XE/run_benchmark.sh

# 화면 없는 서버, 정책 서버가 8001번 포트일 때
MUJOCO_GL=egl PORT=8001 bash LIBERO-XE/run_benchmark.sh
```

**3. 결과 요약**
```bash
python LIBERO-XE/summarize_libero.py data/libero_xe
```
suite별로 설정마다 성공률, task별 성공률, 팔 × 그리퍼 표를 출력합니다.

## Replay (선택)

정책 없이 LIBERO의 Panda 시연을 다른 로봇에서 그대로 재생합니다. 해당 로봇이 Panda 동작을 따라 할 수 있는지 확인할 때 씁니다. 화면을 그리지 않아 CPU로 돕니다.

LIBERO 시연 데이터가 필요합니다. `python -c "from libero.libero import get_libero_path; print(get_libero_path('datasets'))"`로 나오는 폴더에 받습니다.
```bash
huggingface-cli download yifengzhu-hf/LIBERO-datasets --repo-type dataset \
    --include "libero_object/*" --local-dir <위 datasets 폴더>
```
```bash
python LIBERO-XE/replay_demos.py --suite libero_object --robots UR5e --grippers default --demos 10
python LIBERO-XE/summarize_replay.py data/libero_xe_replay
```

## 평가 방식

openpi의 LIBERO 평가(`examples/libero/main.py`)와 같은 프로토콜(초기 상태, 대기 10스텝, 이미지 처리, 5스텝마다 재계획, suite별 최대 스텝, seed 7)에 로봇 교체만 더했습니다.

- 다른 로봇은 LIBERO의 Panda와 같은 위치와 받침대에 배치합니다.
- LIBERO 액션은 그리퍼 위치의 월드 좌표 이동량이라, 모든 팔에서 같은 액션이 같은 그리퍼 움직임이 됩니다.
- 시작 장면은 Panda용 초기 상태에서 물체 상태만 옮겨 오고, 시작 그리퍼 자세는 IK로 Panda와 맞춥니다.

## 참고 결과

`pi05_libero` zero-shot 성공률 (task당 20회, Spatial / Object / Goal / LIBERO-10):

| 설정 | 성공률 |
|---|---|
| Panda | 0.99 / 0.98 / 0.97 / 0.94 |
| UR5e + 기본 그리퍼(Robotiq 85) | 0.23 / 0.77 / 0.39 / 0.34 |
| Panda + Robotiq 85 | 0.27 / 0.75 / 0.38 / 0.34 |
| Jaco + Panda 그리퍼 | 0.83 / 0.77 / 0.70 / 0.39 |
| IIWA + 기본 그리퍼(Robotiq 140) | 0.01 / 0.24 / 0.18 / 0.07 |
| Kinova3 + 기본 그리퍼(Robotiq 85) | 0.18 / 0.65 / 0.36 / 0.29 |
