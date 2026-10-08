#!/usr/bin/env bash
# Evaluate a LIBERO policy on the benchmark settings (robot + gripper) x LIBERO suites, then print the summary.
# Needs a running policy server and the LIBERO Python environment (see README.md).
#   bash run_benchmark.sh
# Options (environment variables):
#   SETTINGS     robot:gripper pairs (default: the 6 benchmark settings below)
#   SUITES       LIBERO suites (default: all four)
#   TRIALS       episodes per task, LIBERO init states 0..TRIALS-1 (default 50, the LIBERO protocol)
#   P            evaluations run in parallel against the one policy server (default 1)
#   OUT          results dir (default data/libero_xe); per-run logs go to $OUT/logs
#   HOST, PORT   policy server (default 127.0.0.1, 8000)
#   VIDEOS       videos saved per task: the first N episodes (default 0 = none)
#   PYTHON       Python of the LIBERO environment (default python)
set -eu

# Benchmark settings ("default" = the robot's own gripper):
#   Panda:default              source robot (reference)
#   UR5e:default               main setting: arm + gripper swapped (Robotiq 85)
#   Panda:Robotiq85Gripper     gripper-only swap
#   Jaco:PandaGripper          arm-only swap
#   IIWA:default               arm + gripper swapped (Robotiq 140)
#   Kinova3:default            arm + gripper swapped (Robotiq 85), second robot for the main setting
SETTINGS=${SETTINGS:-"Panda:default UR5e:default Panda:Robotiq85Gripper Jaco:PandaGripper IIWA:default Kinova3:default"}
SUITES=${SUITES:-"libero_spatial libero_object libero_goal libero_10"}
export TRIALS=${TRIALS:-50}
P=${P:-1}
export OUT=${OUT:-data/libero_xe}
export HOST=${HOST:-127.0.0.1}
export PORT=${PORT:-8000}
export VIDEOS=${VIDEOS:-0}
export PYTHON=${PYTHON:-python}
export HERE=$(cd "$(dirname "$0")" && pwd)
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$HERE"
mkdir -p "$OUT/logs"

run_one() {
    local suite=$1 robot=$2 gripper=$3
    local log="$OUT/logs/${suite}_${robot}_${gripper}.log"
    if "$PYTHON" "$HERE/eval_libero_xe.py" --host "$HOST" --port "$PORT" --task-suite-name "$suite" \
        --robot "$robot" --gripper "$gripper" --num-trials-per-task "$TRIALS" --videos-per-task "$VIDEOS" \
        --out "$OUT" > "$log" 2>&1; then
        echo "$(date +%T) done   $suite $robot $gripper: $(grep -o '"success_rate": [0-9.]*' "$log" | tail -1)"
    else
        echo "$(date +%T) FAILED $suite $robot $gripper (see $log)"
    fi
}
export -f run_one

echo "=== $(date '+%F %T') $(echo $SETTINGS | wc -w) settings x [$SUITES], $TRIALS trials/task, $P in parallel -> $OUT"
for s in $SUITES; do
    for rg in $SETTINGS; do
        echo "$s ${rg%%:*} ${rg#*:}"
    done
done | xargs -P "$P" -L 1 bash -c 'run_one "$@"' _
echo "=== $(date '+%F %T') all done"
"$PYTHON" "$HERE/summarize_libero.py" "$OUT"
