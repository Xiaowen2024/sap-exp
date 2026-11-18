#!/usr/bin/env bash
# Isolated runner bundle is prepared before launch; old analytic artifacts are untouched.
set -euo pipefail
audit=${GAUGE_REAL_AUDIT_ROOT:-/root/SIM-EXP/audits/gauge-real-317-400-20261110}
src="$audit/snapshot"
mkdir -p "$audit/results"
if [[ ! -e "$src/exp-results" ]]; then ln -s /root/SIM-EXP/exp-results "$src/exp-results"; fi
job=preflight
trap 'printf "{\"state\":\"failed_requires_inspection\",\"job\":\"%s\"}\n" "$job" > "$audit/results/state.json"' ERR
for version in 317 400; do
  py="/root/autodl-tmp/venvs/mjwarp${version}-cu12/bin/python"
  expected=3.17.0
  if [[ "$version" == 400 ]]; then expected=4.0.0; fi
  export WARP_CACHE_PATH="/root/autodl-tmp/warp-cache-gauge-real-$version"
  for task in cradle pendulum; do
    job="$version/$task"
    dir="$audit/results/$job"
    mkdir -p "$dir"
    manifest=archive/manifests/newtons_cradle_gauge_real_v1.json   # Newton's cradle: local archive only
    if [[ "$task" == pendulum ]]; then manifest=tasks/real-data-simple/gauge-pendulum/manifests/pendulum_gauge_real_v1.json; fi
    printf '{"state":"preflight","job":"%s"}\n' "$job" > "$audit/results/state.json"
    "$py" -u "$src/tasks/real-data-simple/gauge-pendulum/mjwarp/run_gauge_real_oscillations_mjwarp.py" --task "$task" \
      --manifest "$src/$manifest" --expected-version "$expected" --out "$dir" --smoke > "$dir/smoke.log" 2>&1
    printf '{"state":"running","job":"%s"}\n' "$job" > "$audit/results/state.json"
    "$py" -u "$src/tasks/real-data-simple/gauge-pendulum/mjwarp/run_gauge_real_oscillations_mjwarp.py" --task "$task" \
      --manifest "$src/$manifest" --expected-version "$expected" --out "$dir" > "$dir/run.log" 2>&1
    printf 'Completed %s\n' "$job"
  done
done
printf '{"state":"runs_finished_pending_audit_check_heldout_status","job":"all"}\n' > "$audit/results/state.json"
