#!/usr/bin/env bash
# Copyright 2026 zhouyi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...
#
# Coverage benchmark runner (C0-C3 MVP).  Runs `runs` coverage tasks on one
# benchmark world through coverage_simulation.launch.py and stores every
# goal result JSON/Markdown plus logs in its own run directory.
#
# Usage:
#   ./scripts/run_coverage_benchmark.sh \
#       --world cleaning_room_rect.sdf \
#       --map <abs path>/cleaning_room_rect.yaml \
#       --controller dwb \
#       --runs 5 \
#       --output-dir ~/coverage_benchmarks/rect_dwb
#
# Prerequisites: built & sourced workspace (colcon), ROS2 Jazzy, Gazebo
# Harmonic, and the nav2_minimal_tb3_sim package installed.
set -euo pipefail

WORLD="cleaning_room_rect.sdf"
MAP=""
CONTROLLER="dwb"
RUNS=5
OUT="$HOME/coverage_benchmarks"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --world) WORLD="$2"; shift 2 ;;
    --map) MAP="$2"; shift 2 ;;
    --controller) CONTROLLER="$2"; shift 2 ;;
    --runs) RUNS="$2"; shift 2 ;;
    --output-dir) OUT="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 1 ;;
  esac
done

if [[ -z "$MAP" ]]; then
  echo "--map is required (path to the world's map YAML)" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
WORLD_PATH=""
# Resolve world file (bare name -> tunnel_worlds/worlds)
if [[ -f "$WORLD" ]]; then
  WORLD_PATH="$WORLD"
elif [[ -f "$REPO_DIR/tunnel_worlds/worlds/$WORLD" ]]; then
  WORLD_PATH="$REPO_DIR/tunnel_worlds/worlds/$WORLD"
else
  echo "world not found: $WORLD" >&2
  exit 1
fi

wait_for_ready() {
  local attempts=0
  while [[ $attempts -lt 90 ]]; do
    local phase
    phase=$(timeout 8 ros2 topic echo /coverage/status --once 2>/dev/null \
      | grep -oP '(?<=phase: )\d+' | head -1 || true)
    if [[ "$phase" == "4" ]]; then
      return 0
    fi
    attempts=$((attempts + 1))
    sleep 2
  done
  return 1
}

mkdir -p "$OUT"
for i in $(seq 1 "$RUNS"); do
  run_dir="$OUT/run_$(printf '%02d' "$i")"
  mkdir -p "$run_dir"
  echo "== run $i/$RUNS -> $run_dir =="

  # Clean start.
  (cd "$REPO_DIR" && ./scripts/cleanup_simulation.sh) >/dev/null 2>&1 || true

  ros2 launch tunnel_explorer_bringup coverage_simulation.launch.py \
    world:="$WORLD_PATH" \
    map:="$MAP" \
    headless:=True \
    rviz:=False \
    use_composition:=False > "$run_dir/launch.log" 2>&1 &
  LAUNCH_PID=$!

  if ! wait_for_ready; then
    echo "executor did not reach READY_IDLE in time (run $i)" >&2
    kill "$LAUNCH_PID" 2>/dev/null || true
    (cd "$REPO_DIR" && ./scripts/cleanup_simulation.sh) >/dev/null 2>&1 || true
    exit 1
  fi

  ros2 run benchmark_tools send_coverage_goal.py \
    --timeout 1500 --output-dir "$run_dir"

  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  (cd "$REPO_DIR" && ./scripts/cleanup_simulation.sh) >/dev/null 2>&1 || true
done

echo "== done: $RUNS runs under $OUT =="
