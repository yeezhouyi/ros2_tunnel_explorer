#!/bin/bash
# Day 6-8: full-chain run + dual-denominator coverage audit, one command.
# Usage (ROS env + merge-tree workspace sourced by caller, or run inside
# the installed workspace):
#   bash scripts/run_chain_audit.sh <run_dir> [masks_npz]
#
# Chain: coverage_simulation (static map + AMCL + Nav2 dwb + coverage
# executor) -> send_coverage_goal -> odom bag -> audit_b6_coverage.py
# (linear-repo dual-denominator audit: coverage_task + coverage_known_free).
# audit_b6_coverage.py reads plan_stats.json from the masks npz directory.
#
# Exit 0 = audit JSON produced.  Numeric verdicts are NOT gate-evaluated
# here (Day 6-8 reports mean/range; seal thresholding is Day 9/10).
set +u
RUN=${1:?usage: run_chain_audit.sh <run_dir> [masks_npz]}
MASKS=${2:-/home/zhouyi/b6_chain/plan/audit_masks.npz}
AUDIT_REPO=${AUDIT_REPO:-/home/zhouyi/ros2_ws/src/linear_mpc_controller}
mkdir -p "$RUN"

# ---- precise process cleanup -------------------------------------------
# Snapshot `ps` ONCE, then match with in-shell `case`.  Patterns never enter
# any process argv (no ps|grep pipeline, no pkill -f), so the wrapper shell
# and unrelated processes are never matched.  is_ancestor skips our own
# ancestry.  Same discipline as run_b6b_sandbox.sh full_cleanup.
self=$$
cleanup_once() {
  local snap p args
  snap=$(ps -eo pid=,args=)
  while read -r p args; do
    [ -z "${p:-}" ] && continue
    case "$args" in
      *"coverage_simulation.launch.py"* | *"tunnel_coverage_executor"* | \
      *"send_coverage_goal.py"* | *"ros2 bag record"* | *"gz sim"* | \
      *"gzserver"* | *"parameter_bridge"* | *"robot_state_publisher"* | \
      *"map_server"* | *"amcl"* | *"controller_server"* | *"planner_server"* | \
      *"bt_navigator"*) ;;
      *) continue ;;
    esac
    # ancestor walk of $self
    local a=$self hit=1
    while [ "$a" != "0" ] && [ -n "$a" ]; do
      [ "$a" = "$p" ] && { hit=0; break; }
      a=$(ps -o ppid= -p "$a" 2>/dev/null | tr -d ' ')
    done
    [ "$hit" = 0 ] && continue
    kill -9 "$p" 2>/dev/null || true
  done <<EOF
$snap
EOF
  sleep 2
}

source /opt/ros/jazzy/setup.bash
source /home/zhouyi/ros2_tunnel_explorer/install/setup.bash

echo "$(date +%T) chain run: $RUN"
cleanup_once

echo "$(date +%T) launching coverage_simulation (headless)..."
ros2 launch tunnel_explorer_bringup coverage_simulation.launch.py \
  headless:=True > "$RUN/launch.log" 2>&1 &
LPID=$!
sleep 30
echo "$(date +%T) recording odom bag..."
ros2 bag record /odom -o "$RUN/odom_bag" > "$RUN/bag.log" 2>&1 &
RPID=$!
sleep 4
if ! kill -0 $RPID 2>/dev/null; then
  echo "FATAL: recorder died"; cat "$RUN/bag.log"; kill -INT $LPID; exit 1
fi

echo "$(date +%T) sending coverage goal (max 2700 s, retry on not-READY)..."
GRC=99
for att in 1 2 3 4 5 6; do
  timeout --signal=INT 2820 ros2 run benchmark_tools send_coverage_goal.py \
    --output-dir "$RUN/goal_out" --max-seconds 2700 > "$RUN/goal.log" 2>&1
  GRC=$?
  if [ $GRC -eq 0 ] && grep -q "Terminal result" "$RUN/goal.log"; then
    echo "attempt $att: OK"; break
  fi
  echo "attempt $att failed (rc=$GRC): $(tail -1 "$RUN/goal.log")"
  [ $att -lt 6 ] && sleep 30
done
echo "$(date +%T) goal exit: $GRC"; tail -3 "$RUN/goal.log"

echo "$(date +%T) teardown: recorder FIRST, wait for clean exit"
kill -INT $RPID 2>/dev/null
for i in $(seq 1 30); do kill -0 $RPID 2>/dev/null || break; sleep 1; done
if kill -0 $RPID 2>/dev/null; then
  echo "WARN recorder stuck after 30s, SIGTERM"; kill -TERM $RPID 2>/dev/null; sleep 3
fi
kill -INT $LPID 2>/dev/null
sleep 5
cleanup_once

echo "$(date +%T) bag info:"
ros2 bag info "$RUN/odom_bag" 2>&1 | head -8

echo "$(date +%T) audit (dual denominator)..."
mkdir -p "$RUN/audit"
cd "$AUDIT_REPO" || { echo "FATAL: audit repo missing $AUDIT_REPO"; exit 1; }
python3 benchmark_tools/scripts/audit_b6_coverage.py \
  --track-bag "$RUN/odom_bag" \
  --masks-npz "$MASKS" \
  --outdir "$RUN/audit" > "$RUN/audit.log" 2>&1
ARC=$?
echo "$(date +%T) audit exit=$ARC"
tail -20 "$RUN/audit.log"
exit $ARC
