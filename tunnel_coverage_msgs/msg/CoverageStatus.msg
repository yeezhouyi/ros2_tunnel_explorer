# CoverageStatus.msg
#
# Online, structured status of the coverage executor (published on
# /coverage/status).  Phase and terminal-result values are shared with
# ExecuteCoverage.action — keep the two in sync.

# ── TaskPhase constants ───────────────────────────────────────────────────
# 0  BOOTSTRAP            node starting
# 1  WAIT_MAP             waiting for a frozen static map
# 2  WAIT_LOCALIZATION    map frozen, waiting for valid map -> base TF
# 3  WAIT_NAV2            waiting for Nav2 action servers
# 4  READY_IDLE           ready; no motion until an ExecuteCoverage goal
# 5  PLANNING             building masks + plan
# 6  TRANSITING           navigating to a work segment (NavigateToPose)
# 7  EXECUTING_SEGMENT    following a work segment (FollowPath)
# 8  RECOVERY             handling a failed/blocked attempt
# 9  REPLANNING           residual replanning (U6+)
# 10 RESIDUAL_CHECK       checking for uncovered residual area
# 11 CANCELLING           cancelling the single child goal + stop confirm
# 12 TERMINAL             task finished; result pending acknowledgement
int32 phase

string task_input_id
string plan_id
string map_digest

# Segment dispositions (SegmentDisposition enum):
#   0 PENDING  1 COVERED  2 BLOCKED_TEMP  3 EXEMPT  4 FAILED
int32 segments_total
int32 segments_covered
int32 segments_exempt
int32 segments_failed
int32 segments_pending
int32 segments_blocked_temp

# Live metrics (denominators documented in tunnel_coverage_planner).
float64 gross_coverage
float64 effective_coverage
float64 exempt_ratio
float64 repeat_ratio
float64 path_length_m
float64 last_update_seconds
