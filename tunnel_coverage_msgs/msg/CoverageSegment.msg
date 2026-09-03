# CoverageSegment.msg
#
# One ordered plan segment.  Type: 0 = WORK (FollowPath), 1 = TRANSITION
# (NavigateToPose).  Coordinates are in the static map frame.

string id
string cell_id
int32 type          # 0 work, 1 transition

float64 start_x
float64 start_y
float64 start_yaw
float64 end_x
float64 end_y
float64 end_yaw
