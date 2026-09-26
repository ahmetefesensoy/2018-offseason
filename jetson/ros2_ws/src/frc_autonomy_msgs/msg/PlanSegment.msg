uint8 TASK_PICKUP=1
uint8 TASK_SCORE_SWITCH=2
uint8 TASK_SCORE_SCALE=3
uint8 TASK_DELIVER_VAULT=4
uint8 TASK_CROSS_LINE=5
uint8 TASK_PARK=6
uint8 TASK_CLIMB=7
uint8 TASK_WAIT=8

string segment_id
uint8 task_type
string target_id
int64 earliest_start_us
int64 latest_start_us
int64 expected_duration_us
float64 corridor_radius_m
string fallback_segment_id
TimedPose[] path
