std_msgs/Header header
uint64 world_version
uint64 plan_revision
string goal_task_id

uint8 IDLE=0
uint8 PLANNING=1
uint8 TRACKING=2
uint8 SLOW=3
uint8 STOPPED=4
uint8 GOAL_REACHED=5
uint8 FAILED=6
uint8 state

TimedPose[] planned_path
float64 min_clearance_m
float64 speed_limit_mps
string obstacle_reason
bool goal_reached
