string track_id

uint8 AFFILIATION_UNKNOWN=0
uint8 AFFILIATION_ALLY=1
uint8 AFFILIATION_OPPONENT=2
uint8 affiliation

geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance velocity
geometry_msgs/Vector3 size
PredictedPose[] predictions
float64 confidence
int64 last_seen_us
