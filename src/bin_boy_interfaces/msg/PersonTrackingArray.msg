# Array of tracked persons with metadata

std_msgs/Header header

# All detected persons with tracking info
PersonTracking[] persons

# Current tracking state (IDLE, TRACKING, LOST, REACQUIRING, LOCKED)
string tracking_state

# ID of current target person (0 if no target)
uint32 target_id
