# Telemetry decoded from the Lite3 motion host RobotState frame (UDP code 2305).
# Scalar fields are passed through unchanged from the motion host.

float64 battery_level    # battery percentage reported by the motion host

# is_charging / error_state / task_state are decoded and republished, but the
# motion host currently leaves them unpopulated ("not opened temporarily" in
# the vendor protocol header), so expect 0/false until firmware enables them.
bool is_charging         # charging flag reported by the motion host
uint32 error_state       # motion-host error code, 0 = none reported
int32 task_state         # task_state (reserved by the motion host)

int32 basic_state        # robot_basic_state
int32 gait_state         # robot_gait_state (gait selected by the motion host)
int32 motion_state       # robot_motion_state
bool zero_position_flag  # joint zero-calibration flag
bool is_robot_need_move  # push-to-move hint reported while standing
