# Fixed-wing terrain follower state and heights above ground

uint64 timestamp # time since system start (microseconds)

uint8 STATE_OFF = 0
uint8 STATE_STANDBY = 1 # enabled, the mode's own altitude
uint8 STATE_FOLLOW = 2 # commanded height above ground
uint8 STATE_UPLIFT = 3 # guard holds 1.5 x FW_TF_FLOOR
uint8 STATE_PULL_UP = 4 # climb at FW_T_CLMB_MAX and FW_AIRSPD_MIN
uint8 STATE_RANGE_LOST = 5 # climb to the planned altitude or FW_TF_LOSS_UP above the loss
uint8 STATE_FAULT = 6 # off until the next arming

uint8 state

float32 hagl_cmd # [m] commanded height above ground
float32 hagl_used # [m] height above ground the follower uses
float32 hagl_raw # [m] tilt-corrected range, NAN when invalid
float32 terrain_rate # [m/s] terrain altitude rate, positive rising
float32 height_rate_sp # [m/s] height rate setpoint, NAN when the altitude is set

bool range_valid # range usable for following
uint16 loss_count # range losses since arming
