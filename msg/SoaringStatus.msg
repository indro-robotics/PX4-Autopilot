# Thermal soaring state in Hold (fixed-wing only)

uint64 timestamp # time since system start (microseconds)

uint8 STATE_OFF = 0 # SOAR_ENABLE 0
uint8 STATE_INHIBIT = 1 # stock Hold
uint8 STATE_CLIMB = 2 # motor climb to SOAR_ALT_CUTOFF
uint8 STATE_GLIDE = 3 # motor-off glide on the Hold orbit
uint8 STATE_THERMAL = 4 # motor-off circling in the air mass

uint8 state

uint8 REASON_NONE = 0
uint8 REASON_MODE = 1
uint8 REASON_FAILSAFE = 2
uint8 REASON_AIRSPEED = 3
uint8 REASON_BATTERY = 4
uint8 REASON_HOME = 5
uint8 REASON_BAND = 6
uint8 REASON_RADIUS = 7
uint8 REASON_FLOOR = 8
uint8 REASON_WEAK_LIFT = 9
uint8 REASON_CEILING = 10
uint8 REASON_ESTIMATOR = 12
uint8 REASON_AIRSPEED_LOW = 13
uint8 REASON_OPERATOR = 14 # an end by OPERATOR, MODE, FAILSAFE, MOTOR, TERRAIN or the turn check holds until SOAR_ENABLE 0 then 1 or disarm
uint8 REASON_MOTOR = 15
uint8 REASON_WIND = 16
uint8 REASON_CONFIG = 17
uint8 REASON_TERRAIN = 18
uint8 REASON_HEIGHT_LOST = 19
uint8 REASON_DRIFT = 20

uint8 reason # inhibit reason in INHIBIT, else the reason of the last transition

float32 vario_te # [m/s] total-energy climb rate
float32 vario_netto # [m/s] air-mass vertical speed
float32 vario_netto_slow # [m/s] netto through the entry filter
float32 sink_polar # [m/s] polar sink at the present airspeed and bank
float32 climb_expected # [m/s] netto_slow less the thermalling sink
float32 climb_avg # [m/s] climb through the orbit filter, THERMAL only
float32 thermal_climb # [m/s] estimated climb on the thermalling circle, THERMAL only
float32 thermal_w # [m/s] estimated strength, THERMAL only
float32 thermal_r # [m] estimated radius, THERMAL only
float32 thermal_x # [m] estimated centre, local north, THERMAL only
float32 thermal_y # [m] estimated centre, local east, THERMAL only
float32[4] thermal_var # estimator variances of strength, radius, north, east
float32 loiter_radius # [m] commanded orbit radius
float32 alt_rel # [m] height above home
float32 alt_min # [m] SOAR_ALT_MIN
float32 alt_floor # [m] motor restart height: SOAR_ALT_MIN or the glide reach home, the higher
float32 alt_cut # [m] SOAR_ALT_CUTOFF
float32 alt_max # [m] effective ceiling
float32 dist_home # [m] horizontal distance from home
float32 aspd_excess # [m/s] pitot TAS less the GPS estimate over the last full turn
uint16 thermal_count # thermals entered
float32 height_gained # [m] height gained over all thermals
uint16 cut_count # motor cuts: entries to GLIDE from powered flight
float32 cut_pitch_step # [rad] step of the TECS pitch integrator at the last motor cut: the path angle change to the glide
