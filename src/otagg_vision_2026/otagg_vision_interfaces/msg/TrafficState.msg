# Aggregated traffic state for navigation decisions
std_msgs/Header header

# Traffic Light State
uint8 LIGHT_UNKNOWN = 0
uint8 LIGHT_GREEN = 1
uint8 LIGHT_YELLOW = 2
uint8 LIGHT_RED = 3
uint8 traffic_light_state
float32 traffic_light_distance  # Distance to nearest traffic light (m)

# Stop Sign
bool stop_sign_detected
float32 stop_sign_distance

# Turn Restrictions
bool no_left_turn
bool no_right_turn
bool no_entry
bool road_closed

# Bus Stop
bool bus_stop_detected
float32 bus_stop_distance

# Speed Limit (0 = none detected)
uint8 speed_limit_kmh

# Navigation Override State
uint8 NAV_OVERRIDE_NONE = 0
uint8 NAV_OVERRIDE_STOP = 1    # Full stop required (red light, stop sign)
uint8 NAV_OVERRIDE_SLOW = 2    # Reduce speed (approaching sign)
uint8 NAV_OVERRIDE_PUDO = 3    # PUDO maneuver in progress
uint8 nav_override_state

# Persistence flags for one-shot behaviors
bool stop_sign_handled    # True after stop completed for current sign
bool bus_stop_handled     # True after PUDO completed for current stop
