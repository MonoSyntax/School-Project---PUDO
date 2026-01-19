# Simplified Traffic State for Navigation (Traffic Lights + Stop Sign Only)
std_msgs/Header header

# Traffic Light State
uint8 LIGHT_UNKNOWN = 0
uint8 LIGHT_GREEN = 1
uint8 LIGHT_YELLOW = 2
uint8 LIGHT_RED = 3
uint8 traffic_light_state
float32 traffic_light_distance

# Stop Sign
bool stop_sign_detected
float32 stop_sign_distance

# Navigation Override State
uint8 NAV_OVERRIDE_NONE = 0
uint8 NAV_OVERRIDE_STOP = 1
uint8 nav_override_state
