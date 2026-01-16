# Detected traffic sign message
std_msgs/Header header
string sign_class           # e.g., "red_light", "stop", "no_left_turn"
float32 confidence          # 0.0 to 1.0
int32 bbox_x                # Bounding box center X (pixels)
int32 bbox_y                # Bounding box center Y (pixels)
int32 bbox_width            # Bounding box width (pixels)
int32 bbox_height           # Bounding box height (pixels)
float32 estimated_distance  # Meters (estimated from bbox size)
