std_msgs/Header header
string severity                 # INFO / WARN / CRITICAL
bool motion_allowed
float64 speed_scale             # 1.0 = normal, 0.3 = warn reduced, 0.0 = stopped
string active_stop_source
string reason
string[] active_sources
