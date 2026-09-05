std_msgs/Header header
string backend_name
string tier                     # UNKNOWN / HEALTHY / DEGRADED / CRITICAL
uint64 frames_total
uint64 frames_success
uint64 frames_lost
uint64 frames_crc_error
float64 loss_rate
float64 avg_latency_ms
float64 max_latency_ms
bool is_healthy
