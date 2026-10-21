std_msgs/Header header
string current_comparator
uint32 queue_size
uint32 queue_capacity
uint32 active_missions_count
uint32 paused_missions_count
bool deadlock_detected
string active_mission_id
string[] active_mission_ids
VehicleLinkState[] vehicle_links   # 受管车辆链路与调度状态（结构化，替代平行数组）
