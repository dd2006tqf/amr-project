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

# ---- 派发计数器（累积，供 /metrics 与运维趋势）----
uint64 dispatch_total            # 成功派发的任务数
uint64 dispatch_rejected_total   # 选车永久拒绝（判负）的任务数
uint64 dispatch_deferred_total   # 预留被拒/留队等待的次数
uint64 lock_contention_total     # 路权预约失败次数（含时空窗）
uint64 event_total               # 已追加的调度事件总数
