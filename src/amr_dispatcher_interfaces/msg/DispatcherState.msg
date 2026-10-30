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

# ---- 运行时拓扑叠加层（相位 11：在线施工区/单向/临时站点）----
# 追加字段（rosidl 兼容）：网关据此渲染 /metrics 与告警详情，不必额外打服务。
# 叠加层本身不改变 map_hash —— 它是同一张地图上的"临时交通管制"。
bool overlay_active              # 是否有生效的叠加层
uint32 overlay_revision          # 叠加层版本号（每次成功提交 +1，0 = 从未提交）
uint32 blocked_edges             # 被叠加层禁用的边数
uint32 one_way_edges             # 被叠加层限制为单向的边数
