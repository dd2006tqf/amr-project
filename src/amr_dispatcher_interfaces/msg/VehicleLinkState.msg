# 单台受管车辆的链路与调度状态（结构化消息，替代平行数组）
# 用于 Web 控制台展示与多车调度可观测性。
string vehicle_id              # 车辆 id（fleet.yaml robots[].id / VDA 5050 serialNumber）
string backend                 # 执行后端：local_sim | vda5050
bool online                    # 链路在线（VDA 5050 online 态 / 本地车心跳）
bool busy                      # 已被在途任务占用
bool quarantined               # 隔离态：人工强清后，不再参与自动分配，需显式恢复
bool position_valid            # 位置是否可信（来自新鲜定位状态）
string current_station_id      # 已知位置对应的站点（可空）
float64 battery_percent        # 电量百分比
int64 state_age_ms             # 状态时延（距上次收到状态报文的时间）
string last_error              # 最近一次错误原因（可空）
