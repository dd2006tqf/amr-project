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

# T3：命令送达率（下行 0xF3 是否真的被 MCU 收到）。
# 此前链路质量只能统计接收侧现象（读空 / CRC 错），命令在串口上真的丢了测不出来。
# 语义：最近 command_sent 帧下行里，MCU 回显序号已确认送达的比例。
# -1 = 不适用（文本协议，或 MCU 尚未回显过序号）。
float64 command_delivery_rate
uint64 commands_sent
uint64 commands_delivered
