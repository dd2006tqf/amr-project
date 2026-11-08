// 双发开销基准：量化「proto 侧让序列化次数翻倍」到底值多少 CPU。
//
// ============================================================================
// 为什么需要它
// ============================================================================
// PROTOBUF_MIGRATION_PLAN.md §6 阶段 2 的实施记录里留了一条如实说明：
//
//   「TrackingError 是 20Hz、SafetyState 是 10Hz+50Hz 兜底，双发意味着这两条
//     链路的序列化次数翻倍。目前**没有**做 CPU/带宽的基准对比。」
//
// 这是个悬着的决策依据：现场要不要保留双发，取决于成本到底有多大。没有数字
// 就只能一直「先开着」，§7 的保留意见也就永远无法收敛。
//
// ============================================================================
// 本工具测什么、不测什么（边界必须写清楚，否则数字会被误读）
// ============================================================================
// 测：
//   - 投影（rosidl -> proto）：双发**新增**的纯 CPU 成本，在本进程内可精确计时
//   - protobuf 编码：双发让编码跑两遍，这是新增的第二遍
//
// **不测**：rosidl 侧（Fast DDS）的序列化耗时。它在 publish() 内部、本进程
// 之外，测不到。因此下面报的「增量」是**双发相对单发的新增成本**，不是双发的
// 总成本——单发本身的开销（Fast DDS 编码 + 收发）在两种模式下都存在，被抵消了。
//
// **不测带宽**：那需要抓包或读 DDS 统计，跨进程噪声大且受 QoS/发现影响。
// 本工具只回答「CPU 增量」这一个问题。
//
// 刻意**不**输出「rosidl 近似字节数」：那只能靠按字段宽度累加去猜（uint64 恒 8
// 字节、bool 1 字节、字符串 4+len…），猜出来的数和真实 CDR 载荷能差出一倍
// （对齐填充、序列长度头、Header 嵌套）。给一个假数字比留空更糟，所以只报
// proto 侧字节数——那是真实 SerializeToString 的输出长度。
//
// ============================================================================
// 为什么每次迭代都新建 proto 对象
// ============================================================================
// 真实发布路径就是这么做的：ProtoDualPublisher::Publish() 在栈上构造一个
// ProtoT，投影后发布（proto_dual_publisher.hpp:160）。复用同一个对象会漏掉
// 构造/析构成本，更危险的是 repeated 字段会累积——DispatcherState 的
// vehicle_links 和 TopologyState 的四组字符串数组在第二次迭代后就会翻倍，
// 量出来的字节数是假的。
//
// 因此这里的时间包含 protobuf 对象构造与析构。那是真实成本的一部分。
//
// ============================================================================
// 为什么不做成 ctest
// ============================================================================
// 计时类断言在共享 CI 机器上必然抖动（同一份代码在冷/热缓存下能差数倍）。
// 做成测试的唯一结局：要么阈值宽到测不出回归，要么随机失败。因此它只输出
// 数字、由人读，不参与 pass/fail。
//
// 用法：
//   ros2 run amr_dispatcher_ros proto_dual_benchmark
//   ros2 run amr_dispatcher_ros proto_dual_benchmark -- --iterations 200000
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"
#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/msg/mission_event.hpp"
#include "amr_dispatcher_interfaces/msg/safety_state.hpp"
#include "amr_dispatcher_interfaces/msg/topology_state.hpp"
#include "amr_dispatcher_interfaces/msg/tracking_error.hpp"
#include "amr_dispatcher_interfaces/msg/vehicle_link_state.hpp"

#include "amr_dispatcher_ros/proto_dual_publisher.hpp"
#include "amr_dispatcher_ros/proto_projection.hpp"

namespace
{

using Clock = std::chrono::steady_clock;
using amr_dispatcher_interfaces::msg::ChassisLinkHealth;
using amr_dispatcher_interfaces::msg::DispatcherState;
using amr_dispatcher_interfaces::msg::MissionEvent;
using amr_dispatcher_interfaces::msg::SafetyState;
using amr_dispatcher_interfaces::msg::TopologyState;
using amr_dispatcher_interfaces::msg::TrackingError;
using amr_dispatcher_interfaces::msg::VehicleLinkState;

// ---------------------------------------------------------------------------
// 典型消息构造
// ---------------------------------------------------------------------------
// 填的是「稳态下的代表性内容」，不是极端值（超长字符串、上千条重复项）。测边界
// 没有意义——现场该不该保留双发，取决于稳态成本。
//
// 字段清单与 .msg 严格对齐；多写一个不存在的字段会编译失败（这是好事：说明
// 基准本身跟着消息演进了），少写一个则会被下面的「基线与实际一致」检查掩盖。
// 因此每条消息都填**全部**字段，重复字段也填成有代表性的规模。

ChassisLinkHealth MakeChassisLinkHealth()
{
  ChassisLinkHealth m;
  m.header.stamp.sec = 1762300000;
  m.header.stamp.nanosec = 123456789u;
  m.header.frame_id = "base_link";
  m.backend_name = "serial";
  m.tier = "HEALTHY";
  m.frames_total = 1234567u;
  m.frames_success = 1234000u;
  m.frames_lost = 567u;
  m.frames_crc_error = 3u;
  m.loss_rate = 0.000459;
  m.avg_latency_ms = 12.5;
  m.max_latency_ms = 88.0;
  m.is_healthy = true;
  // -1 是「不适用」哨兵，会走 valid=false 分支——这是最容易漏测的一处语义。
  m.command_delivery_rate = 0.98;
  m.commands_sent = 500000u;
  m.commands_delivered = 490000u;
  return m;
}

TrackingError MakeTrackingError()
{
  TrackingError m;
  m.header.stamp.sec = 1762300000;
  m.header.stamp.nanosec = 987654321u;
  m.header.frame_id = "map";
  m.controller_name = "pure_pursuit";
  m.lateral_error_m = 0.032;
  m.heading_error_rad = 0.011;
  m.lookahead_distance_m = 1.2;
  m.target_linear_x_mps = 0.8;
  m.target_angular_z_radps = 0.15;
  m.actual_linear_x_mps = 0.79;
  m.actual_angular_z_radps = 0.14;
  m.nearest_waypoint_index = 42;
  m.arrived = false;
  return m;
}

SafetyState MakeSafetyState()
{
  SafetyState m;
  m.header.stamp.sec = 1762300000;
  m.header.stamp.nanosec = 555555555u;
  m.header.frame_id = "";
  m.severity = "INFO";
  m.motion_allowed = true;
  m.speed_scale = 1.0;
  m.active_stop_source = "";
  m.reason = "idle";
  m.active_sources = {"watchdog", "chassis"};
  return m;
}

TopologyState MakeTopologyState()
{
  TopologyState m;
  m.stamp.sec = 1762300000;
  m.stamp.nanosec = 111111111u;
  // 四组平行数组，规模按现场拓扑量级取：十几条边、少量锁与死锁点。
  for (int i = 0; i < 16; ++i) {
    m.active_edges.push_back("E" + std::to_string(i));
  }
  for (int i = 0; i < 4; ++i) {
    m.locked_edges.push_back("E" + std::to_string(i));
    m.edge_owners.push_back("M_20261106_000" + std::to_string(i));
  }
  m.deadlocked_nodes = {"ST_3", "ST_7"};
  m.deadlock_detected = true;
  return m;
}

MissionEvent MakeMissionEvent()
{
  MissionEvent m;
  m.stamp.sec = 1762300000;
  m.stamp.nanosec = 222222222u;
  m.mission_id = "M_20261106_0001";
  m.order_id = "ORD_8842";
  m.event = "finished";
  m.from_state = "active";
  m.to_state = "done";
  m.reason = "reached goal";
  return m;
}

DispatcherState MakeDispatcherState()
{
  DispatcherState m;
  m.header.stamp.sec = 1762300000;
  m.header.stamp.nanosec = 333333333u;
  m.header.frame_id = "map";
  m.current_comparator = "priority_fifo";
  m.queue_size = 3;
  m.queue_capacity = 128;
  m.active_missions_count = 2;
  m.paused_missions_count = 0;
  m.deadlock_detected = false;
  m.active_mission_id = "M_20261106_0001";
  m.active_mission_ids = {"M_20261106_0001", "M_20261106_0002"};
  // 4 条车辆链路：接近多车现场规模，让嵌套 repeated message 的成本可见。
  for (int i = 0; i < 4; ++i) {
    VehicleLinkState v;
    v.vehicle_id = "amr-" + std::to_string(i);
    v.backend = "vda5050";
    v.online = true;
    v.busy = (i % 2 == 0);
    v.quarantined = false;
    v.position_valid = true;
    v.current_station_id = "ST_" + std::to_string(i);
    v.battery_percent = 80.0 - i;
    v.state_age_ms = 120 + i;
    v.last_error = "";
    m.vehicle_links.push_back(v);
  }
  m.dispatch_total = 18422u;
  m.dispatch_rejected_total = 37u;
  m.dispatch_deferred_total = 210u;
  m.lock_contention_total = 96u;
  m.event_total = 40118u;
  m.overlay_active = true;
  m.overlay_revision = 7u;
  // 注意：这两个在 DispatcherState 里是 **uint32 计数**，不是边 ID 数组
  // （数组在 TopologyState 里）。同名不同义，容易写错。
  m.blocked_edges = 2u;
  m.one_way_edges = 1u;
  return m;
}

// ---------------------------------------------------------------------------
// 计时
// ---------------------------------------------------------------------------

/// 计时结果。median / p95 都要：平均会被个别调度抖动带偏。
struct Stats
{
  double median_ns{0.0};
  double p95_ns{0.0};
};

Stats Summarize(std::vector<double> samples)
{
  if (samples.empty()) {
    return {};
  }
  std::sort(samples.begin(), samples.end());
  const std::size_t n = samples.size();
  const std::size_t mid = n / 2;
  Stats s;
  s.median_ns = (n % 2 == 1) ? samples[mid] : 0.5 * (samples[mid - 1] + samples[mid]);
  s.p95_ns = samples[std::min(n - 1, static_cast<std::size_t>(n * 0.95))];
  return s;
}

/// 测一个可调用对象单次执行的耗时分布。
///
/// 先 warmup：首次调用会触发热路径（protobuf 的内部字符串池、分配器预热等），
/// 算进去会得到「首次投影 30 微秒」这种对稳态毫无意义的结果。
template<typename Fn>
Stats TimeIt(Fn && fn, int iterations, int warmup = 2000)
{
  for (int i = 0; i < warmup; ++i) {
    fn();
  }
  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>(iterations));
  for (int i = 0; i < iterations; ++i) {
    const auto t0 = Clock::now();
    fn();
    const auto t1 = Clock::now();
    samples.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count());
  }
  return Summarize(std::move(samples));
}

/// 一条消息的完整测量结果。
///
/// 中位数和 p95 都保留：中位数代表稳态，p95 代表「实际会遇到的那一档」。
/// 只用中位数会低估长尾（GC、分配器扩容、页错误都会落在长尾里），而现场
/// 关心的恰恰是「最坏情况下这套系统还扛得住吗」。
///
/// p95 存的是**投影与编码各自的 p95 之和**，不是「合计耗时的 p95」。两者
/// 严格来说不是同一个量（需要把两次测量配对），但作为上界解读足够：任何一次
/// 真实发布的耗时都不会超过两次 p95 之和。不做配对是因为两次测量各自独立
/// 计时，强行配对本就无意义。
struct Row
{
  double project_ns{0.0};
  double encode_ns{0.0};
  double project_p95_ns{0.0};
  double encode_p95_ns{0.0};
  std::size_t proto_bytes{0};
  double rate_hz{0.0};

  double total_median_ns() const {return project_ns + encode_ns;}
  double total_p95_ns() const {return project_p95_ns + encode_p95_ns;}
};

/// 测一条消息：投影 + proto 编码。
///
/// 两个测量都用同样的「每次新建对象」模式（见文件头说明）。
template<typename RosidlT, typename ProtoT, typename ProjectFn>
Row Measure(const RosidlT & msg, int iterations, double rate_hz, ProjectFn project)
{
  // 1) 投影。与真实路径一致：每次都在栈上新建 ProtoT。
  const Stats proj = TimeIt(
    [&]() {
      ProtoT out;
      project(msg, &out);
    },
    iterations);

  // 2) protobuf 编码。复用 buffer：真实路径里 rclcpp 也复用序列化缓冲，
  //    每次新建 std::string 会把分配成本算进「编码」，虚高且不稳定。
  ProtoT projected;
  project(msg, &projected);
  std::string buffer;
  const Stats enc = TimeIt(
    [&]() {
      buffer.clear();
      projected.SerializeToString(&buffer);
    },
    iterations);

  return Row{proj.median_ns, enc.median_ns,
    proj.p95_ns, enc.p95_ns,
    buffer.size(), rate_hz};
}

}  // namespace

int main(int argc, char ** argv)
{
  int iterations = 100000;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--iterations" && i + 1 < argc) {
      iterations = std::atoi(argv[++i]);
      if (iterations <= 0) {
        std::fprintf(stderr, "--iterations 必须是正整数\n");
        return 2;
      }
    } else if (arg == "--help" || arg == "-h") {
      std::printf(
        "用法: proto_dual_benchmark [--iterations N]\n"
        "\n"
        "测量双发相对单发**新增**的 CPU 成本（投影 + protobuf 编码）。\n"
        "不测 rosidl 侧编码（在本进程外），不测带宽。不参与 pass/fail。\n"
        "同样的代码在冷/热缓存下能差数倍，请在同一台机器上做前后对比。\n");
      return 0;
    }
  }

  // 各话题的发布频率取**代码里的真实值**，不是估计：
  //   /chassis/link_health      chassis.yaml health_rate_hz = 5.0
  //   /tracking/error           path_tracker control_rate_hz = 20.0
  //   /safety/state             watchdog 100ms 空闲播报 + 每条 cmd_vel 一次
  //   /dispatcher/state         dispatcher_node state_timer_ 500ms
  //   /dispatcher/topology_state 与 state 同一出口，同频
  //   /dispatcher/events        事件驱动，按低频估 1Hz
  //
  // SafetyState 的实际频率不是固定值：50Hz 的 publish_timer_ 只发 cmd_vel，
  // 不碰 /safety/state；/safety/state 由 WatchdogTick(10Hz 空闲播报) 与
  // RawCmdVelCallback(每条上游指令) 触发。取 10Hz 作为**下限**，若上游 20Hz
  // 控制则实际是 30Hz —— 这一点在解读时要记住，不能当成精确值。
  struct Case
  {
    const char * label;
    const char * topic;
    double rate_hz;
    Row row;
  };

  std::vector<Case> cases;
  cases.push_back(Case{
    "ChassisLinkHealth", "/chassis/link_health", 5.0,
    Measure<ChassisLinkHealth, amr_dispatcher::proto::ChassisLinkHealth>(MakeChassisLinkHealth(), iterations, 5.0,
      [](const ChassisLinkHealth & in, amr_dispatcher::proto::ChassisLinkHealth * out) {
        amr_dispatcher_ros::proto_projection::ProjectChassisLinkHealth(in, out);
      })});
  cases.push_back(Case{
    "TrackingError", "/tracking/error", 20.0,
    Measure<TrackingError, amr_dispatcher::proto::TrackingError>(MakeTrackingError(), iterations, 20.0,
      [](const TrackingError & in, amr_dispatcher::proto::TrackingError * out) {
        amr_dispatcher_ros::proto_projection::ProjectTrackingError(in, out);
      })});
  cases.push_back(Case{
    "SafetyState", "/safety/state", 10.0,
    Measure<SafetyState, amr_dispatcher::proto::SafetyState>(MakeSafetyState(), iterations, 10.0,
      [](const SafetyState & in, amr_dispatcher::proto::SafetyState * out) {
        amr_dispatcher_ros::proto_projection::ProjectSafetyState(in, out);
      })});
  cases.push_back(Case{
    "TopologyState", "/dispatcher/topology_state", 2.0,
    Measure<TopologyState, amr_dispatcher::proto::TopologyState>(MakeTopologyState(), iterations, 2.0,
      [](const TopologyState & in, amr_dispatcher::proto::TopologyState * out) {
        amr_dispatcher_ros::proto_projection::ProjectTopologyState(in, out);
      })});
  cases.push_back(Case{
    "MissionEvent", "/dispatcher/events", 1.0,
    Measure<MissionEvent, amr_dispatcher::proto::MissionEvent>(MakeMissionEvent(), iterations, 1.0,
      [](const MissionEvent & in, amr_dispatcher::proto::MissionEvent * out) {
        amr_dispatcher_ros::proto_projection::ProjectMissionEvent(in, out);
      })});
  cases.push_back(Case{
    "DispatcherState", "/dispatcher/state", 2.0,
    Measure<DispatcherState, amr_dispatcher::proto::DispatcherState>(MakeDispatcherState(), iterations, 2.0,
      [](const DispatcherState & in, amr_dispatcher::proto::DispatcherState * out) {
        amr_dispatcher_ros::proto_projection::ProjectDispatcherState(in, out);
      })});

  std::printf("双发新增开销基准（iterations=%d，单位 ns，取中位数/p95）\n", iterations);
  std::printf("只含投影 + protobuf 编码；rosidl 侧编码在进程外，两边都有，被抵消\n");
  std::printf("%s\n", std::string(96, '-').c_str());
  std::printf("%-19s %7s %10s %10s %8s %10s %11s\n",
    "消息", "频率Hz", "投影med", "编码med", "合计p95", "proto字节", "每秒CPU");
  std::printf("%s\n", std::string(96, '-').c_str());

  double total_us_per_sec = 0.0;
  for (const auto & c : cases) {
    const double us_per_sec = c.row.total_median_ns() * c.rate_hz / 1000.0;
    total_us_per_sec += us_per_sec;
    std::printf("%-19s %7.0f %10.0f %10.0f %8.0f %10zu %8.1f us/s\n",
      c.label, c.rate_hz,
      c.row.project_ns, c.row.encode_ns,
      c.row.total_p95_ns(),
      c.row.proto_bytes,
      us_per_sec);
  }
  std::printf("%s\n", std::string(96, '-').c_str());

  // 占比换算：一秒 = 1e6 微秒，所以「微秒/秒」直接除以 1e6 就是单核占用比例。
  // 分母写 1e6 而不是 10000：曾把这里写成 /10000.0，等于把占比虚放大了 100 倍，
  // 而注释还写着「1000ns」——数字和注释两头都错，看图的人不会去验算。
  // 结论只对单核有意义：多核机器上这部分开销可被其他核吸收。
  std::printf("合计新增：%.2f 微秒CPU/秒  ≈ 单核的 %.5f%%\n",
    total_us_per_sec, total_us_per_sec / 1e6 * 100.0);
  std::printf(
    "解读：投影与编码都是**额外**成本，单发时不存在。上表合计即双发相对单发的\n"
    "      净增量。若要评估带宽增量，看 proto字节 × 频率；rosidl 侧字节更大\n"
    "      （固定宽度编码），所以双发的净带宽增量小于 proto 侧字节的两倍。\n");
  return 0;
}
