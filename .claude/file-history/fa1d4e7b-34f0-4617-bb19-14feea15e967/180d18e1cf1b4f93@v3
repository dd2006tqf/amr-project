#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/bool.hpp>

#include "amr_dispatcher_core/catalog/station_catalog.hpp"
#include "amr_dispatcher_core/planning/map_geometry_utils.hpp"
#include "amr_dispatcher_core/planning/safety_space_filter.hpp"
#include "amr_dispatcher_core/planning/skeleton_extractor.hpp"
#include "amr_dispatcher_core/planning/topology_graph_builder.hpp"
#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"
#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/msg/mission_event.hpp"
#include "amr_dispatcher_interfaces/msg/safety_state.hpp"
#include "amr_dispatcher_interfaces/srv/cancel_order.hpp"
#include "amr_dispatcher_interfaces/srv/force_clear_vda5050_link.hpp"
#include "amr_dispatcher_interfaces/srv/restore_vda5050_link.hpp"
#include "amr_dispatcher_interfaces/srv/get_system_readiness.hpp"
#include "amr_dispatcher_interfaces/srv/submit_order.hpp"
#include "amr_dispatcher_interfaces/srv/block_edge.hpp"
#include "amr_dispatcher_interfaces/srv/topology_hot_update.hpp"
#include "amr_dispatcher_interfaces/srv/get_topology_overlay.hpp"

#include "amr_dispatcher_tools/alert_webhook_client.hpp"
#include "amr_dispatcher_tools/alerting.hpp"
#include "amr_dispatcher_tools/bearer_auth.hpp"
#include "amr_dispatcher_tools/health_probe.hpp"
#include "amr_dispatcher_tools/http_limits.hpp"
#include "amr_dispatcher_tools/idempotency_store.hpp"
#include "amr_dispatcher_tools/json_escape.hpp"
#include "amr_dispatcher_tools/metrics_format.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_tools {
namespace {

// 一次完整的 HTTP 请求（头部已解析、请求体已读满）。
struct HttpRequest {
  std::string method;
  std::string target;  // 原始 request-target（含 query）
  std::string path;    // 剥掉 query/fragment 的路径
  std::string query;   // 不含 '?'
  std::string body;
  std::string authorization;
};

// 一次响应（状态行 + 内容类型 + 内容）。
struct HttpResponse {
  std::string status = "200 OK";
  std::string content_type = "application/json";
  std::string body;
};

// 读请求头/体的结果。放在文件作用域是因为 QueuedConn 要携带它
// （accept 线程读头失败时，worker 必须知道"为什么失败"才能给出正确状态码）。
//
// kTimeout 与 kClosed 必须分开：读超时说明对端"还活着但没发完"，
// accept 线程应把已读字节转交 worker 继续等；而 kClosed 是对端真的断了。
// 修复前两者都返回 kClosed，accept 线程只能把半截请求丢掉——
// 于是"探针快车道"在慢客户端上会静默丢请求。
enum class ReadStatus { kOk, kClosed, kBadRequest, kTooLarge, kTimeout };

// 排队中的连接。accept 线程已经把请求头读掉（探针快车道需要它做路由判断），
// 因此这里带着已解析的头与前缀字节入队，worker 只续读请求体。
//
// `head_complete` 与 `head_ok` 是两个不同的东西，必须分开：
//   * head_complete = accept 线程**已经读完请求头**（无论结论是 Ok/400/413）。
//     此时 socket 已被 consume 掉这些字节，worker 绝不能再去读头，
//     否则它会阻塞在"下一个请求"上（本网关一连接一请求，没有下一个），
//     最终变成 5s 超时 → 对端收到连接重置而不是 400/413。
//   * head_ok = 头合法且可以继续读体。
// 修复前这里只有一个布尔量，未覆盖的错误路径（畸形 Content-Length、
// 超限、分块传输）落进了"worker 重新读头"分支，于是加固验收里
// 4 条 400/413 断言全变成连接重置。慢路径上的另一处同类缺陷是
// `raw.size() > kMaxHeaderBytes` 的判断放在 read 之前：一次 read 最多读
// 4096 字节，只有累计超过 8 KiB 才会在下一次循环被发现，而那时又先
// read 了一次——静默连接因此能多拖一个 5s 超时。
struct QueuedConn {
  int fd = -1;
  std::string prebuffered;
  bool head_complete = false;
  bool head_ok = false;
  HttpRequest req;
  std::size_t content_length = 0;
  ReadStatus head_status = ReadStatus::kOk;
  // 连接被 accept 的时刻。超时预算从这一刻起算，而不是从 worker 拿到它
  // 起算 —— 否则请求在队列里排队的时间会被重复计算，高负载下队列越深
  // 每个请求等得越久，恶化成雪崩。
  std::chrono::steady_clock::time_point accepted_at = std::chrono::steady_clock::now();
};

// 待发送的一条告警（body 已渲染成 JSON，发送线程只负责投递）。
struct AlertJob {
  std::string body;
};

// 纯 JSON 错误体。
std::string JsonErrorBody(const std::string& error, const std::string& detail = "") {
  std::string out = "{" + JsonStringPair("error", error);
  if (!detail.empty()) {
    out += "," + JsonStringPair("detail", detail);
  }
  out += "}";
  return out;
}

// 从 ad-hoc JSON 体里读一个整数；解析不了就返回默认值（绝不抛异常）。
long long SafeScanInt(const std::string& text, std::size_t from) {
  std::size_t i = from;
  while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) ++i;
  bool negative = false;
  if (i < text.size() && (text[i] == '-' || text[i] == '+')) {
    negative = (text[i] == '-');
    ++i;
  }
  long long value = 0;
  bool any = false;
  while (i < text.size() && text[i] >= '0' && text[i] <= '9') {
    value = value * 10 + (text[i] - '0');
    if (value > 1000000000LL) {  // 地图尺寸/整数参数远小于此，超了按 0 处理
      return 0;
    }
    any = true;
    ++i;
  }
  if (!any) return 0;
  return negative ? -value : value;
}

// 同理读一个浮点数：只接受 [-,+]digits[.digits][eE[+,-]digits] 的前缀。
double SafeScanDouble(const std::string& text, std::size_t from) {
  std::size_t i = from;
  while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) ++i;
  const std::size_t begin = i;
  if (i < text.size() && (text[i] == '-' || text[i] == '+')) ++i;
  bool digits = false;
  while (i < text.size() && std::isdigit(static_cast<unsigned char>(text[i]))) {
    ++i;
    digits = true;
  }
  if (i < text.size() && text[i] == '.') {
    ++i;
    while (i < text.size() && std::isdigit(static_cast<unsigned char>(text[i]))) {
      ++i;
      digits = true;
    }
  }
  if (!digits) return 0.0;
  if (i < text.size() && (text[i] == 'e' || text[i] == 'E')) {
    std::size_t j = i + 1;
    if (j < text.size() && (text[j] == '+' || text[j] == '-')) ++j;
    bool exp_digits = false;
    while (j < text.size() && std::isdigit(static_cast<unsigned char>(text[j]))) {
      ++j;
      exp_digits = true;
    }
    if (exp_digits) i = j;
  }
  const std::string num = text.substr(begin, i - begin);
  try {
    return std::stod(num);
  } catch (...) {
    // num 已由上面的正则式扫描保证形状合法，这里只是防御
    return 0.0;
  }
}

// 完整写出一段字节；对端提前关闭时返回 false（不抛、不崩）。
bool WriteAll(int fd, const std::string& data) {
  std::size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t n = ::send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    return false;  // EPIPE / EAGAIN（已超时）/ 其它
  }
  return true;
}

// 错误响应后先把对端还在发送的数据读掉再关闭。
// 直接 close 一个仍有未读数据的连接会发 RST，客户端可能连 400/413 都收不到。
// 上限 2 MiB + 单次读超时，恶意客户端最多多占一个读超时。
void DrainForClose(int fd, std::size_t max_bytes = 2U * 1024U * 1024U) {
  char sink[4096];
  std::size_t drained = 0;
  while (drained < max_bytes) {
    const ssize_t n = ::read(fd, sink, sizeof(sink));
    if (n > 0) {
      drained += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    return;
  }
}

}  // namespace

class RestGatewayNode : public rclcpp::Node {
 public:
  explicit RestGatewayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("rest_gateway_node", options) {
    declare_parameter("port", 8080);
    declare_parameter("html_path", "tools/operator_console.html");
    // 鉴权默认关闭：留空 = 不校验任何请求（现网与 e2e 行为逐字不变）
    declare_parameter("auth_token", "");
    // 急停单独口令；留空则回退 auth_token
    declare_parameter("estop_token", "");
    // 跨域白名单；留空 = 不下发 CORS 头（操作台与网关同源时无需 CORS）
    declare_parameter("cors_allow_origin", "");
    // 工作线程数：静默/慢连接只会占住一个线程，不再卡死整个网关
    declare_parameter("worker_threads", 4);
    // 待处理连接上限：过载时快速回 503，而不是无界积压
    declare_parameter("max_pending_connections", 32);
    // /metrics 抓取口令；留空 = 复用 auth_token（再空 = 不鉴权）
    declare_parameter("metrics_token", "");
    // 幂等键落盘（空 = 仅内存，重启即失；配置后跨重启去重）
    declare_parameter("idempotency_store_file", "");
    declare_parameter("idempotency_capacity", 4096);
    declare_parameter("idempotency_ttl_sec", 3600);

    // ---- 相位 11：就绪探针阈值（秒）----
    // 取值依据实际发布频率：/dispatcher/state 2Hz、/safety/state 空闲 10Hz（有指令时
    // 最高 50Hz）、/chassis/link_health 5Hz、/system/ready_flag 2Hz。阈值都给到
    // 数拍以上，避免一次调度抖动就翻红。
    declare_parameter("readyz_dispatcher_state_stale_sec", 3.0);
    declare_parameter("readyz_safety_stale_sec", 2.0);
    declare_parameter("readyz_chassis_stale_sec", 5.0);
    declare_parameter("readyz_coordinator_stale_sec", 3.0);
    // 纯仿真/多车栈里网关订阅不到全局 /chassis/link_health，置 false 避免永远 503
    declare_parameter("readyz_require_chassis", true);

    // ---- 相位 11：主动告警（默认全关 = 现网行为逐字不变）----
    declare_parameter("alert_webhook_url", "");
    declare_parameter("alert_webhook_timeout_ms", 3000);
    declare_parameter("alert_min_interval_sec", 300);
    declare_parameter("alert_queue_capacity", 64);
    declare_parameter("alert_probe_period_ms", 1000);
    declare_parameter("alert_not_ready_threshold", 3);
    declare_parameter("alert_safety_critical_sec", 30);

    // ---- 相位 11：拓扑运维端点超时 ----
    // N 台车重规划的耗时随在途任务数增长，因而远大于普通业务端点的 1~2s。
    declare_parameter("topology_op_timeout_ms", 10000);

    port_ = get_parameter("port").as_int();
    html_path_ = get_parameter("html_path").as_string();
    auth_token_ = get_parameter("auth_token").as_string();
    estop_token_ = get_parameter("estop_token").as_string();
    if (estop_token_.empty()) {
      estop_token_ = auth_token_;
    }
    cors_allow_origin_ = get_parameter("cors_allow_origin").as_string();
    metrics_token_ = get_parameter("metrics_token").as_string();
    if (metrics_token_.empty()) {
      metrics_token_ = auth_token_;  // 未单独配置 -> 跟主鉴权同口令
    }
    const int worker_threads = std::max(1, static_cast<int>(get_parameter("worker_threads").as_int()));
    max_pending_ = static_cast<std::size_t>(
        std::max(1, static_cast<int>(get_parameter("max_pending_connections").as_int())));

    readyz_dispatcher_state_stale_sec_ =
        get_parameter("readyz_dispatcher_state_stale_sec").as_double();
    readyz_safety_stale_sec_ = get_parameter("readyz_safety_stale_sec").as_double();
    readyz_chassis_stale_sec_ = get_parameter("readyz_chassis_stale_sec").as_double();
    readyz_coordinator_stale_sec_ = get_parameter("readyz_coordinator_stale_sec").as_double();
    readyz_require_chassis_ = get_parameter("readyz_require_chassis").as_bool();
    // 告警载荷里带上域号：同一台机器上可能并存多套栈（e2e / 灰度），
    // 接收端靠这个字段区分"哪一套在报警"。
    {
      const char* domain = std::getenv("ROS_DOMAIN_ID");
      if (domain != nullptr) {
        ros_domain_id_ = std::atoi(domain);
      }
    }

    // 告警配置：URL 解析失败时明确禁用（而不是每拍都撞同一个解析错误刷日志）
    const std::string alert_url = get_parameter("alert_webhook_url").as_string();
    alert_timeout_ms_ = static_cast<int>(get_parameter("alert_webhook_timeout_ms").as_int());
    alert_queue_capacity_ = static_cast<std::size_t>(
        std::max(1, static_cast<int>(get_parameter("alert_queue_capacity").as_int())));
    alert_probe_period_ms_ =
        std::max(100, static_cast<int>(get_parameter("alert_probe_period_ms").as_int()));
    topology_op_timeout_ms_ =
        std::max(1000, static_cast<int>(get_parameter("topology_op_timeout_ms").as_int()));
    AlertPolicy alert_policy;
    alert_policy.fire_after_consecutive_failures = static_cast<std::uint32_t>(
        std::max(1, static_cast<int>(get_parameter("alert_not_ready_threshold").as_int())));
    alert_policy.min_interval_sec = get_parameter("alert_min_interval_sec").as_int();
    alert_policy.safety_critical_sec = get_parameter("alert_safety_critical_sec").as_int();
    alerts_ = AlertStateMachine(alert_policy);

    if (!alert_url.empty()) {
      std::string url_error;
      const auto parsed = ParseHttpUrl(alert_url, &url_error);
      if (parsed.has_value()) {
        alert_url_ = *parsed;
      } else {
        // 不静默降级：告警通道配错的后果是"故障时没人知道"，必须在启动日志里喊出来
        RCLCPP_ERROR(get_logger(),
                     "alert_webhook_url is invalid (%s) — alerting stays DISABLED: %s",
                     url_error.c_str(), alert_url.c_str());
      }
    }

    IdempotencyStore::Options idem;
    idem.capacity = static_cast<std::size_t>(
        std::max(1, static_cast<int>(get_parameter("idempotency_capacity").as_int())));
    idem.ttl_sec = static_cast<std::int64_t>(get_parameter("idempotency_ttl_sec").as_int());
    idempotency_ = IdempotencyStore(idem);
    idempotency_store_file_ = get_parameter("idempotency_store_file").as_string();
    if (!idempotency_store_file_.empty()) {
      std::ifstream in(idempotency_store_file_);
      if (in) {
        const std::string text((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>());
        const std::size_t loaded = idempotency_.Restore(text, NowEpochSec());
        RCLCPP_INFO(get_logger(), "Idempotency store restored from %s: %zu key(s)",
                    idempotency_store_file_.c_str(), loaded);
      } else {
        RCLCPP_INFO(get_logger(), "Idempotency store file %s not found (starting empty)",
                    idempotency_store_file_.c_str());
      }
    }

    submit_cli_ = create_client<amr_dispatcher_interfaces::srv::SubmitOrder>("/dispatcher/submit_order");
    cancel_cli_ = create_client<amr_dispatcher_interfaces::srv::CancelOrder>("/dispatcher/cancel_order");
    readiness_cli_ = create_client<amr_dispatcher_interfaces::srv::GetSystemReadiness>("/system/ready");
    force_clear_cli_ = create_client<amr_dispatcher_interfaces::srv::ForceClearVda5050Link>(
        "/dispatcher/force_clear_vda5050_link");
    restore_cli_ = create_client<amr_dispatcher_interfaces::srv::RestoreVda5050Link>(
        "/dispatcher/restore_vda5050_link");
    // 相位 11：拓扑运维客户端（只在运维请求时调用；不参与探针/指标路径）
    block_edge_cli_ = create_client<amr_dispatcher_interfaces::srv::BlockEdge>(
        "/v2/block_edge");
    topology_hot_update_cli_ =
        create_client<amr_dispatcher_interfaces::srv::TopologyHotUpdate>(
            "/v2/topology_hot_update");
    get_topology_overlay_cli_ =
        create_client<amr_dispatcher_interfaces::srv::GetTopologyOverlay>(
            "/v2/get_topology_overlay");

    estop_pub_ = create_publisher<std_msgs::msg::Bool>("/safety/estop", 10);

    state_sub_ = create_subscription<amr_dispatcher_interfaces::msg::DispatcherState>(
        "/dispatcher/state", 10,
        [this](const amr_dispatcher_interfaces::msg::DispatcherState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_state_ = *msg;
          state_seen_ = true;
          last_state_arrival_ = std::chrono::steady_clock::now();
          // 拓扑叠加层状态随 DispatcherState 一起下发，指标与 /readyz 都不用再打服务
          // 字段名以 DispatcherState.msg 为准（overlay_active / overlay_revision /
          // blocked_edges / one_way_edges），本地成员名才带 topology_ 前缀。
          topology_overlay_active_ = msg->overlay_active;
          topology_overlay_revision_ = msg->overlay_revision;
          topology_blocked_edges_ = msg->blocked_edges;
          topology_one_way_edges_ = msg->one_way_edges;
        });

    safety_sub_ = create_subscription<amr_dispatcher_interfaces::msg::SafetyState>(
        "/safety/state", 10,
        [this](const amr_dispatcher_interfaces::msg::SafetyState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_safety_ = *msg;
          safety_seen_ = true;
          last_safety_arrival_ = std::chrono::steady_clock::now();
          safety_critical_ = (msg->severity == "CRITICAL");
          if (safety_critical_) {
            if (safety_critical_since_.time_since_epoch().count() == 0) {
              safety_critical_since_ = std::chrono::steady_clock::now();
            }
          } else {
            safety_critical_since_ = {};
          }
        });

    link_sub_ = create_subscription<amr_dispatcher_interfaces::msg::ChassisLinkHealth>(
        "/chassis/link_health", 10,
        [this](const amr_dispatcher_interfaces::msg::ChassisLinkHealth::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_link_ = *msg;
          link_seen_ = true;
          last_link_arrival_ = std::chrono::steady_clock::now();
        });

    // 协调器的整体就绪布尔量（transient_local）：晚加入的网关也能立刻拿到当前值，
    // 于是"协调器还在但没提拉调度器"与"协调器根本没起来"能被区分开。
    ready_flag_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/system/ready_flag", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          ready_flag_ = msg->data;
          ready_flag_seen_ = true;
          last_ready_flag_arrival_ = std::chrono::steady_clock::now();
        });

    event_sub_ = create_subscription<amr_dispatcher_interfaces::msg::MissionEvent>(
        "/dispatcher/events", 20,
        [this](const amr_dispatcher_interfaces::msg::MissionEvent::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          recent_events_.push_back(*msg);
          if (recent_events_.size() > 50) {
            recent_events_.erase(recent_events_.begin());
          }
        });

    server_thread_ = std::thread(&RestGatewayNode::RunServer, this);
    for (int i = 0; i < worker_threads; ++i) {
      workers_.emplace_back(&RestGatewayNode::WorkerLoop, this);
    }

    // 告警发送线程与巡检定时器只在配置了 webhook 时才存在：
    // 未配置时进程的线程数/定时器/话题与改动前完全一致。
    if (!alert_url_.host.empty()) {
      alert_thread_ = std::thread(&RestGatewayNode::AlertSenderLoop, this);
      alert_timer_ = create_wall_timer(std::chrono::milliseconds(alert_probe_period_ms_),
                                       std::bind(&RestGatewayNode::AlertProbeTick, this));
      RCLCPP_INFO(get_logger(),
                  "Alerting ENABLED: webhook http://%s:%d%s (probe %dms, threshold %u, "
                  "safety sustain %llds)",
                  alert_url_.host.c_str(), alert_url_.port, alert_url_.path.c_str(),
                  alert_probe_period_ms_,
                  alerts_.policy().fire_after_consecutive_failures,
                  static_cast<long long>(alerts_.policy().safety_critical_sec));
    }

    RCLCPP_INFO(get_logger(), "REST Gateway started on port %d", port_);
    if (!auth_token_.empty()) {
      RCLCPP_INFO(get_logger(), "REST Gateway: Bearer token authentication ENABLED");
    }
  }

  ~RestGatewayNode() override {
    running_ = false;
    if (server_fd_ >= 0) {
      ::shutdown(server_fd_, SHUT_RDWR);  // 唤醒阻塞在 accept 的线程
      close(server_fd_);
      server_fd_ = -1;
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stopping_ = true;
      for (const auto& conn : pending_) {
        close(conn.fd);  // 未处理的连接直接断开
      }
      pending_.clear();
    }
    queue_cv_.notify_all();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    for (auto& worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    // 告警线程最后收：先停巡检定时器（它在 executor 上，spin 结束后自然不再触发），
    // 再唤醒发送线程退出。队列里未发出的告警直接丢弃——进程都要关了。
    {
      std::lock_guard<std::mutex> lock(alert_mutex_);
      alert_stopping_ = true;
      alert_queue_.clear();
    }
    alert_cv_.notify_all();
    if (alert_thread_.joinable()) {
      alert_thread_.join();
    }
  }

 private:
  // ---------------------------------------------------------------------------
  // 连接调度：主线程只 accept，工作线程负责读写与业务
  // ---------------------------------------------------------------------------
  void RunServer() {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create socket");
      return;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(static_cast<uint16_t>(port_));

    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
      RCLCPP_ERROR(get_logger(), "Socket bind failed on port %d", port_);
      return;
    }

    if (listen(server_fd_, 64) < 0) {
      RCLCPP_ERROR(get_logger(), "Socket listen failed");
      return;
    }

    while (running_ && rclcpp::ok()) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      const int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
      const auto accepted_at = std::chrono::steady_clock::now();
      if (client_fd < 0) {
        if (!running_) break;
        if (errno == EINTR) continue;
        continue;
      }
      // 每个连接都必须有限等待：静默连接不能永久占住工作线程
      timeval tv{};
      tv.tv_sec = kSocketTimeoutSec;
      tv.tv_usec = 0;
      setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

      // 探针快车道：在主线程读请求头，命中 /healthz 与 /readyz 就地应答。
      // 这样即使 4 个 worker 全部卡在某个慢服务调用里（DDS 卡死就是这样），
      // 编排系统的探针依然拿得到答案 —— 否则探针会和它要检测的故障一起失效。
      //
      // **主线程绝不能被一个慢客户端占住**（旧检查里"静默连接存在期间
      // 另一连接正常响应 < 3s"就是这么挂的：主线程在 read 上等了 5s 超时）。
      // 所以先 poll 一个很短的预算：
      //   * 有数据 -> 立刻读头（本地探针请求头只有几百字节，一次 read 就够）
      //   * 无数据 -> 立刻入队，让 worker 去等那 5s。主线程马上回到 accept。
      std::string prebuffered;
      HttpRequest probe_req;
      std::size_t probe_content_length = 0;
      ReadStatus probe_head = ReadStatus::kTimeout;
      {
        struct pollfd pfd{};
        pfd.fd = client_fd;
        pfd.events = POLLIN;
        const int ready = ::poll(&pfd, 1, kAcceptReadBudgetMs);
        if (ready > 0 && (pfd.revents & POLLIN) != 0) {
          // 只读"已到达的字节"：把 socket 切成非阻塞，read 立刻返回 EAGAIN
          // 而不是在这里等满 5s。头部已完整时才可能得到 kOk。
          //
          // 为什么用 O_NONBLOCK 而不是 SO_RCVTIMEO=0：按 Linux 语义，接收超时
          // 置零表示"禁用超时"（= 永远阻塞）。写这行时把它当成"不等待"，
          // 于是 accept 线程在任何一个只发了半截头的连接上永久卡死 ——
          // 全站请求排队，重启进程后又在同一个客户端上卡死，表现成
          // "重启也没用"。O_NONBLOCK 是唯一能真正表达"不等待"的手段。
          const int old_flags = ::fcntl(client_fd, F_GETFL, 0);
          if (old_flags >= 0) {
            ::fcntl(client_fd, F_SETFL, old_flags | O_NONBLOCK);
          }
          probe_head = ReadRequestHead(client_fd, &probe_req, &probe_content_length,
                                       &prebuffered);
          if (old_flags >= 0) {
            ::fcntl(client_fd, F_SETFL, old_flags);
          }
        }
      }
      if (probe_head == ReadStatus::kOk && probe_req.method == "GET" &&
          (probe_req.path == "/healthz" || probe_req.path == "/readyz")) {
        HandleProbeInline(client_fd, probe_req);
        close(client_fd);
        continue;
      }
      if (probe_head == ReadStatus::kClosed) {
        // 对端已断开；若已经读到半截请求，交给 worker 也无意义（socket 已 EOF）
        close(client_fd);
        continue;
      }

      bool queued = false;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (pending_.size() < max_pending_) {
          // head_complete 的含义是"头字节已经从 socket 上消费掉"：
          //   * kOk / kBadRequest / kTooLarge —— accept 线程读到了完整头部，
          //     worker 只能按状态处置，绝不能再读头；
          //   * kTimeout —— 只读到半截（或一个字节都没读），头**没有**消费完，
          //     worker 必须带着 prebuffered 前缀继续读。
          const bool head_complete = probe_head != ReadStatus::kTimeout;
          pending_.push_back(QueuedConn{client_fd, std::move(prebuffered),
                                        head_complete,
                                        /*head_ok=*/probe_head == ReadStatus::kOk,
                                        std::move(probe_req),
                                        probe_content_length,
                                        probe_head,
                                        accepted_at});
          queued = true;
        }
      }
      if (!queued) {
        // 过载保护：立刻回 503 并关闭，避免无界积压把内存吃光
        HttpResponse resp;
        resp.status = "503 Service Unavailable";
        resp.body = JsonErrorBody("gateway_overloaded", "too many pending connections");
        RecordRequestStatus(resp.status);
        WriteAll(client_fd, SerializeResponse(resp, false));
        close(client_fd);
        continue;
      }
      queue_cv_.notify_one();
    }
  }

  void WorkerLoop() {
    while (true) {
      QueuedConn conn;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] { return stopping_ || !pending_.empty(); });
        if (stopping_ && pending_.empty()) {
          return;
        }
        conn = std::move(pending_.front());
        pending_.pop_front();
      }
      HandleClientSafely(conn);
      close(conn.fd);
    }
  }

  // 任何异常都必须收敛成 400：旧实现里一个 stoi 抛异常就把整个进程 terminate。
  //
  // `accepted_at` 是连接被 accept 的时刻。慢路径（worker 自己读头）按它算
  // **剩余预算**再设读写超时：请求在队列里排了多久，就该少等多久。
  // 否则"accept 时已经耗掉半个超时"的连接在 worker 这里又能重新等满一个
  // 完整超时，高负载下队列越深、每个请求被拖得越久 —— 雪崩就是这样长出来的。
  void HandleClientSafely(const QueuedConn& conn) {
    try {
      if (conn.head_complete) {
        // accept 线程已经把请求头读完（成功的与失败的都算）。这里**只能**
        // 按 head_status 处置，绝不能再读头：socket 上的头字节已经消费掉了，
        // 再读会阻塞到超时，对端拿到的是连接重置而不是应有的 400/413。
        switch (conn.head_status) {
          case ReadStatus::kOk: {
            // 只续读请求体
            HttpRequest req = conn.req;
            const ReadStatus body_status =
                ReadRequestBody(conn.fd, &req, conn.content_length, conn.prebuffered);
            FinishAfterBody(conn.fd, req, body_status);
            return;
          }
          case ReadStatus::kBadRequest:
            // 与 HandleClient / FinishAfterBody 的错误响应逐字一致
            FinishAfterBody(conn.fd, conn.req, ReadStatus::kBadRequest);
            return;
          case ReadStatus::kTooLarge:
            FinishAfterBody(conn.fd, conn.req, ReadStatus::kTooLarge);
            return;
          case ReadStatus::kClosed:
            return;  // 对端没发完整请求就断了：静默关闭
        }
        return;
      }
      // accept 线程没有读完头（读超时，或一个字节都没到）：worker 带着已读
      // 前缀继续读。前缀必须原样传回去 —— 丢掉就会把半截请求当"对端断开"。
      //
      // 超时按"从 accept 起算的剩余预算"重设：accept 线程已经花掉的
      // kAcceptReadBudgetMs 不该在 worker 这里被重新送一遍。
      const auto spent = std::chrono::steady_clock::now() - conn.accepted_at;
      const auto budget = std::chrono::seconds(kSocketTimeoutSec) - spent;
      const int remaining_sec =
          budget.count() > 0 ? static_cast<int>(budget.count()) : 1;
      timeval tv{};
      tv.tv_sec = remaining_sec;
      tv.tv_usec = 0;
      setsockopt(conn.fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      setsockopt(conn.fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
      HandleClient(conn.fd, conn.prebuffered);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "请求处理异常（已兜底为 400）: %s", e.what());
      HttpResponse resp;
      resp.status = "400 Bad Request";
      resp.body = JsonErrorBody("bad_request");
      RecordRequestStatus(resp.status);
      WriteAll(conn.fd, SerializeResponse(resp, false));
    } catch (...) {
      RCLCPP_WARN(get_logger(), "请求处理未知异常（已兜底为 400）");
      HttpResponse resp;
      resp.status = "400 Bad Request";
      resp.body = JsonErrorBody("bad_request");
      RecordRequestStatus(resp.status);
      WriteAll(conn.fd, SerializeResponse(resp, false));
    }
  }

  // ---------------------------------------------------------------------------
  // 健康探针：在 accept 线程内就地应答，绝不调用任何 ROS 服务
  //
  // 这里只读本地缓存的消息与到达时刻 + 一个 std::mutex 的短临界区。
  // 不调 /system/ready 是刻意的：那个调用会阻塞 300ms~1s，一旦 DDS 卡死
  // 探针就会跟着一起卡住——而"DDS 卡死"正是探针要发现的东西。
  // ---------------------------------------------------------------------------
  void HandleProbeInline(int client_fd, const HttpRequest& req) {
    HttpResponse resp;
    if (req.path == "/healthz") {
      // liveness：只回答"这个进程还能处理 HTTP"。不含任何心跳/业务状态，
      // 所以 DDS 全挂时它依然是 200 —— 这正是 liveness 的定义。
      const double uptime =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at_)
              .count();
      resp.body = RenderLivenessJson(uptime);
      RecordRequestStatus(resp.status);
      WriteAll(client_fd, SerializeResponse(resp, false));
      return;
    }

    // readiness：逐组件聚合
    const auto probes = CollectComponentProbes();
    const auto verdict = EvaluateReadiness(probes);
    resp.body = RenderReadinessJson(verdict);
    if (!verdict.ready) {
      resp.status = "503 Service Unavailable";
    }
    RecordRequestStatus(resp.status);
    WriteAll(client_fd, SerializeResponse(resp, false));
  }

  // 从本地缓存组装逐组件探针。时间在这里读（纯逻辑层不碰时钟）。
  // 加锁版：调用方自己持 data_mutex_（RenderMetrics 已在锁内，再锁一次就是死锁）。
  std::vector<ComponentProbe> CollectComponentProbesLocked() const {
    using namespace std::chrono;
    const auto now = steady_clock::now();

    const auto age_of = [&now](const steady_clock::time_point& stamp) -> double {
      return duration<double>(now - stamp).count();
    };

    std::vector<ComponentProbe> probes;

    ComponentProbe dispatcher;
    dispatcher.name = "dispatcher";
    dispatcher.seen = state_seen_;
    dispatcher.age_seconds = state_seen_ ? age_of(last_state_arrival_) : -1.0;
    dispatcher.stale_after_seconds = readyz_dispatcher_state_stale_sec_;
    // 能收到 /dispatcher/state 就说明节点在 active 并且定时器在跑；
    // 消息本身不带 lifecycle 状态，所以判据就是"新鲜"。
    dispatcher.semantic_ok = true;
    dispatcher.detail = "state_age_ms=" +
                        std::to_string(static_cast<long long>(dispatcher.age_seconds * 1000.0));
    probes.push_back(dispatcher);

    ComponentProbe safety;
    safety.name = "safety";
    safety.seen = safety_seen_;
    safety.age_seconds = safety_seen_ ? age_of(last_safety_arrival_) : -1.0;
    safety.stale_after_seconds = readyz_safety_stale_sec_;
    safety.semantic_ok = safety_seen_ && latest_safety_.motion_allowed &&
                         latest_safety_.severity != "CRITICAL";
    safety.detail = latest_safety_.severity.empty() ? "no_state" : latest_safety_.severity;
    if (safety_seen_ && !latest_safety_.reason.empty()) {
      safety.detail += ":" + latest_safety_.reason;
    }
    probes.push_back(safety);

    ComponentProbe chassis;
    chassis.name = "chassis";
    chassis.seen = link_seen_;
    chassis.age_seconds = link_seen_ ? age_of(last_link_arrival_) : -1.0;
    chassis.stale_after_seconds = readyz_chassis_stale_sec_;
    chassis.semantic_ok = link_seen_ && latest_link_.is_healthy;
    chassis.detail = link_seen_ ? latest_link_.tier : "no_link_health";
    // 多车模式下 /chassis/link_health 是 per-vehicle 话题，网关订阅不到：
    // 如实标"未监控"，而不是用一个恒为 false 的组件把整个就绪探针拖成 503。
    chassis.monitored = readyz_require_chassis_;
    probes.push_back(chassis);

    ComponentProbe coordinator;
    coordinator.name = "coordinator";
    coordinator.seen = ready_flag_seen_;
    coordinator.age_seconds = ready_flag_seen_ ? age_of(last_ready_flag_arrival_) : -1.0;
    coordinator.stale_after_seconds = readyz_coordinator_stale_sec_;
    coordinator.semantic_ok = ready_flag_seen_ && ready_flag_;
    coordinator.detail = ready_flag_seen_ ? (ready_flag_ ? "ready" : "not_ready") : "no_flag";
    probes.push_back(coordinator);

    return probes;
  }

  // 加锁版：供探针与告警巡检使用
  std::vector<ComponentProbe> CollectComponentProbes() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return CollectComponentProbesLocked();
  }

  // ---------------------------------------------------------------------------
  // 主动告警：巡检（ROS executor 上，只读缓存）+ 发送（独立线程，唯一做阻塞 IO 的地方）
  //
  // 分工的理由：巡检要读探针状态，必须和订阅回调在同一 executor 上；而 HTTP POST
  // 会阻塞到超时。把 POST 留在 executor 上会拖慢所有 ROS 回调（包括 /healthz 路径
  // 之外的业务），所以拆成"executor 组装 → 有界队列 → 独立线程发送"。
  // ---------------------------------------------------------------------------
  void AlertProbeTick() {
    const auto probes = CollectComponentProbes();
    const auto verdict = EvaluateReadiness(probes);

    AlertObservation observation;
    observation.now_unix = NowEpochSec();
    observation.ready = verdict.ready;
    for (const auto& probe : probes) {
      switch (EvaluateComponent(probe)) {
        case ComponentStatus::kStale:
        case ComponentStatus::kNeverSeen:
        case ComponentStatus::kUnhealthy:
          observation.missing_components.push_back(probe.name);
          break;
        case ComponentStatus::kOk:
        case ComponentStatus::kNotMonitored:
          break;
      }
    }

    // 安全门 CRITICAL 持续时长 + 关键计数，供接收端一眼判断严重程度
    double critical_seconds = 0.0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (safety_critical_since_.time_since_epoch().count() != 0) {
        critical_seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          safety_critical_since_)
                .count();
      }
      observation.safety_critical = safety_critical_;
    }    char buffer[256];
    std::snprintf(buffer, sizeof(buffer),
                  "{\"state\":\"%s\",\"safety_critical_seconds\":%.1f,"
                  "\"topology_overlay_revision\":%u,\"blocked_edges\":%u",
                  verdict.state.c_str(), critical_seconds,
                  static_cast<unsigned>(topology_overlay_revision_),
                  static_cast<unsigned>(topology_blocked_edges_));
    observation.details_json = buffer;
    // 每台受监控组件的时延都带上，接收端不必再回查 /readyz
    for (const auto& probe : probes) {
      char part[160];
      std::snprintf(part, sizeof(part), ",\"%s_age_seconds\":%.1f", probe.name.c_str(),
                    probe.age_seconds);
      observation.details_json += part;
    }
    observation.details_json += "}";

    for (auto& event : alerts_.Observe(observation)) {
      EnqueueAlert(std::move(event));
    }
  }

  void EnqueueAlert(AlertEvent event) {
    const std::string body =
        RenderAlertPayload(event, HostName(), port_, ros_domain_id_, static_cast<long>(::getpid()));
    {
      std::lock_guard<std::mutex> lock(alert_mutex_);
      if (alert_queue_.size() >= alert_queue_capacity_) {
        // 队满丢最旧：一条 5 分钟前的 "not ready" 不如刚刚这条有用。
        alert_queue_.pop_front();
        ++alert_dropped_;
      }
      alert_queue_.push_back(AlertJob{body});
    }
    alert_cv_.notify_one();
  }

  void AlertSenderLoop() {
    while (true) {
      AlertJob job;
      {
        std::unique_lock<std::mutex> lock(alert_mutex_);
        alert_cv_.wait(lock, [this] { return alert_stopping_ || !alert_queue_.empty(); });
        if (alert_stopping_) {
          return;
        }
        job = std::move(alert_queue_.front());
        alert_queue_.pop_front();
      }
      const auto outcome = PostWebhook(alert_url_, job.body, alert_timeout_ms_);
      if (outcome.result == WebhookResult::kOk) {
        ++alert_sent_;
      } else {
        // 不重试：告警是"当下状态快照"，重发一条过时的告警不如让下一拍发新的。
        // 失败计数进 /metrics，抓取侧可以据此对"告警通道本身"告警。
        ++alert_failures_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000,
                             "Alert webhook delivery failed (%s): %s",
                             WebhookResultName(outcome.result), outcome.detail.c_str());
      }
    }
  }

  static std::string HostName() {
    char name[256] = {0};
    if (::gethostname(name, sizeof(name) - 1) != 0) {
      return "unknown";
    }
    return std::string(name);
  }

  // ---------------------------------------------------------------------------
  // 请求读取：头部上限、请求体上限、Content-Length 严格解析、分块传输拒绝
  //
  // 拆成"读头"与"读体"两步，是为了让健康探针能绕过工作线程池：
  // accept 线程读完头就知道路径，探针直接就地应答；其余请求带着已读到的
  // 前缀字节入队，由 worker 继续读体。旧实现把两件事绑在一起，一旦 DDS 卡死
  // 占满 worker，探针就永远得不到应答——而那正是探针要发现的失效模式。
  // ---------------------------------------------------------------------------


  ReadStatus ReadRequestHead(int client_fd, HttpRequest* out, std::size_t* content_length,
                             std::string* prebuffered) {
    // prebuffered 既是入参也是出参：
    //   入参 = 此前已经读到的字节（accept 线程探针快车道读超时后转交 worker 续读）。
    //          修复前这里直接 `std::string raw;` 把入参丢掉，所谓"带前缀入队"
    //          从来只是假象。
    //   出参 = 头部之后剩下的字节，交给 ReadRequestBody。
    std::string raw = std::move(*prebuffered);
    prebuffered->clear();
    raw.reserve(raw.size() + 1024);
    char buffer[4096];
    // 失败时把已读字节写回，让上层可以继续读，而不是丢掉它们
    const auto bail = [&](ReadStatus status) {
      *prebuffered = std::move(raw);
      return status;
    };

    // 1) 读到头部结束标记
    std::size_t header_end = std::string::npos;
    while (true) {
      header_end = raw.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        break;
      }
      // 先判上限再 read：反过来的话，一次 read 就能把 raw 推过上限，
      // 而循环体在下一次迭代才发现 —— 对静默连接等于白白多等一个超时。
      if (raw.size() > kMaxHeaderBytes) {
        return bail(ReadStatus::kBadRequest);
      }
      const ssize_t n = ::read(client_fd, buffer, sizeof(buffer));
      if (n == 0) {
        return bail(ReadStatus::kClosed);
      }
      if (n < 0) {
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return bail(ReadStatus::kTimeout);  // 对端还活着但没发完
        }
        return bail(ReadStatus::kClosed);
      }
      raw.append(buffer, static_cast<std::size_t>(n));
      // read 之后再判一次：超大头部必须在**本次**循环就拒绝，
      // 不能等到下一轮（那时又会先 read 一次）。
      if (raw.size() > kMaxHeaderBytes && raw.find("\r\n\r\n") == std::string::npos) {
        return bail(ReadStatus::kBadRequest);
      }
    }
    if (header_end > kMaxHeaderBytes) {
      return bail(ReadStatus::kBadRequest);  // 头部过大
    }
    const std::string head = raw.substr(0, header_end);

    // 2) 请求行
    std::istringstream first_line(head.substr(0, head.find("\r\n")));
    std::string version;
    first_line >> out->method >> out->target >> version;
    if (out->method.empty() || out->target.empty()) {
      return bail(ReadStatus::kBadRequest);
    }
    out->path = RequestPathOnly(out->target);
    out->query = RequestQueryOnly(out->target);
    FindHeaderValue(head, "Authorization", &out->authorization);

    // 3) 请求体长度：分块传输不支持（猜长度 = 读到连接耗尽，直接拒绝）
    if (HasChunkedTransferEncoding(head)) {
      return bail(ReadStatus::kBadRequest);
    }
    *content_length = 0;
    std::string cl_value;
    if (FindHeaderValue(head, "Content-Length", &cl_value)) {
      switch (ParseContentLength(cl_value, content_length)) {
        case ContentLengthStatus::kOk:
          break;
        case ContentLengthStatus::kMissing:
          *content_length = 0;
          break;
        case ContentLengthStatus::kTooLarge:
          return bail(ReadStatus::kTooLarge);
        case ContentLengthStatus::kInvalid:
          return bail(ReadStatus::kBadRequest);
      }
    }

    // 4) 把已读到的字节（可能已含部分/全部请求体）交给调用方，
    //    由 ReadRequestBody 续读。探针路径在这里就返回，不需要读体。
    *prebuffered = raw.substr(header_end + 4);
    if (prebuffered->size() > kMaxRequestBytes) {
      return ReadStatus::kTooLarge;
    }
    return ReadStatus::kOk;
  }

  // 续读请求体：先用 ReadRequestHead 留下的前缀字节，再按 Content-Length 补足。
  ReadStatus ReadRequestBody(int client_fd, HttpRequest* out, std::size_t content_length,
                             std::string prebuffered) {
    std::string body;
    body.reserve(content_length);
    if (prebuffered.size() > content_length) {
      prebuffered.resize(content_length);  // 多读的属于下一个请求（本网关一连接一请求，丢弃）
    }
    body = std::move(prebuffered);

    char buffer[4096];
    while (body.size() < content_length) {
      if (body.size() > kMaxRequestBytes) {
        return ReadStatus::kTooLarge;
      }
      const ssize_t n = ::read(client_fd, buffer, sizeof(buffer));
      if (n == 0) {
        return ReadStatus::kBadRequest;  // 声明了长度却提前断开
      }
      if (n < 0) {
        if (errno == EINTR) continue;
        return ReadStatus::kClosed;
      }
      body.append(buffer, static_cast<std::size_t>(n));
    }
    if (body.size() > kMaxRequestBytes) {
      return ReadStatus::kTooLarge;
    }
    out->body = std::move(body);
    return ReadStatus::kOk;
  }

  // ---------------------------------------------------------------------------
  // 请求分发
  // ---------------------------------------------------------------------------
  void HandleClient(int client_fd, std::string prebuffered = std::string()) {
    HttpRequest req;
    std::size_t content_length = 0;
    const ReadStatus head_status =
        ReadRequestHead(client_fd, &req, &content_length, &prebuffered);
    // kTimeout 必须与 kClosed 一样静默关闭。
    //
    // 引出这一行的真实事故：accept 快车道把"半截请求"以 kTimeout 入队，worker
    // 接了却只等满自己的 5s（对端一个字都没再发），拿回 kTimeout —— 而下面
    // 三个 if 都不匹配它，于是带着一个**全空**的 HttpRequest 一路走到
    // DispatchAfterRead，最后落到路由兜底返回 404 {"error":"not_found"}。
    // 表现就是：浏览器偶尔打开控制台得到一页 not_found，而且这种"空请求"
    // 完全不影响其它连接，只在压测/冷启动的时序里冒出来。
    //
    // 语义上 kTimeout 就是"对端在超时内没交出请求"，等价于本该发生的
    // kClosed —— 相位 11 之前 ReadRequestHead 把 EAGAIN 直接当 kClosed，
    // 这里保持那条老路径的行为：不派发、不应答、静默关闭。
    if (head_status == ReadStatus::kClosed || head_status == ReadStatus::kTimeout) {
      return;  // 对端没发完整请求就断了 / 迟迟不发：静默关闭
    }
    if (head_status == ReadStatus::kBadRequest) {
      HttpResponse resp;
      resp.status = "400 Bad Request";
      resp.body = JsonErrorBody("bad_request");
      RecordRequestStatus(resp.status);
      WriteAll(client_fd, SerializeResponse(resp, false));
      DrainForClose(client_fd, 64U * 1024U);
      return;
    }
    if (head_status == ReadStatus::kTooLarge) {
      HttpResponse resp;
      resp.status = "413 Payload Too Large";
      resp.body = JsonErrorBody("payload_too_large");
      RecordRequestStatus(resp.status);
      WriteAll(client_fd, SerializeResponse(resp, false));
      DrainForClose(client_fd);
      return;
    }
    const ReadStatus body_status =
        ReadRequestBody(client_fd, &req, content_length, std::move(prebuffered));
    FinishAfterBody(client_fd, req, body_status);
  }

  // 请求体读取结果的统一收口：400 / 413 / 放行。
  // HandleClient（worker 自读头）与 HandleClientSafely（accept 线程预读头）共用，
  // 保证两条路径的错误码与响应体逐字一致。
  void FinishAfterBody(int client_fd, const HttpRequest& req, ReadStatus body_status) {
    if (body_status == ReadStatus::kClosed) {
      return;
    }
    if (body_status == ReadStatus::kBadRequest) {
      HttpResponse resp;
      resp.status = "400 Bad Request";
      resp.body = JsonErrorBody("bad_request");
      RecordRequestStatus(resp.status);
      WriteAll(client_fd, SerializeResponse(resp, false));
      DrainForClose(client_fd, 64U * 1024U);
      return;
    }
    if (body_status == ReadStatus::kTooLarge) {
      HttpResponse resp;
      resp.status = "413 Payload Too Large";
      resp.body = JsonErrorBody("payload_too_large");
      RecordRequestStatus(resp.status);
      WriteAll(client_fd, SerializeResponse(resp, false));
      DrainForClose(client_fd);
      return;
    }
    DispatchAfterRead(client_fd, req);
  }

  // 请求已完整读取（头 + 体）后的公共路径：OPTIONS / 鉴权 / 路由。
  // 探针快车道与 worker 路径共用这一段，保证两边的 OPTIONS 与鉴权语义不分叉。
  void DispatchAfterRead(int client_fd, const HttpRequest& req) {
    if (req.method == "OPTIONS") {
      HttpResponse resp;
      resp.status = "204 No Content";
      resp.body.clear();
      resp.content_type.clear();
      RecordRequestStatus(resp.status);
      WriteAll(client_fd, SerializeResponse(resp, true));
      return;
    }

    // 鉴权中间件：auth_token 为空时 IsAuthorized 恒真（默认关闭）
    if (!IsAuthExemptPath(req.path) && !Authorize(req)) {
      HttpResponse resp;
      resp.status = "401 Unauthorized";
      resp.body = JsonErrorBody("unauthorized");
      std::string raw = "HTTP/1.1 401 Unauthorized\r\n";
      raw += "Content-Type: application/json\r\n";
      raw += "Content-Length: " + std::to_string(resp.body.size()) + "\r\n";
      raw += "WWW-Authenticate: Bearer\r\n";
      raw += ExtraHeaders();
      raw += "Connection: close\r\n\r\n";
      raw += resp.body;
      RecordRequestStatus(resp.status);
      WriteAll(client_fd, raw);
      return;
    }

    const HttpResponse resp = Dispatch(req);
    RecordRequestStatus(resp.status);
    WriteAll(client_fd, SerializeResponse(resp, true));
  }

  // 状态码按前三位归类（"200 OK" -> "200"），保证指标标签基数有限
  void RecordRequestStatus(const std::string& status) {
    const std::string code = status.substr(0, std::min<std::size_t>(3, status.size()));
    std::lock_guard<std::mutex> lock(data_mutex_);
    ++request_counts_[code];
  }

  bool Authorize(const HttpRequest& req) const {
    // 急停可单独配口令；estop_token 为空时回退 auth_token
    if (req.path == "/metrics") {
      return IsAuthorized(req.authorization, metrics_token_);
    }
    if (req.path == "/api/operator/estop") {
      return IsAuthorized(req.authorization, estop_token_);
    }
    return IsAuthorized(req.authorization, auth_token_);
  }

  HttpResponse Dispatch(const HttpRequest& req) {
    HttpResponse resp;
    const std::string& method = req.method;
    const std::string& path = req.path;

    // 精确路由：旧实现用 find() 子串匹配，/api/v1/ordersXYZ 也会命中订单路由
    if (method == "GET" && PathMatchesAny(path, "/", "/index.html")) {
      resp.content_type = "text/html; charset=utf-8";
      resp.body = ReadStaticHtml();
      if (resp.body.empty()) {
        resp.status = "404 Not Found";
        resp.content_type = "text/html; charset=utf-8";
        resp.body = "<h1>404 Not Found: operator_console.html</h1>";
      }
      return resp;
    }
    if (method == "GET" && PathMatchesAny(path, "/api/operator/snapshot", "/api/v1/state")) {
      resp.body = FormatSnapshotJson();
      return resp;
    }
    if (method == "GET" && PathMatches(path, "/api/v1/vehicle_links")) {
      resp.body = FormatVehicleLinksJson();
      return resp;
    }
    if (method == "POST" && PathMatches(path, "/api/v1/force_clear_link")) {
      return HandleForceClear(req);
    }
    if (method == "POST" && PathMatches(path, "/api/v1/restore_link")) {
      return HandleRestore(req);
    }
    if (method == "GET" && PathMatches(path, "/metrics")) {
      resp.status = "200 OK";
      resp.content_type = "text/plain; version=0.0.4; charset=utf-8";
      resp.body = RenderMetrics();
      return resp;
    }
    if (method == "GET" && PathMatches(path, "/api/system/ready")) {
      return HandleReadiness();
    }
    if (method == "POST" && PathMatches(path, "/api/v1/orders")) {
      return HandleSubmitOrder(req);
    }
    if (method == "POST" && PathMatchesAny(path, "/api/v1/cancel", "/api/workflows/cancel")) {
      return HandleCancel(req);
    }
    if (method == "POST" && PathMatches(path, "/api/operator/estop")) {
      std_msgs::msg::Bool estop_msg;
      estop_msg.data = true;
      estop_pub_->publish(estop_msg);
      resp.body = "{\"status\":\"ok\",\"estop_engaged\":true}";
      return resp;
    }
    if (method == "POST" && PathMatches(path, "/api/v1/map/upload")) {
      return HandleMapUpload(req);
    }
    // 相位 11：拓扑运维（施工区 / 单向 / 临时站点）
    if (method == "GET" && PathMatches(path, "/api/v1/topology/overlay")) {
      return HandleGetTopologyOverlay(req);
    }
    if (method == "POST" && PathMatches(path, "/api/v1/topology/block_edge")) {
      return HandleBlockEdge(req, "block");
    }
    if (method == "POST" && PathMatches(path, "/api/v1/topology/unblock_edge")) {
      return HandleBlockEdge(req, "unblock");
    }
    if (method == "POST" && PathMatches(path, "/api/v1/topology/one_way_edge")) {
      return HandleBlockEdge(req, "one_way");
    }
    if (method == "POST" && PathMatches(path, "/api/v1/topology/two_way_edge")) {
      return HandleBlockEdge(req, "two_way");
    }
    if (method == "POST" && PathMatches(path, "/api/v1/topology/hot_update")) {
      return HandleTopologyHotUpdate(req, false);
    }
    if (method == "POST" && PathMatches(path, "/api/v1/topology/preview")) {
      // 预演恒为 dry_run：客户端即使写 dry_run:false 也不能借此落盘
      return HandleTopologyHotUpdate(req, true);
    }

    resp.status = "404 Not Found";
    resp.body = JsonErrorBody("not_found");
    return resp;
  }

  HttpResponse HandleForceClear(const HttpRequest& req) {
    HttpResponse resp;
    const std::string vehicle_id = JsonStringField(req.body, "vehicle_id");
    const std::string confirm_id = JsonStringField(req.body, "confirm_vehicle_id");
    const std::string operator_id = JsonStringField(req.body, "operator_id");
    const std::string reason = JsonStringField(req.body, "reason");
    if (vehicle_id.empty() || confirm_id.empty() || operator_id.empty() || reason.empty()) {
      resp.status = "400 Bad Request";
      resp.body = "{\"error\":\"missing_fields\",\"detail\":\"vehicle_id, confirm_vehicle_id, "
                  "operator_id, reason are all required\"}";
      return resp;
    }
    auto request = std::make_shared<amr_dispatcher_interfaces::srv::ForceClearVda5050Link::Request>();
    request->vehicle_id = vehicle_id;
    request->confirm_vehicle_id = confirm_id;
    request->operator_id = operator_id;
    request->reason = reason;
    if (!force_clear_cli_->wait_for_service(1s)) {
      resp.status = "503 Service Unavailable";
      resp.body = JsonErrorBody("service_unavailable");
      return resp;
    }
    auto fut = force_clear_cli_->async_send_request(request);
    if (fut.wait_for(2s) != std::future_status::ready) {
      resp.status = "504 Gateway Timeout";
      resp.body = JsonErrorBody("timeout");
      return resp;
    }
    auto res = fut.get();
    std::string body = "{\"status\":\"" + std::string(res->success ? "ok" : "rejected") + "\",";
    body += JsonStringPair("message", res->message) + ",\"quarantined\":";
    body += (res->quarantined ? "true" : "false");
    body += ",\"released_resources\":[";
    for (std::size_t i = 0; i < res->released_resources.size(); ++i) {
      if (i) body += ",";
      body += "\"" + JsonEscape(res->released_resources[i]) + "\"";
    }
    body += "]}";
    resp.body = body;
    if (!res->success) {
      resp.status = "409 Conflict";
    }
    return resp;
  }

  HttpResponse HandleRestore(const HttpRequest& req) {
    HttpResponse resp;
    const std::string vehicle_id = JsonStringField(req.body, "vehicle_id");
    const std::string confirm_id = JsonStringField(req.body, "confirm_vehicle_id");
    const std::string operator_id = JsonStringField(req.body, "operator_id");
    const std::string reason = JsonStringField(req.body, "reason");
    const bool position_verified = JsonBoolField(req.body, "position_verified");
    if (vehicle_id.empty() || confirm_id.empty() || operator_id.empty() || reason.empty()) {
      resp.status = "400 Bad Request";
      resp.body = "{\"error\":\"missing_fields\",\"detail\":\"vehicle_id, confirm_vehicle_id, "
                  "operator_id, reason are all required\"}";
      return resp;
    }
    auto request = std::make_shared<amr_dispatcher_interfaces::srv::RestoreVda5050Link::Request>();
    request->vehicle_id = vehicle_id;
    request->confirm_vehicle_id = confirm_id;
    request->operator_id = operator_id;
    request->reason = reason;
    request->position_verified = position_verified;
    if (!restore_cli_->wait_for_service(1s)) {
      resp.status = "503 Service Unavailable";
      resp.body = JsonErrorBody("service_unavailable");
      return resp;
    }
    auto fut = restore_cli_->async_send_request(request);
    if (fut.wait_for(2s) != std::future_status::ready) {
      resp.status = "504 Gateway Timeout";
      resp.body = JsonErrorBody("timeout");
      return resp;
    }
    auto res = fut.get();
    resp.body = "{\"status\":\"" + std::string(res->success ? "ok" : "rejected") + "\"," +
                JsonStringPair("message", res->message) + "}";
    if (!res->success) {
      resp.status = "409 Conflict";
    }
    return resp;
  }

  // ===========================================================================
  // 相位 11：拓扑运维端点
  //
  // 全部状态码/错误码映射集中在 TopologyErrorStatus()，保证 REST 与 ROS 服务的
  // error_code 词表只有一处真源。请求处理沿用 HandleForceClear 骨架：
  // 必填字段 -> 400；服务不存在 -> 503；重规划超时 -> 504（N 车重规划确实慢）。
  // ===========================================================================

  // error_code -> HTTP 状态码（与 docs/API.md 的词表一一对应）。
  // 未知错误码一律 500：绝不把"不认识的失败"伪装成 200/409。
  static const char* TopologyErrorStatus(const std::string& error_code) {
    if (error_code == "ok" || error_code == "no_change") return "200 OK";
    if (error_code == "invalid_request" || error_code == "not_confirmed") {
      return "400 Bad Request";
    }
    if (error_code == "unknown_edge") return "404 Not Found";
    if (error_code == "map_hash_mismatch" || error_code == "would_disconnect_topology" ||
        error_code == "edge_occupied_in_flight" || error_code == "would_strand_vehicles" ||
        error_code == "lock_conflict" || error_code == "overlay_invalid") {
      return "409 Conflict";
    }
    if (error_code == "catalog_unavailable") return "503 Service Unavailable";
    // 功能未启用（topology_overlay_file 为空）。重试无意义，也不属于客户端写错，
    // 501 是唯一精确的答复：服务端不支持该功能（因部署配置关闭）。
    if (error_code == "overlay_disabled") return "501 Not Implemented";
    return "500 Internal Server Error";
  }

  // 幂等成功的 status 字面量必须是 "noop"（不是 "ok"）：
  // 控制台靠它区分"已生效"与"本来就是这样"，e2e 也据此断言重放不递增 revision。
  static std::string TopologyStatusLiteral(bool success, const std::string& error_code) {
    if (!success) return "rejected";
    return error_code == "no_change" ? "noop" : "ok";
  }

  static std::string JsonStringArray(const std::vector<std::string>& values) {
    std::string out = "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i) out += ",";
      out += "\"" + JsonEscape(values[i]) + "\"";
    }
    out += "]";
    return out;
  }

  // GET /api/v1/topology/overlay[?yaml=1]
  HttpResponse HandleGetTopologyOverlay(const HttpRequest& req) {
    HttpResponse resp;
    auto request =
        std::make_shared<amr_dispatcher_interfaces::srv::GetTopologyOverlay::Request>();
    // 控制台默认只要计数；?yaml=1 才要可编辑文本（YAML 可能很长）
    request->include_yaml = GetQueryParam(req.query, "yaml") == "1";
    if (!get_topology_overlay_cli_->wait_for_service(1s)) {
      resp.status = "503 Service Unavailable";
      resp.body = JsonErrorBody("service_unavailable");
      return resp;
    }
    auto fut = get_topology_overlay_cli_->async_send_request(request);
    if (fut.wait_for(std::chrono::milliseconds(topology_op_timeout_ms_)) !=
        std::future_status::ready) {
      resp.status = "504 Gateway Timeout";
      resp.body = JsonErrorBody("timeout");
      return resp;
    }
    auto res = fut.get();
    std::string body = "{" + JsonStringPair("status", res->success ? "ok" : "rejected") + ",";
    body += "\"overlay_active\":" + std::string(res->overlay_active ? "true" : "false") + ",";
    body += "\"revision\":" + std::to_string(res->revision) + ",";
    body += "\"blocked_edges\":" + std::to_string(res->blocked_edge_count) + ",";
    body += "\"one_way_edges\":" + std::to_string(res->one_way_edge_count) + ",";
    body += "\"markers\":" + std::to_string(res->marker_count) + ",";
    body += JsonStringPair("base_map_hash", res->base_map_hash) + ",";
    body += JsonStringPair("effective_map_hash", res->effective_map_hash) + ",";
    body += JsonStringPair("message", res->message);
    if (request->include_yaml) {
      body += "," + JsonStringPair("overlay_yaml", res->overlay_yaml);
    }
    body += "}";
    resp.body = body;
    if (!res->success) {
      resp.status = "409 Conflict";
    }
    return resp;
  }

  // POST /api/v1/topology/block_edge 与 /unblock_edge
  // action 由路径决定（客户端不用重复填，也不会填错）
  HttpResponse HandleBlockEdge(const HttpRequest& req, const std::string& action) {
    HttpResponse resp;
    const std::string from_station = JsonStringField(req.body, "from_station");
    const std::string to_station = JsonStringField(req.body, "to_station");
    const std::string operator_id = JsonStringField(req.body, "operator_id");
    const std::string reason = JsonStringField(req.body, "reason");
    const std::string direction = JsonStringField(req.body, "direction");
    if (from_station.empty() || to_station.empty() || operator_id.empty() ||
        reason.empty()) {
      resp.status = "400 Bad Request";
      resp.body = "{\"error\":\"missing_fields\",\"detail\":\"from_station, to_station, "
                  "operator_id, reason are all required\"}";
      return resp;
    }
    if (!JsonBoolField(req.body, "confirm")) {
      resp.status = "400 Bad Request";
      resp.body = "{\"error\":\"not_confirmed\",\"detail\":\"confirm must be true\"}";
      return resp;
    }
    auto request = std::make_shared<amr_dispatcher_interfaces::srv::BlockEdge::Request>();
    request->from_station = from_station;
    request->to_station = to_station;
    request->action = action;
    request->direction = direction.empty() ? "both" : direction;
    request->operator_id = operator_id;
    request->reason = reason;
    request->expected_map_hash = JsonStringField(req.body, "expected_map_hash");
    request->allow_edge_under_vehicle =
        JsonBoolField(req.body, "allow_edge_under_vehicle");
    request->confirm = true;
    if (!block_edge_cli_->wait_for_service(1s)) {
      resp.status = "503 Service Unavailable";
      resp.body = JsonErrorBody("service_unavailable");
      return resp;
    }
    auto fut = block_edge_cli_->async_send_request(request);
    if (fut.wait_for(std::chrono::milliseconds(topology_op_timeout_ms_)) !=
        std::future_status::ready) {
      resp.status = "504 Gateway Timeout";
      resp.body = JsonErrorBody("timeout");
      return resp;
    }
    auto res = fut.get();
    std::string body =
        "{" + JsonStringPair("status", TopologyStatusLiteral(res->success, res->error_code)) + ",";
    body += JsonStringPair("error_code", res->error_code) + ",";
    body += JsonStringPair("message", res->message) + ",";
    body += JsonStringPair("resource_id", res->resource_id) + ",";
    body += "\"impacted_vehicles\":" + JsonStringArray(res->impacted_vehicles) + ",";
    body += "\"impacted_missions\":" + JsonStringArray(res->impacted_missions) + ",";
    body += "\"rerouted\":" + std::string(res->rerouted ? "true" : "false") + ",";
    body += "\"overlay_revision\":" + std::to_string(res->overlay_revision) + "}";
    resp.status = TopologyErrorStatus(res->error_code);
    resp.body = body;
    return resp;
  }

  // POST /api/v1/topology/hot_update 与 /preview
  // dry_run 由路径决定：preview 恒为真，即使请求体里写了 dry_run:false
  HttpResponse HandleTopologyHotUpdate(const HttpRequest& req, bool force_dry_run) {
    HttpResponse resp;
    const std::string operator_id = JsonStringField(req.body, "operator_id");
    const std::string reason = JsonStringField(req.body, "reason");
    const bool dry_run = force_dry_run || JsonBoolField(req.body, "dry_run");
    const bool clear = JsonBoolField(req.body, "clear");
    if (!dry_run && (operator_id.empty() || reason.empty())) {
      resp.status = "400 Bad Request";
      resp.body = "{\"error\":\"missing_fields\",\"detail\":\"operator_id and reason are "
                  "required unless dry_run=true\"}";
      return resp;
    }
    if (!dry_run && !JsonBoolField(req.body, "confirm")) {
      resp.status = "400 Bad Request";
      resp.body = "{\"error\":\"not_confirmed\",\"detail\":\"confirm must be true when "
                  "dry_run=false\"}";
      return resp;
    }
    auto request =
        std::make_shared<amr_dispatcher_interfaces::srv::TopologyHotUpdate::Request>();
    request->overlay_yaml = JsonStringField(req.body, "overlay_yaml");
    request->expected_map_hash = JsonStringField(req.body, "expected_map_hash");
    request->operator_id = operator_id;
    request->reason = reason;
    request->dry_run = dry_run;
    request->clear = clear;
    request->allow_edge_under_vehicle = JsonBoolField(req.body, "allow_edge_under_vehicle");
    request->confirm = JsonBoolField(req.body, "confirm");
    if (!topology_hot_update_cli_->wait_for_service(1s)) {
      resp.status = "503 Service Unavailable";
      resp.body = JsonErrorBody("service_unavailable");
      return resp;
    }
    auto fut = topology_hot_update_cli_->async_send_request(request);
    if (fut.wait_for(std::chrono::milliseconds(topology_op_timeout_ms_)) !=
        std::future_status::ready) {
      resp.status = "504 Gateway Timeout";
      resp.body = JsonErrorBody("timeout");
      return resp;
    }
    auto res = fut.get();
    std::string body =
        "{" + JsonStringPair("status", TopologyStatusLiteral(res->success, res->error_code)) + ",";
    body += JsonStringPair("error_code", res->error_code) + ",";
    body += JsonStringPair("message", res->message) + ",";
    body += "\"dry_run\":" + std::string(dry_run ? "true" : "false") + ",";
    body += "\"unreachable_nodes\":" + JsonStringArray(res->unreachable_nodes) + ",";
    body += "\"impacted_vehicles\":" + JsonStringArray(res->impacted_vehicles) + ",";
    body += "\"impacted_missions\":" + JsonStringArray(res->impacted_missions) + ",";
    body += "\"reroute_failures\":" + JsonStringArray(res->reroute_failures) + ",";
    body += JsonStringPair("catalog_map_hash", res->catalog_map_hash) + ",";
    body += "\"overlay_revision\":" + std::to_string(res->overlay_revision) + ",";
    body += "\"file_written\":" + std::string(res->file_written ? "true" : "false") + "}";
    resp.status = TopologyErrorStatus(res->error_code);
    resp.body = body;
    return resp;
  }

  HttpResponse HandleReadiness() {
    HttpResponse resp;
    if (!readiness_cli_->wait_for_service(500ms)) {
      resp.status = "503 Service Unavailable";
      resp.body = "{\"ready\":false,\"state\":\"INITIALIZING\","
                  "\"message\":\"Lifecycle Coordinator not ready\"}";
      return resp;
    }
    auto request = std::make_shared<amr_dispatcher_interfaces::srv::GetSystemReadiness::Request>();
    auto fut = readiness_cli_->async_send_request(request);
    if (fut.wait_for(1s) != std::future_status::ready) {
      resp.status = "504 Gateway Timeout";
      resp.body = JsonErrorBody("readiness_probe_timeout");
      return resp;
    }
    auto res = fut.get();
    resp.body = "{\"ready\":" + std::string(res->ready ? "true" : "false") +
                ",\"healthy\":" + std::string(res->healthy ? "true" : "false") + "," +
                JsonStringPair("state", res->state) + "," +
                JsonStringPair("message", res->message) + "}";
    if (!res->ready) {
      resp.status = "503 Service Unavailable";
    }
    return resp;
  }

  HttpResponse HandleSubmitOrder(const HttpRequest& req) {
    HttpResponse resp;
    if (req.query.empty()) {
      resp.status = "400 Bad Request";
      resp.body = JsonErrorBody("missing_query_parameters");
      return resp;
    }
    const std::string mid = GetQueryParam(req.query, "id");
    const std::string from = GetQueryParam(req.query, "from");
    const std::string to = GetQueryParam(req.query, "to");
    const std::string vehicle = GetQueryParam(req.query, "vehicle");
    const std::string ikey = GetQueryParam(req.query, "idempotency_key");
    // 批次 6.4：补齐上游契约字段透传（此前网关只传 id/from/to/prio/vehicle，
    // 业务订单号、截止期、任务类型在网关这一层被静默丢弃）。
    const std::string order_id = GetQueryParam(req.query, "order_id");
    const std::string mission_type = GetQueryParam(req.query, "type");
    const std::string deadline_text = GetQueryParam(req.query, "deadline_unix_ms");
    const std::string expected_text = GetQueryParam(req.query, "expected_seconds");

    std::int32_t prio = 5;
    const std::string prio_text = GetQueryParam(req.query, "prio", "5");
    switch (ParseInt32Strict(prio_text, &prio)) {
      case IntParseStatus::kOk:
        break;
      case IntParseStatus::kOutOfRange:
        resp.status = "400 Bad Request";
        resp.body = JsonErrorBody("invalid_priority", "prio out of range");
        return resp;
      case IntParseStatus::kInvalid:
        // 旧实现 std::stoi("notanumber") 直接 terminate 整个网关
        resp.status = "400 Bad Request";
        resp.body = JsonErrorBody("invalid_priority", "prio must be an integer");
        return resp;
    }

    // 可选数值字段：给了就必须是合法整数（绝不静默当 0——那会让任务"没有截止期"
    // 而调用方以为设上了）。截止期额外要求为正数。
    std::int64_t deadline_unix_ms = 0;
    if (!deadline_text.empty()) {
      std::int32_t parsed = 0;
      if (ParseInt32Strict(deadline_text, &parsed) != IntParseStatus::kOk || parsed <= 0) {
        resp.status = "400 Bad Request";
        resp.body = JsonErrorBody("invalid_deadline", "deadline_unix_ms must be a positive integer");
        return resp;
      }
      deadline_unix_ms = parsed;
    }
    std::int32_t expected_seconds = 0;
    if (!expected_text.empty()) {
      if (ParseInt32Strict(expected_text, &expected_seconds) != IntParseStatus::kOk ||
          expected_seconds < 0) {
        resp.status = "400 Bad Request";
        resp.body = JsonErrorBody("invalid_expected_seconds",
                                  "expected_seconds must be a non-negative integer");
        return resp;
      }
    }

    // 幂等：先占位，只有后端**确认受理**才 Commit；失败/超时 Abort 允许重试
    bool reserved = false;
    if (!ikey.empty()) {
      const auto result = ReserveIdempotency(ikey);
      if (result == IdempotencyStore::ReserveResult::kDuplicate) {
        resp.body = "{\"status\":\"ignored\",\"reason\":\"idempotent_duplicate\"}";
        return resp;
      }
      if (result == IdempotencyStore::ReserveResult::kInvalidKey) {
        resp.status = "400 Bad Request";
        resp.body = JsonErrorBody("invalid_idempotency_key");
        return resp;
      }
      if (result == IdempotencyStore::ReserveResult::kFull) {
        resp.status = "503 Service Unavailable";
        resp.body = JsonErrorBody("idempotency_store_full");
        return resp;
      }
      reserved = true;
    }

    auto request = std::make_shared<amr_dispatcher_interfaces::srv::SubmitOrder::Request>();
    request->mission_id = mid;
    request->order_id = order_id;
    request->mission_type = mission_type;
    request->pickup_station = from;
    request->dropoff_station = to;
    request->priority = prio;
    request->deadline_unix_ms = deadline_unix_ms;
    request->expected_seconds = expected_seconds;
    request->assigned_vehicle_id = vehicle;

    if (!submit_cli_->wait_for_service(1s)) {
      if (reserved) AbortIdempotency(ikey);
      resp.status = "503 Service Unavailable";
      resp.body = JsonErrorBody("service_unavailable");
      return resp;
    }
    auto fut = submit_cli_->async_send_request(request);
    if (fut.wait_for(2s) != std::future_status::ready) {
      if (reserved) AbortIdempotency(ikey);
      resp.status = "504 Gateway Timeout";
      resp.body = JsonErrorBody("timeout");
      return resp;
    }
    auto res = fut.get();
    // 后端给出了明确答复（受理或业务拒绝）就算消费掉该 key；只有传输失败才回滚
    if (reserved) CommitIdempotency(ikey);
    resp.body = "{\"status\":\"ok\",\"accepted\":" + std::string(res->accepted ? "true" : "false") +
                "," + JsonStringPair("mission_id", res->mission_id) + "}";
    return resp;
  }

  HttpResponse HandleCancel(const HttpRequest& req) {
    HttpResponse resp;
    const std::string mid = GetQueryParam(req.query, "id");
    // 批次 6.4：整单取消。`CancelOrder.srv` 的契约是"mission_id 为空则按
    // order_id 取消整单"，但网关此前只传 mission_id，整单取消在 REST 层不可达。
    const std::string order_id = GetQueryParam(req.query, "order_id");
    if (mid.empty() && order_id.empty()) {
      resp.status = "400 Bad Request";
      resp.body = JsonErrorBody("missing_identifier", "either id or order_id is required");
      return resp;
    }
    auto request = std::make_shared<amr_dispatcher_interfaces::srv::CancelOrder::Request>();
    request->mission_id = mid;
    request->order_id = order_id;
    if (!cancel_cli_->wait_for_service(1s)) {
      resp.status = "503 Service Unavailable";
      resp.body = JsonErrorBody("service_unavailable");
      return resp;
    }
    auto fut = cancel_cli_->async_send_request(request);
    if (fut.wait_for(2s) != std::future_status::ready) {
      resp.status = "504 Gateway Timeout";
      resp.body = JsonErrorBody("timeout");
      return resp;
    }
    auto res = fut.get();
    resp.body = "{\"status\":\"ok\",\"canceled\":" + std::string(res->success ? "true" : "false") +
                ",\"canceled_count\":" + std::to_string(res->canceled_count) + "}";
    return resp;
  }

  // ---------------------------------------------------------------------------
  // 一键热上传与地图骨架拓扑生成
  // ---------------------------------------------------------------------------
  HttpResponse HandleMapUpload(const HttpRequest& req) {
    HttpResponse resp;
    const std::string& body = req.body;

    auto find_value_pos = [&body](const std::string& key) -> std::size_t {
      const auto pos = body.find("\"" + key + "\"");
      if (pos == std::string::npos) return std::string::npos;
      const auto colon = body.find(':', pos);
      if (colon == std::string::npos) return std::string::npos;
      return colon + 1;
    };
    auto get_int = [&](const std::string& key, long long def) -> long long {
      const auto pos = find_value_pos(key);
      if (pos == std::string::npos) return def;
      const long long value = SafeScanInt(body, pos);
      return value == 0 ? def : value;
    };
    auto get_double = [&](const std::string& key, double def) -> double {
      const auto pos = find_value_pos(key);
      if (pos == std::string::npos) return def;
      const double value = SafeScanDouble(body, pos);
      return value == 0.0 ? def : value;
    };

    const long long width = get_int("width", 0);
    const long long height = get_int("height", 0);
    const double res = get_double("resolution", 0.05);
    const double origin_x = get_double("origin_x", 0.0);
    const double origin_y = get_double("origin_y", 0.0);

    if (width <= 0 || height <= 0) {
      resp.status = "400 Bad Request";
      resp.body = JsonErrorBody("invalid_dimensions");
      return resp;
    }
    // 尺寸上限：width*height 会直接决定内存分配，不能让请求方随便点
    const long long cells = width * height;
    if (cells <= 0 || cells > 4000000LL) {  // 4M 格 ≈ 2000x2000
      resp.status = "413 Payload Too Large";
      resp.body = JsonErrorBody("grid_too_large");
      return resp;
    }

    amr_dispatcher_core::planning::MapGrid grid;
    grid.width = static_cast<uint32_t>(width);
    grid.height = static_cast<uint32_t>(height);
    grid.resolution = res;
    grid.origin_x = origin_x;
    grid.origin_y = origin_y;
    grid.origin_yaw = 0.0;
    grid.data.assign(static_cast<std::size_t>(cells), 0);  // 默认为空闲通行区

    const auto grid_pos = body.find("\"grid_data\"");
    if (grid_pos != std::string::npos) {
      const auto arr_start = body.find('[', grid_pos);
      const auto arr_end = body.find(']', arr_start);
      if (arr_start != std::string::npos && arr_end != std::string::npos) {
        const std::string arr_str = body.substr(arr_start + 1, arr_end - arr_start - 1);
        std::stringstream ss(arr_str);
        std::string item;
        std::size_t idx = 0;
        while (std::getline(ss, item, ',') && idx < grid.data.size()) {
          // 解析失败按 0（空闲）处理：旧实现对异常是 try/catch 吞掉
          grid.data[idx++] = static_cast<int8_t>(SafeScanInt(item, 0));
        }
      }
    }

    std::vector<amr_dispatcher_core::planning::AnchorStation> anchors;
    const auto anchors_pos = body.find("\"anchors\"");
    if (anchors_pos != std::string::npos) {
      const auto arr_start = body.find('[', anchors_pos);
      const auto arr_end = body.find(']', arr_start);
      if (arr_start != std::string::npos && arr_end != std::string::npos) {
        const std::string arr_str = body.substr(arr_start + 1, arr_end - arr_start - 1);
        std::size_t obj_start = 0;
        while ((obj_start = arr_str.find('{', obj_start)) != std::string::npos) {
          const std::size_t obj_end = arr_str.find('}', obj_start);
          if (obj_end == std::string::npos) break;
          const std::string obj_str = arr_str.substr(obj_start, obj_end - obj_start + 1);

          auto extract_str = [&obj_str](const std::string& k) -> std::string {
            const auto p = obj_str.find("\"" + k + "\"");
            if (p == std::string::npos) return "";
            const auto colon = obj_str.find(':', p);
            if (colon == std::string::npos) return "";
            const auto q1 = obj_str.find('\"', colon);
            if (q1 == std::string::npos) return "";
            const auto q2 = obj_str.find('\"', q1 + 1);
            if (q2 == std::string::npos) return "";
            return obj_str.substr(q1 + 1, q2 - q1 - 1);
          };
          auto extract_num = [&obj_str](const std::string& k) -> double {
            const auto p = obj_str.find("\"" + k + "\"");
            if (p == std::string::npos) return 0.0;
            const auto colon = obj_str.find(':', p);
            if (colon == std::string::npos) return 0.0;
            return SafeScanDouble(obj_str, colon + 1);
          };

          amr_dispatcher_core::planning::AnchorStation a;
          a.id = extract_str("id");
          a.type = extract_str("type");
          if (a.type.empty()) a.type = "normal";
          a.x = extract_num("x");
          a.y = extract_num("y");
          a.yaw = extract_num("yaw");
          if (!a.id.empty()) {
            anchors.push_back(a);
          }
          obj_start = obj_end + 1;
        }
      }
    }

    // 核心库自动化流水线：腐蚀 -> 细化 -> 剪枝 -> 拓扑构建
    amr_dispatcher_core::planning::SafetyFilterConfig sf_cfg;
    sf_cfg.robot_radius_m = 0.25;
    sf_cfg.safety_margin_m = 0.05;
    const auto safe_grid = amr_dispatcher_core::planning::FilterSafetySpace(grid, sf_cfg);
    const auto skel = amr_dispatcher_core::planning::ZhangSuenThinning(safe_grid);

    std::vector<amr_dispatcher_core::planning::PixelCoord> protected_points;
    for (const auto& a : anchors) {
      const amr_dispatcher_core::planning::WorldCoord w{a.x, a.y};
      protected_points.push_back(amr_dispatcher_core::planning::WorldToPixel(w, grid));
    }
    amr_dispatcher_core::planning::PruneConfig pr_cfg;
    pr_cfg.min_branch_length_m = 0.5;
    pr_cfg.resolution = res;
    const auto pruned =
        amr_dispatcher_core::planning::PruneSkeletonProtected(skel, protected_points, pr_cfg);

    amr_dispatcher_core::planning::TopologyBuilderConfig tb_cfg;
    tb_cfg.max_snap_dist_m = 3.0;
    tb_cfg.map_hash = "web_upload_" + std::to_string(width) + "x" + std::to_string(height);
    tb_cfg.graph_version = "v1_web";
    const auto build_res =
        amr_dispatcher_core::planning::BuildTopologyCatalog(pruned, grid, anchors, tb_cfg);

    if (!build_res.success) {
      resp.status = "422 Unprocessable Entity";
      resp.body = "{\"status\":\"error\"," + JsonStringPair("message", build_res.error_message) + "}";
      return resp;
    }

    // 写入 config/generated/stations.yaml (tmp + rename 原子替换)
    const std::filesystem::path final_path("config/generated/stations.yaml");
    std::filesystem::create_directories(final_path.parent_path());
    const std::filesystem::path tmp_path("config/generated/stations.yaml.tmp");
    {
      std::ofstream out(tmp_path);
      out << "version:\n";
      out << "  map_hash: \"" << build_res.catalog.map_hash << "\"\n";
      out << "  graph_version: \"" << build_res.catalog.graph_version << "\"\n";
      out << "stations:\n";
      for (const auto& s : build_res.catalog.stations) {
        out << "  - id: \"" << s.id << "\"\n";
        out << "    x: " << s.x << "\n";
        out << "    y: " << s.y << "\n";
        out << "    yaw: " << s.yaw << "\n";
        out << "    type: \"" << s.type << "\"\n";
      }
      out << "road_nodes:\n";
      for (const auto& rn : build_res.catalog.road_nodes) {
        out << "  - id: \"" << rn.id << "\"\n";
        out << "    x: " << rn.x << "\n";
        out << "    y: " << rn.y << "\n";
        out << "    yaw: " << rn.yaw << "\n";
      }
      out << "edges:\n";
      for (const auto& e : build_res.catalog.edges) {
        out << "  - from: \"" << e.from << "\"\n";
        out << "    to: \"" << e.to << "\"\n";
        out << "    distance_m: " << e.distance_m << "\n";
        out << "    bidirectional: " << (e.bidirectional ? "true" : "false") << "\n";
        out << "    enabled: true\n";
        out << "    polyline:\n";
        for (const auto& pt : e.polyline) {
          out << "      - [" << pt.first << ", " << pt.second << "]\n";
        }
      }
      out.close();
    }
    std::error_code rename_ec;
    std::filesystem::rename(tmp_path, final_path, rename_ec);
    if (rename_ec) {
      resp.status = "500 Internal Server Error";
      resp.body = JsonErrorBody("stations_write_failed", rename_ec.message());
      return resp;
    }

    resp.body = "{\"status\":\"ok\",\"stations_count\":" +
                std::to_string(build_res.catalog.stations.size()) +
                ",\"road_nodes_count\":" + std::to_string(build_res.catalog.road_nodes.size()) +
                ",\"edges_count\":" + std::to_string(build_res.catalog.edges.size()) + "," +
                JsonStringPair("map_hash", build_res.catalog.map_hash) + "}";
    return resp;
  }

  // ---------------------------------------------------------------------------
  // 响应序列化
  // ---------------------------------------------------------------------------
  std::string ExtraHeaders() const {
    if (cors_allow_origin_.empty()) {
      return "";
    }
    std::string headers;
    headers += "Access-Control-Allow-Origin: " + cors_allow_origin_ + "\r\n";
    headers += "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n";
    headers += "Access-Control-Allow-Headers: Authorization, Content-Type\r\n";
    return headers;
  }

  std::string SerializeResponse(const HttpResponse& resp, bool with_length) const {
    std::ostringstream os;
    os << "HTTP/1.1 " << resp.status << "\r\n";
    if (!resp.content_type.empty()) {
      os << "Content-Type: " << resp.content_type << "\r\n";
    }
    if (with_length) {
      os << "Content-Length: " << resp.body.size() << "\r\n";
    }
    os << ExtraHeaders();
    os << "Connection: close\r\n\r\n";
    os << resp.body;
    return os.str();
  }

  // ---------------------------------------------------------------------------
  // 数据读取与格式化
  // ---------------------------------------------------------------------------
  std::string ReadStaticHtml() {
    std::ifstream file(html_path_);
    if (file.is_open()) {
      return std::string((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
    }

    // 1. 尝试当前工作区 tools/ 相对目录
    std::ifstream fallback("tools/operator_console.html");
    if (fallback.is_open()) {
      return std::string((std::istreambuf_iterator<char>(fallback)),
                         std::istreambuf_iterator<char>());
    }

    // 2. 尝试容器内 /workspace/src/amr_dispatcher_all/tools/operator_console.html
    std::ifstream container_file("/workspace/src/amr_dispatcher_all/tools/operator_console.html");
    if (container_file.is_open()) {
      return std::string((std::istreambuf_iterator<char>(container_file)),
                         std::istreambuf_iterator<char>());
    }

    // 3. 尝试 ament 包 share 路径
    try {
      const std::string share_dir = ament_index_cpp::get_package_share_directory("amr_dispatcher_tools");
      std::ifstream share_file(share_dir + "/tools/operator_console.html");
      if (share_file.is_open()) {
        return std::string((std::istreambuf_iterator<char>(share_file)),
                           std::istreambuf_iterator<char>());
      }
    } catch (...) {
      // 忽略 ament 寻找异常
    }

    return "";
  }

  static std::string GetQueryParam(const std::string& query, const std::string& key,
                                   const std::string& fallback = "") {
    const std::string target = key + "=";
    auto pos = query.find(target);
    // 只认 "key="（前面必须是查询串开头或 '&'），避免 "xid=" 命中 "id="
    while (pos != std::string::npos) {
      if (pos == 0 || query[pos - 1] == '&') {
        break;
      }
      pos = query.find(target, pos + 1);
    }
    if (pos == std::string::npos) return fallback;
    const auto amp = query.find('&', pos);
    if (amp == std::string::npos) {
      return query.substr(pos + target.size());
    }
    return query.substr(pos + target.size(), amp - (pos + target.size()));
  }

  // 极简 JSON 字符串字段提取（请求体由本项目自己的控制台产生，格式可控）：
  // 形如 "key": "value" -> value；缺失或非字符串返回空串。
  static std::string JsonStringField(const std::string& body, const std::string& key) {
    const std::string target = "\"" + key + "\"";
    auto pos = body.find(target);
    if (pos == std::string::npos) return "";
    pos = body.find(':', pos);
    if (pos == std::string::npos) return "";
    ++pos;
    while (pos < body.size() && std::isspace(static_cast<unsigned char>(body[pos]))) ++pos;
    if (pos >= body.size() || body[pos] != '"') return "";
    ++pos;
    std::string value;
    while (pos < body.size() && body[pos] != '"') {
      if (body[pos] == '\\' && pos + 1 < body.size()) {
        ++pos;
        switch (body[pos]) {
          case 'n': value.push_back('\n'); break;
          case 't': value.push_back('\t'); break;
          case 'r': value.push_back('\r'); break;
          default: value.push_back(body[pos]); break;
        }
        ++pos;
        continue;
      }
      value.push_back(body[pos]);
      ++pos;
    }
    return value;
  }

  // 极简 JSON 布尔字段提取：true / false。
  static bool JsonBoolField(const std::string& body, const std::string& key) {
    const std::string target = "\"" + key + "\"";
    auto pos = body.find(target);
    if (pos == std::string::npos) return false;
    pos = body.find(':', pos);
    if (pos == std::string::npos) return false;
    ++pos;
    while (pos < body.size() && std::isspace(static_cast<unsigned char>(body[pos]))) ++pos;
    return body.compare(pos, 4, "true") == 0;
  }

  // ---------------------------------------------------------------------------
  // /metrics：Prometheus 文本格式（对外契约，渲染逻辑在 metrics_format.hpp 单测）
  // ---------------------------------------------------------------------------
  std::string RenderMetrics() {
    using amr_dispatcher_tools::MetricFamily;
    using amr_dispatcher_tools::MetricSample;
    using amr_dispatcher_tools::MetricType;

    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<MetricFamily> families;

    // ---- 健康面（相位 11）----
    // 这些族**无条件**产生样本（0 而不是省略）：check_metrics_e2e.sh 断言"无空指标族"，
    // 而抓取端也依赖"指标存在且为 0"来区分"没告警"与"没有这条指标"。
    MetricFamily ready;
    ready.name = "amr_ready";
    ready.help = "1 when every monitored component is fresh and healthy (same verdict as /readyz)";
    ready.type = MetricType::kGauge;
    MetricFamily component_ok;
    component_ok.name = "amr_ready_component";
    component_ok.help = "Per-component readiness (1 = fresh and healthy, 0 = not)";
    component_ok.type = MetricType::kGauge;
    for (const auto& probe : CollectComponentProbesLocked()) {
      component_ok.samples.push_back(MetricSample{
          {{"component", probe.name}},
          EvaluateComponent(probe) == ComponentStatus::kOk ? 1.0 : 0.0});
    }
    ready.samples.push_back(
        MetricSample{{}, EvaluateReadiness(CollectComponentProbesLocked()).ready ? 1.0 : 0.0});
    families.push_back(std::move(ready));
    families.push_back(std::move(component_ok));

    MetricFamily dispatcher_age;
    dispatcher_age.name = "amr_dispatcher_state_age_seconds";
    dispatcher_age.help = "Seconds since the last /dispatcher/state message (-1 = never seen)";
    dispatcher_age.type = MetricType::kGauge;
    {
      const double age = state_seen_
                             ? std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                             last_state_arrival_)
                                   .count()
                             : -1.0;
      dispatcher_age.samples.push_back(MetricSample{{}, age});
    }
    families.push_back(std::move(dispatcher_age));

    MetricFamily severity_level;
    severity_level.name = "amr_safety_severity_level";
    severity_level.help =
        "Safety severity level (0=INFO, 1=WARN, 2=CRITICAL, -1=no safety state seen)";
    severity_level.type = MetricType::kGauge;
    {
      double level = -1.0;
      if (safety_seen_) {
        if (latest_safety_.severity == "CRITICAL") {
          level = 2.0;
        } else if (latest_safety_.severity == "WARN") {
          level = 1.0;
        } else {
          level = 0.0;
        }
      }
      severity_level.samples.push_back(MetricSample{{}, level});
    }
    families.push_back(std::move(severity_level));

    MetricFamily safety_critical;
    safety_critical.name = "amr_safety_critical";
    safety_critical.help = "1 when the safety gate reports CRITICAL, else 0";
    safety_critical.type = MetricType::kGauge;
    safety_critical.samples.push_back(
        MetricSample{{}, (safety_seen_ && latest_safety_.severity == "CRITICAL") ? 1.0 : 0.0});
    families.push_back(std::move(safety_critical));

    // ---- 拓扑叠加层（相位 11）：标签绝不带边名，否则基数与地图规模成正比 ----
    MetricFamily overlay_active;
    overlay_active.name = "amr_topology_overlay_active";
    overlay_active.help = "1 when a runtime topology overlay (temporary blocks / one-way) is applied";
    overlay_active.type = MetricType::kGauge;
    overlay_active.samples.push_back(MetricSample{{}, topology_overlay_active_ ? 1.0 : 0.0});
    families.push_back(std::move(overlay_active));

    MetricFamily overlay_revision;
    overlay_revision.name = "amr_topology_overlay_revision";
    overlay_revision.help = "Revision counter of the applied topology overlay (0 = none)";
    overlay_revision.type = MetricType::kGauge;
    overlay_revision.samples.push_back(
        MetricSample{{}, static_cast<double>(topology_overlay_revision_)});
    families.push_back(std::move(overlay_revision));

    MetricFamily blocked_edges;
    blocked_edges.name = "amr_topology_blocked_edges";
    blocked_edges.help = "Number of edges disabled by the runtime topology overlay";
    blocked_edges.type = MetricType::kGauge;
    blocked_edges.samples.push_back(
        MetricSample{{}, static_cast<double>(topology_blocked_edges_)});
    families.push_back(std::move(blocked_edges));

    MetricFamily one_way_edges;
    one_way_edges.name = "amr_topology_one_way_edges";
    one_way_edges.help = "Number of edges restricted to one direction by the overlay";
    one_way_edges.type = MetricType::kGauge;
    one_way_edges.samples.push_back(
        MetricSample{{}, static_cast<double>(topology_one_way_edges_)});
    families.push_back(std::move(one_way_edges));

    // ---- 告警通道自身的健康：告警发不出去必须能被抓取侧发现 ----
    MetricFamily alert_sent;
    alert_sent.name = "amr_alert_sent_total";
    alert_sent.help = "Alert webhook deliveries that returned 2xx";
    alert_sent.type = MetricType::kCounter;
    alert_sent.samples.push_back(MetricSample{{}, static_cast<double>(alert_sent_.load())});
    families.push_back(std::move(alert_sent));

    MetricFamily alert_failures;
    alert_failures.name = "amr_alert_webhook_failures_total";
    alert_failures.help = "Alert webhook deliveries that failed (non-2xx, timeout, connect error)";
    alert_failures.type = MetricType::kCounter;
    alert_failures.samples.push_back(
        MetricSample{{}, static_cast<double>(alert_failures_.load())});
    families.push_back(std::move(alert_failures));

    MetricFamily alert_dropped;
    alert_dropped.name = "amr_alert_dropped_total";
    alert_dropped.help = "Alerts dropped because the send queue was full (oldest dropped first)";
    alert_dropped.type = MetricType::kCounter;
    alert_dropped.samples.push_back(
        MetricSample{{}, static_cast<double>(alert_dropped_.load())});
    families.push_back(std::move(alert_dropped));

    MetricFamily queue;
    queue.name = "amr_queue_size";
    queue.help = "Missions currently waiting in the dispatch queue";
    queue.type = MetricType::kGauge;
    queue.samples.push_back(MetricSample{{}, static_cast<double>(latest_state_.queue_size)});
    families.push_back(std::move(queue));

    MetricFamily capacity;
    capacity.name = "amr_queue_capacity";
    capacity.help = "Configured mission queue capacity";
    capacity.type = MetricType::kGauge;
    capacity.samples.push_back(
        MetricSample{{}, static_cast<double>(latest_state_.queue_capacity)});
    families.push_back(std::move(capacity));

    MetricFamily active;
    active.name = "amr_active_missions";
    active.help = "Missions currently executing";
    active.type = MetricType::kGauge;
    active.samples.push_back(
        MetricSample{{}, static_cast<double>(latest_state_.active_missions_count)});
    families.push_back(std::move(active));

    MetricFamily deadlock;
    deadlock.name = "amr_deadlock_active";
    deadlock.help = "1 when the dispatcher reports an active deadlock, else 0";
    deadlock.type = MetricType::kGauge;
    deadlock.samples.push_back(
        MetricSample{{}, latest_state_.deadlock_detected ? 1.0 : 0.0});
    families.push_back(std::move(deadlock));

    MetricFamily online;
    online.name = "amr_vehicle_online";
    online.help = "Per-vehicle link online flag (1 = online)";
    online.type = MetricType::kGauge;
    MetricFamily busy;
    busy.name = "amr_vehicle_busy";
    busy.help = "Per-vehicle busy flag (1 = executing a mission)";
    busy.type = MetricType::kGauge;
    MetricFamily quarantined;
    quarantined.name = "amr_vehicle_quarantined";
    quarantined.help = "Per-vehicle quarantine flag (1 = excluded from auto assignment)";
    quarantined.type = MetricType::kGauge;
    MetricFamily battery;
    battery.name = "amr_vehicle_battery_percent";
    battery.help = "Per-vehicle battery percentage";
    battery.type = MetricType::kGauge;
    for (const auto& link : latest_state_.vehicle_links) {
      const std::vector<std::pair<std::string, std::string>> labels{
          {"vehicle_id", link.vehicle_id}};
      online.samples.push_back(
          MetricSample{labels, link.online ? 1.0 : 0.0});
      busy.samples.push_back(MetricSample{labels, link.busy ? 1.0 : 0.0});
      quarantined.samples.push_back(MetricSample{labels, link.quarantined ? 1.0 : 0.0});
      battery.samples.push_back(
          MetricSample{labels, link.battery_percent});
    }
    families.push_back(std::move(online));
    families.push_back(std::move(busy));
    families.push_back(std::move(quarantined));
    families.push_back(std::move(battery));

    // 派发面计数器（来源：/dispatcher/state 的累积计数字段）
    MetricFamily dispatch_total;
    dispatch_total.name = "amr_dispatch_total";
    dispatch_total.help = "Missions dispatched so far (cumulative)";
    dispatch_total.type = MetricType::kCounter;
    dispatch_total.samples.push_back(
        MetricSample{{}, static_cast<double>(latest_state_.dispatch_total)});
    families.push_back(std::move(dispatch_total));

    MetricFamily dispatch_rejected;
    dispatch_rejected.name = "amr_dispatch_rejected_total";
    dispatch_rejected.help = "Missions permanently rejected by vehicle selection";
    dispatch_rejected.type = MetricType::kCounter;
    dispatch_rejected.samples.push_back(
        MetricSample{{}, static_cast<double>(latest_state_.dispatch_rejected_total)});
    families.push_back(std::move(dispatch_rejected));

    MetricFamily dispatch_deferred;
    dispatch_deferred.name = "amr_dispatch_deferred_total";
    dispatch_deferred.help = "Dispatch attempts that stayed queued (deferred)";
    dispatch_deferred.type = MetricType::kCounter;
    dispatch_deferred.samples.push_back(
        MetricSample{{}, static_cast<double>(latest_state_.dispatch_deferred_total)});
    families.push_back(std::move(dispatch_deferred));

    MetricFamily contention;
    contention.name = "amr_lock_contention_total";
    contention.help = "Route lock reservation failures (static and time-space)";
    contention.type = MetricType::kCounter;
    contention.samples.push_back(
        MetricSample{{}, static_cast<double>(latest_state_.lock_contention_total)});
    families.push_back(std::move(contention));

    MetricFamily events;
    events.name = "amr_event_total";
    events.help = "Mission events appended to the event log";
    events.type = MetricType::kCounter;
    events.samples.push_back(MetricSample{{}, static_cast<double>(latest_state_.event_total)});
    families.push_back(std::move(events));

    // 网关自身的请求计数（按状态码分标签；只为已发生的组合产生样本）
    MetricFamily requests;
    requests.name = "amr_gateway_requests_total";
    requests.help = "HTTP requests handled by the gateway, partitioned by status code";
    requests.type = MetricType::kCounter;
    for (const auto& [code, count] : request_counts_) {
      requests.samples.push_back(
          MetricSample{{{"code", code}}, static_cast<double>(count)});
    }
    families.push_back(std::move(requests));

    return amr_dispatcher_tools::RenderMetricFamilies(families);
  }

  std::string FormatVehicleLinksJson() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::ostringstream os;
    os << "{\"vehicle_links\":[";
    const auto& links = latest_state_.vehicle_links;
    for (std::size_t i = 0; i < links.size(); ++i) {
      const auto& v = links[i];
      if (i) os << ",";
      os << "{" << JsonStringPair("vehicle_id", v.vehicle_id) << ","
         << JsonStringPair("backend", v.backend) << ","
         << "\"online\":" << (v.online ? "true" : "false") << ","
         << "\"busy\":" << (v.busy ? "true" : "false") << ","
         << "\"quarantined\":" << (v.quarantined ? "true" : "false") << ","
         << "\"position_valid\":" << (v.position_valid ? "true" : "false") << ","
         << JsonStringPair("current_station_id", v.current_station_id) << ","
         << "\"battery_percent\":" << v.battery_percent << ","
         << "\"state_age_ms\":" << v.state_age_ms << ","
         << JsonStringPair("last_error", v.last_error) << "}";
    }
    os << "]}";
    return os.str();
  }

  std::string FormatSnapshotJson() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::ostringstream os;
    os << "{"
       << JsonStringPair("comparator", latest_state_.current_comparator) << ","
       << JsonStringPair("strategy", latest_state_.current_comparator) << ","
       << "\"queue_size\":" << latest_state_.queue_size << ","
       << "\"queue_capacity\":" << latest_state_.queue_capacity << ","
       << "\"active_count\":" << latest_state_.active_missions_count << ","
       << JsonStringPair("active_mission_id", latest_state_.active_mission_id) << ","
       << "\"deadlock\":" << (latest_state_.deadlock_detected ? "true" : "false") << ","
       << "\"deadlock_detected\":" << (latest_state_.deadlock_detected ? "true" : "false") << ","
       // 未收到任何 SafetyState 时不渲染急停（默认值会误报 estop）
       << "\"estop_active\":" << ((safety_seen_ && !latest_safety_.motion_allowed) ? "true" : "false") << ","
       << JsonStringPair("safety_severity",
                         latest_safety_.severity.empty() ? "INFO" : latest_safety_.severity) << ","
       << JsonStringPair("chassis_backend",
                         latest_link_.backend_name.empty() ? "Serial/Mock" : latest_link_.backend_name) << ","
       << "\"chassis_healthy\":" << (latest_link_.is_healthy ? "true" : "false") << ","
       << "\"chassis_loss_rate\":" << latest_link_.loss_rate << ","
       // T3：命令送达率。三种情况都渲染 null（不适用），绝不给 0：
       //   1) 从未收到 link_health —— 此时 command_delivery_rate 还是消息默认值 0.0，
       //      直接打印会显示成"0% 全丢"，这是最容易被误读的假象（和上面的
       //      estop_active 同一个坑：默认值不能当现状）；
       //   2) 文本协议，或固件尚未回显序号 —— chassis_driver 显式发 -1；
       //   3) 负数（协议外的异常值）。
       << "\"chassis_command_delivery_rate\":";
    if (link_seen_ && latest_link_.command_delivery_rate >= 0.0) {
      os << latest_link_.command_delivery_rate;
    } else {
      os << "null";
    }
    os << ",";
    os << "\"chassis_commands_sent\":" << latest_link_.commands_sent << ","
       << "\"chassis_commands_delivered\":" << latest_link_.commands_delivered << ","
       << "\"recent_events\":[";

    for (std::size_t i = 0; i < recent_events_.size(); ++i) {
      const auto& ev = recent_events_[i];
      os << "{" << JsonStringPair("mission_id", ev.mission_id) << ","
         << JsonStringPair("event", ev.event) << ","
         << JsonStringPair("reason", ev.reason) << "}";
      if (i + 1 < recent_events_.size()) os << ",";
    }
    os << "]}";
    return os.str();
  }

  // ---------------------------------------------------------------------------
  // 幂等键（多工作线程访问，需加锁）
  // ---------------------------------------------------------------------------
  static std::int64_t NowEpochSec() {
    return static_cast<std::int64_t>(std::time(nullptr));
  }

  IdempotencyStore::ReserveResult ReserveIdempotency(const std::string& key) {
    std::lock_guard<std::mutex> lock(idempotency_mutex_);
    return idempotency_.Reserve(key, NowEpochSec());
  }

  void CommitIdempotency(const std::string& key) {
    std::string text;
    {
      std::lock_guard<std::mutex> lock(idempotency_mutex_);
      idempotency_.Commit(key, NowEpochSec());
      if (idempotency_store_file_.empty()) {
        return;
      }
      idempotency_.PruneExpired(NowEpochSec());
      text = idempotency_.Serialize();
    }
    PersistIdempotency(text);
  }

  // 原子落盘：tmp + rename。写失败只告警——去重是尽力而为，不能因此拒绝已经
  // 被后端受理的订单（订单已经进队列了，返回错误反而误导调用方）。
  void PersistIdempotency(const std::string& text) {
    const std::filesystem::path final_path(idempotency_store_file_);
    const std::filesystem::path tmp_path(final_path.string() + ".tmp");
    std::error_code ec;
    if (final_path.has_parent_path()) {
      std::filesystem::create_directories(final_path.parent_path(), ec);
    }
    {
      std::ofstream out(tmp_path, std::ios::trunc);
      if (!out) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "Cannot open idempotency store temp file %s", tmp_path.c_str());
        return;
      }
      out << text;
      out.flush();
      if (!out) {
        out.close();
        std::filesystem::remove(tmp_path, ec);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "Failed writing idempotency store %s", tmp_path.c_str());
        return;
      }
    }
    std::filesystem::rename(tmp_path, final_path, ec);
    if (ec) {
      std::filesystem::remove(tmp_path, ec);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Atomic rename of idempotency store failed: %s", ec.message().c_str());
    }
  }

  void AbortIdempotency(const std::string& key) {
    std::lock_guard<std::mutex> lock(idempotency_mutex_);
    idempotency_.Abort(key);
  }

  int port_ = 8080;
  std::string html_path_{"tools/operator_console.html"};
  std::string auth_token_;
  std::string estop_token_;
  std::string cors_allow_origin_;
  int server_fd_ = -1;
  std::atomic<bool> running_{true};

  // 连接队列（主线程 accept / 工作线程消费）
  std::thread server_thread_;
  std::vector<std::thread> workers_;
  std::deque<QueuedConn> pending_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::size_t max_pending_ = 32;
  bool stopping_ = false;

  std::mutex data_mutex_;
  std::mutex idempotency_mutex_;
  IdempotencyStore idempotency_;
  std::string idempotency_store_file_;
  std::string metrics_token_;
  // /metrics 用：按 HTTP 状态码聚合的请求计数（只在 worker 线程写、读时持 data_mutex_）
  std::map<std::string, std::size_t> request_counts_;

  rclcpp::Client<amr_dispatcher_interfaces::srv::SubmitOrder>::SharedPtr submit_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::CancelOrder>::SharedPtr cancel_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::GetSystemReadiness>::SharedPtr readiness_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::ForceClearVda5050Link>::SharedPtr
      force_clear_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::RestoreVda5050Link>::SharedPtr restore_cli_;
  // 相位 11：拓扑运维（GET 状态 / 单边管制 / 全量叠加层）
  rclcpp::Client<amr_dispatcher_interfaces::srv::BlockEdge>::SharedPtr block_edge_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::TopologyHotUpdate>::SharedPtr
      topology_hot_update_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::GetTopologyOverlay>::SharedPtr
      get_topology_overlay_cli_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;

  rclcpp::Subscription<amr_dispatcher_interfaces::msg::DispatcherState>::SharedPtr state_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::ChassisLinkHealth>::SharedPtr link_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::MissionEvent>::SharedPtr event_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_flag_sub_;
  rclcpp::TimerBase::SharedPtr alert_timer_;

  amr_dispatcher_interfaces::msg::DispatcherState latest_state_;
  amr_dispatcher_interfaces::msg::SafetyState latest_safety_;
  amr_dispatcher_interfaces::msg::ChassisLinkHealth latest_link_;
  std::vector<amr_dispatcher_interfaces::msg::MissionEvent> recent_events_;
  bool safety_seen_ = false;

  // ---- 相位 11：探针与告警 ----
  const std::chrono::steady_clock::time_point started_at_ = std::chrono::steady_clock::now();
  // 各组件消息的到达时刻与"是否见过"。全部在 data_mutex_ 下读写。
  bool state_seen_ = false;
  bool link_seen_ = false;
  bool ready_flag_seen_ = false;
  bool ready_flag_ = false;
  bool safety_critical_ = false;
  std::chrono::steady_clock::time_point last_state_arrival_{};
  std::chrono::steady_clock::time_point last_safety_arrival_{};
  std::chrono::steady_clock::time_point last_link_arrival_{};
  std::chrono::steady_clock::time_point last_ready_flag_arrival_{};
  // CRITICAL 起始时刻：0 = 当前不是 CRITICAL（判"持续多久"的计时起点）
  std::chrono::steady_clock::time_point safety_critical_since_{};

  double readyz_dispatcher_state_stale_sec_ = 3.0;
  double readyz_safety_stale_sec_ = 2.0;
  double readyz_chassis_stale_sec_ = 5.0;
  double readyz_coordinator_stale_sec_ = 3.0;
  bool readyz_require_chassis_ = true;

  // 拓扑叠加层状态（由 DispatcherState 带下来，供 /metrics 与告警详情）
  bool topology_overlay_active_ = false;
  std::uint32_t topology_overlay_revision_ = 0;
  std::uint32_t topology_blocked_edges_ = 0;
  std::uint32_t topology_one_way_edges_ = 0;

  // 告警：URL 为空 = 整条链路不存在（不起线程、不建定时器）
  ParsedUrl alert_url_;
  int alert_timeout_ms_ = 3000;
  int alert_probe_period_ms_ = 1000;
  int topology_op_timeout_ms_ = 10000;
  int ros_domain_id_ = 0;
  std::size_t alert_queue_capacity_ = 64;
  AlertStateMachine alerts_;
  std::deque<AlertJob> alert_queue_;
  std::mutex alert_mutex_;
  std::condition_variable alert_cv_;
  std::thread alert_thread_;
  bool alert_stopping_ = false;
  std::atomic<std::uint64_t> alert_sent_{0};
  std::atomic<std::uint64_t> alert_failures_{0};
  std::atomic<std::uint64_t> alert_dropped_{0};
};

}  // namespace amr_dispatcher_tools

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_tools::RestGatewayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
