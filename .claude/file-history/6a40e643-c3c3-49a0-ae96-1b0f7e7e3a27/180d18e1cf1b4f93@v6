#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cctype>
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

#include "amr_dispatcher_tools/bearer_auth.hpp"
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

    estop_pub_ = create_publisher<std_msgs::msg::Bool>("/safety/estop", 10);

    state_sub_ = create_subscription<amr_dispatcher_interfaces::msg::DispatcherState>(
        "/dispatcher/state", 10,
        [this](const amr_dispatcher_interfaces::msg::DispatcherState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_state_ = *msg;
        });

    safety_sub_ = create_subscription<amr_dispatcher_interfaces::msg::SafetyState>(
        "/safety/state", 10,
        [this](const amr_dispatcher_interfaces::msg::SafetyState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_safety_ = *msg;
          safety_seen_ = true;
        });

    link_sub_ = create_subscription<amr_dispatcher_interfaces::msg::ChassisLinkHealth>(
        "/chassis/link_health", 10,
        [this](const amr_dispatcher_interfaces::msg::ChassisLinkHealth::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_link_ = *msg;
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
      for (const int fd : pending_) {
        close(fd);  // 未处理的连接直接断开
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

      bool queued = false;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (pending_.size() < max_pending_) {
          pending_.push_back(client_fd);
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
      int client_fd = -1;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] { return stopping_ || !pending_.empty(); });
        if (stopping_ && pending_.empty()) {
          return;
        }
        client_fd = pending_.front();
        pending_.pop_front();
      }
      HandleClientSafely(client_fd);
      close(client_fd);
    }
  }

  // 任何异常都必须收敛成 400：旧实现里一个 stoi 抛异常就把整个进程 terminate。
  void HandleClientSafely(int client_fd) {
    try {
      HandleClient(client_fd);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "请求处理异常（已兜底为 400）: %s", e.what());
      HttpResponse resp;
      resp.status = "400 Bad Request";
      resp.body = JsonErrorBody("bad_request");
      RecordRequestStatus(resp.status);
      WriteAll(client_fd, SerializeResponse(resp, false));
    } catch (...) {
      RCLCPP_WARN(get_logger(), "请求处理未知异常（已兜底为 400）");
      HttpResponse resp;
      resp.status = "400 Bad Request";
      resp.body = JsonErrorBody("bad_request");
      RecordRequestStatus(resp.status);
      WriteAll(client_fd, SerializeResponse(resp, false));
    }
  }

  // ---------------------------------------------------------------------------
  // 请求读取：头部上限、请求体上限、Content-Length 严格解析、分块传输拒绝
  // ---------------------------------------------------------------------------
  enum class ReadStatus { kOk, kClosed, kBadRequest, kTooLarge };

  ReadStatus ReadRequest(int client_fd, HttpRequest* out) {
    std::string raw;
    raw.reserve(1024);
    char buffer[4096];

    // 1) 读到头部结束标记
    std::size_t header_end = std::string::npos;
    while (true) {
      header_end = raw.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        break;
      }
      if (raw.size() > kMaxHeaderBytes) {
        return ReadStatus::kBadRequest;
      }
      const ssize_t n = ::read(client_fd, buffer, sizeof(buffer));
      if (n == 0) {
        return ReadStatus::kClosed;
      }
      if (n < 0) {
        if (errno == EINTR) continue;
        return ReadStatus::kClosed;  // 超时(EAGAIN) 或对端重置
      }
      raw.append(buffer, static_cast<std::size_t>(n));
    }
    if (header_end > kMaxHeaderBytes) {
      return ReadStatus::kBadRequest;  // 头部过大
    }
    const std::string head = raw.substr(0, header_end);

    // 2) 请求行
    std::istringstream first_line(head.substr(0, head.find("\r\n")));
    std::string version;
    first_line >> out->method >> out->target >> version;
    if (out->method.empty() || out->target.empty()) {
      return ReadStatus::kBadRequest;
    }
    out->path = RequestPathOnly(out->target);
    out->query = RequestQueryOnly(out->target);
    FindHeaderValue(head, "Authorization", &out->authorization);

    // 3) 请求体长度：分块传输不支持（猜长度 = 读到连接耗尽，直接拒绝）
    if (HasChunkedTransferEncoding(head)) {
      return ReadStatus::kBadRequest;
    }
    std::size_t content_length = 0;
    std::string cl_value;
    if (FindHeaderValue(head, "Content-Length", &cl_value)) {
      switch (ParseContentLength(cl_value, &content_length)) {
        case ContentLengthStatus::kOk:
          break;
        case ContentLengthStatus::kMissing:
          content_length = 0;
          break;
        case ContentLengthStatus::kTooLarge:
          return ReadStatus::kTooLarge;
        case ContentLengthStatus::kInvalid:
          return ReadStatus::kBadRequest;
      }
    }

    // 4) 读满请求体（总字节数同样受上限约束）
    const std::size_t body_begin = header_end + 4;
    std::size_t body_size = raw.size() - body_begin;
    if (body_size > content_length) {
      body_size = content_length;  // 多读的属于下一个请求（本网关一连接一请求，丢弃）
    }
    while (body_size < content_length) {
      const ssize_t n = ::read(client_fd, buffer, sizeof(buffer));
      if (n == 0) {
        return ReadStatus::kBadRequest;  // 声明了长度却提前断开
      }
      if (n < 0) {
        if (errno == EINTR) continue;
        return ReadStatus::kClosed;
      }
      body_size += static_cast<std::size_t>(n);
      raw.append(buffer, static_cast<std::size_t>(n));
      if (body_size > kMaxRequestBytes) {
        return ReadStatus::kTooLarge;
      }
    }
    out->body = raw.substr(body_begin, content_length);
    return ReadStatus::kOk;
  }

  // ---------------------------------------------------------------------------
  // 请求分发
  // ---------------------------------------------------------------------------
  void HandleClient(int client_fd) {
    HttpRequest req;
    const ReadStatus status = ReadRequest(client_fd, &req);
    if (status == ReadStatus::kClosed) {
      return;  // 对端没发完整请求就断了：静默关闭
    }
    if (status == ReadStatus::kBadRequest) {
      HttpResponse resp;
      resp.status = "400 Bad Request";
      resp.body = JsonErrorBody("bad_request");
      RecordRequestStatus(resp.status);
      WriteAll(client_fd, SerializeResponse(resp, false));
      DrainForClose(client_fd, 64U * 1024U);
      return;
    }
    if (status == ReadStatus::kTooLarge) {
      HttpResponse resp;
      resp.status = "413 Payload Too Large";
      resp.body = JsonErrorBody("payload_too_large");
      RecordRequestStatus(resp.status);
      WriteAll(client_fd, SerializeResponse(resp, false));
      DrainForClose(client_fd);
      return;
    }

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
    request->pickup_station = from;
    request->dropoff_station = to;
    request->priority = prio;
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
    auto request = std::make_shared<amr_dispatcher_interfaces::srv::CancelOrder::Request>();
    request->mission_id = mid;
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
    resp.body = "{\"status\":\"ok\",\"canceled\":" + std::string(res->success ? "true" : "false") + "}";
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
  std::deque<int> pending_;
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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;

  rclcpp::Subscription<amr_dispatcher_interfaces::msg::DispatcherState>::SharedPtr state_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::ChassisLinkHealth>::SharedPtr link_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::MissionEvent>::SharedPtr event_sub_;

  amr_dispatcher_interfaces::msg::DispatcherState latest_state_;
  amr_dispatcher_interfaces::msg::SafetyState latest_safety_;
  amr_dispatcher_interfaces::msg::ChassisLinkHealth latest_link_;
  std::vector<amr_dispatcher_interfaces::msg::MissionEvent> recent_events_;
  bool safety_seen_ = false;
};

}  // namespace amr_dispatcher_tools

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_tools::RestGatewayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
