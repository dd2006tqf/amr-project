#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/bool.hpp>

#include "amr_dispatcher_interfaces/msg/chassis_link_health.hpp"
#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/msg/mission_event.hpp"
#include "amr_dispatcher_interfaces/msg/safety_state.hpp"
#include "amr_dispatcher_interfaces/srv/cancel_order.hpp"
#include "amr_dispatcher_interfaces/srv/get_system_readiness.hpp"
#include "amr_dispatcher_interfaces/srv/submit_order.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_tools {

class RestGatewayNode : public rclcpp::Node {
 public:
  explicit RestGatewayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("rest_gateway_node", options) {
    declare_parameter("port", 8080);
    declare_parameter("html_path", "tools/operator_console.html");
    port_ = get_parameter("port").as_int();
    html_path_ = get_parameter("html_path").as_string();

    submit_cli_ = create_client<amr_dispatcher_interfaces::srv::SubmitOrder>("/dispatcher/submit_order");
    cancel_cli_ = create_client<amr_dispatcher_interfaces::srv::CancelOrder>("/dispatcher/cancel_order");
    readiness_cli_ = create_client<amr_dispatcher_interfaces::srv::GetSystemReadiness>("/system/ready");

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
    RCLCPP_INFO(get_logger(), "REST Gateway started on port %d", port_);
  }

  ~RestGatewayNode() override {
    running_ = false;
    if (server_fd_ >= 0) {
      close(server_fd_);
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

 private:
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
    address.sin_port = htons(port_);

    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
      RCLCPP_ERROR(get_logger(), "Socket bind failed on port %d", port_);
      return;
    }

    if (listen(server_fd_, 10) < 0) {
      RCLCPP_ERROR(get_logger(), "Socket listen failed");
      return;
    }

    while (running_ && rclcpp::ok()) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
      if (client_fd < 0) {
        if (!running_) break;
        continue;
      }
      HandleClient(client_fd);
      close(client_fd);
    }
  }

  void HandleClient(int client_fd) {
    std::string request;
    char buffer[4096] = {0};
    ssize_t bytes_read = read(client_fd, buffer, sizeof(buffer) - 1);
    if (bytes_read <= 0) return;
    request.append(buffer, bytes_read);

    // 检查 Content-Length，支持完整的长请求流读取
    auto cl_pos = request.find("Content-Length: ");
    if (cl_pos != std::string::npos) {
      size_t cl_end = request.find("\r\n", cl_pos);
      if (cl_end != std::string::npos) {
        int content_len = std::stoi(request.substr(cl_pos + 16, cl_end - (cl_pos + 16)));
        size_t header_end = request.find("\r\n\r\n");
        if (header_end != std::string::npos) {
          size_t body_len = request.size() - (header_end + 4);
          while (body_len < static_cast<size_t>(content_len)) {
            bytes_read = read(client_fd, buffer, sizeof(buffer) - 1);
            if (bytes_read <= 0) break;
            request.append(buffer, bytes_read);
            body_len += bytes_read;
          }
        }
      }
    }

    std::istringstream req_stream(request);
    std::string method, path;
    req_stream >> method >> path;

    std::string response_body;
    std::string content_type = "application/json";
    std::string status_code = "200 OK";

    if (method == "OPTIONS") {
      status_code = "204 No Content";
      response_body = "";
    } else if (method == "GET" && (path == "/" || path == "/index.html")) {
      content_type = "text/html; charset=utf-8";
      response_body = ReadStaticHtml();
      if (response_body.empty()) {
        status_code = "404 Not Found";
        response_body = "<h1>404 Not Found: operator_console.html</h1>";
      }
    } else if (method == "GET" && (path == "/api/operator/snapshot" || path == "/api/v1/state")) {
      response_body = FormatSnapshotJson();
    } else if (method == "GET" && path == "/api/system/ready") {
      if (readiness_cli_->wait_for_service(500ms)) {
        auto req = std::make_shared<amr_dispatcher_interfaces::srv::GetSystemReadiness::Request>();
        auto fut = readiness_cli_->async_send_request(req);
        if (fut.wait_for(1s) == std::future_status::ready) {
          auto res = fut.get();
          response_body = "{\"ready\":" + std::string(res->ready ? "true" : "false") +
                          ",\"healthy\":" + std::string(res->healthy ? "true" : "false") +
                          ",\"state\":\"" + res->state + "\",\"message\":\"" + res->message + "\"}";
          if (!res->ready) status_code = "503 Service Unavailable";
        } else {
          status_code = "504 Gateway Timeout";
          response_body = "{\"error\":\"readiness_probe_timeout\"}";
        }
      } else {
        status_code = "503 Service Unavailable";
        response_body = "{\"ready\":false,\"state\":\"INITIALIZING\",\"message\":\"Lifecycle Coordinator not ready\"}";
      }
    } else if (method == "POST" && path.find("/api/v1/orders") != std::string::npos) {
      // /api/v1/orders?id=m1&from=A&to=B&prio=5&idempotency_key=k123
      auto q_pos = path.find('?');
      if (q_pos != std::string::npos) {
        std::string query = path.substr(q_pos + 1);
        std::string mid = GetQueryParam(query, "id");
        std::string from = GetQueryParam(query, "from");
        std::string to = GetQueryParam(query, "to");
        int prio = std::stoi(GetQueryParam(query, "prio", "5"));
        std::string ikey = GetQueryParam(query, "idempotency_key");

        if (!ikey.empty() && seen_idempotency_keys_.find(ikey) != seen_idempotency_keys_.end()) {
          response_body = "{\"status\":\"ignored\",\"reason\":\"idempotent_duplicate\"}";
        } else {
          if (!ikey.empty()) seen_idempotency_keys_.insert(ikey);
          auto req = std::make_shared<amr_dispatcher_interfaces::srv::SubmitOrder::Request>();
          req->mission_id = mid;
          req->pickup_station = from;
          req->dropoff_station = to;
          req->priority = prio;

          if (submit_cli_->wait_for_service(1s)) {
            auto fut = submit_cli_->async_send_request(req);
            if (fut.wait_for(2s) == std::future_status::ready) {
              auto res = fut.get();
              response_body = "{\"status\":\"ok\",\"accepted\":" + std::string(res->accepted ? "true" : "false") +
                              ",\"mission_id\":\"" + res->mission_id + "\"}";
            } else {
              status_code = "504 Gateway Timeout";
              response_body = "{\"error\":\"timeout\"}";
            }
          } else {
            status_code = "503 Service Unavailable";
            response_body = "{\"error\":\"service_unavailable\"}";
          }
        }
      } else {
        status_code = "400 Bad Request";
        response_body = "{\"error\":\"missing_query_parameters\"}";
      }
    } else if (method == "POST" && (path.find("/api/v1/cancel") != std::string::npos || path.find("/api/workflows/cancel") != std::string::npos)) {
      auto q_pos = path.find('?');
      std::string mid;
      if (q_pos != std::string::npos) {
        mid = GetQueryParam(path.substr(q_pos + 1), "id");
      }
      auto req = std::make_shared<amr_dispatcher_interfaces::srv::CancelOrder::Request>();
      req->mission_id = mid;
      if (cancel_cli_->wait_for_service(1s)) {
        auto fut = cancel_cli_->async_send_request(req);
        if (fut.wait_for(2s) == std::future_status::ready) {
          auto res = fut.get();
          response_body = "{\"status\":\"ok\",\"canceled\":" + std::string(res->success ? "true" : "false") + "}";
        } else {
          status_code = "504 Gateway Timeout";
          response_body = "{\"error\":\"timeout\"}";
        }
      } else {
        status_code = "503 Service Unavailable";
        response_body = "{\"error\":\"service_unavailable\"}";
      }
    } else if (method == "POST" && path == "/api/operator/estop") {
      std_msgs::msg::Bool estop_msg;
      estop_msg.data = true;
      estop_pub_->publish(estop_msg);
      response_body = "{\"status\":\"ok\",\"estop_engaged\":true}";
    } else {
      status_code = "404 Not Found";
      response_body = "{\"error\":\"not_found\"}";
    }

    std::ostringstream resp;
    resp << "HTTP/1.1 " << status_code << "\r\n";
    resp << "Content-Type: " << content_type << "\r\n";
    resp << "Content-Length: " << response_body.size() << "\r\n";
    resp << "Access-Control-Allow-Origin: *\r\n";
    resp << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n";
    resp << "Connection: close\r\n\r\n";
    resp << response_body;

    std::string resp_str = resp.str();
    write(client_fd, resp_str.c_str(), resp_str.size());
  }

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
      std::string share_dir = ament_index_cpp::get_package_share_directory("amr_dispatcher_tools");
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

  std::string GetQueryParam(const std::string& query, const std::string& key, const std::string& fallback = "") {
    std::string target = key + "=";
    auto pos = query.find(target);
    if (pos == std::string::npos) return fallback;
    auto end = query.find('&', pos);
    if (end == std::string::npos) {
      return query.substr(pos + target.size());
    }
    return query.substr(pos + target.size(), end - (pos + target.size()));
  }

  std::string FormatSnapshotJson() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::ostringstream os;
    os << "{"
       << "\"comparator\":\"" << latest_state_.current_comparator << "\","
       << "\"strategy\":\"" << latest_state_.current_comparator << "\","
       << "\"queue_size\":" << latest_state_.queue_size << ","
       << "\"queue_capacity\":" << latest_state_.queue_capacity << ","
       << "\"active_count\":" << latest_state_.active_missions_count << ","
       << "\"active_mission_id\":\"" << latest_state_.active_mission_id << "\","
       << "\"deadlock\":" << (latest_state_.deadlock_detected ? "true" : "false") << ","
       << "\"deadlock_detected\":" << (latest_state_.deadlock_detected ? "true" : "false") << ","
       << "\"estop_active\":" << (!latest_safety_.motion_allowed ? "true" : "false") << ","
       << "\"safety_severity\":\"" << (latest_safety_.severity.empty() ? "INFO" : latest_safety_.severity) << "\","
       << "\"chassis_backend\":\"" << (latest_link_.backend_name.empty() ? "Serial/Mock" : latest_link_.backend_name) << "\","
       << "\"chassis_healthy\":" << (latest_link_.is_healthy ? "true" : "false") << ","
       << "\"chassis_loss_rate\":" << latest_link_.loss_rate << ","
       << "\"recent_events\":[";

    for (size_t i = 0; i < recent_events_.size(); ++i) {
      const auto& ev = recent_events_[i];
      os << "{\"mission_id\":\"" << ev.mission_id << "\",\"event\":\"" << ev.event
         << "\",\"reason\":\"" << ev.reason << "\"}";
      if (i + 1 < recent_events_.size()) os << ",";
    }
    os << "]}";
    return os.str();
  }

  int port_ = 8080;
  std::string html_path_{"tools/operator_console.html"};
  int server_fd_ = -1;
  std::atomic<bool> running_{true};
  std::thread server_thread_;
  std::mutex data_mutex_;
  std::unordered_set<std::string> seen_idempotency_keys_;

  rclcpp::Client<amr_dispatcher_interfaces::srv::SubmitOrder>::SharedPtr submit_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::CancelOrder>::SharedPtr cancel_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::GetSystemReadiness>::SharedPtr readiness_cli_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;

  rclcpp::Subscription<amr_dispatcher_interfaces::msg::DispatcherState>::SharedPtr state_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::ChassisLinkHealth>::SharedPtr link_sub_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::MissionEvent>::SharedPtr event_sub_;

  amr_dispatcher_interfaces::msg::DispatcherState latest_state_;
  amr_dispatcher_interfaces::msg::SafetyState latest_safety_;
  amr_dispatcher_interfaces::msg::ChassisLinkHealth latest_link_;
  std::vector<amr_dispatcher_interfaces::msg::MissionEvent> recent_events_;
};

}  // namespace amr_dispatcher_tools

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_tools::RestGatewayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
