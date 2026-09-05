#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/srv/cancel_order.hpp"
#include "amr_dispatcher_interfaces/srv/submit_order.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_tools {

// 轻量 HTTP REST 网关，支持简单的幂等性校验
class RestGatewayNode : public rclcpp::Node {
 public:
  explicit RestGatewayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("rest_gateway_node", options) {
    declare_parameter("port", 8080);
    port_ = get_parameter("port").as_int();

    submit_cli_ = create_client<amr_dispatcher_interfaces::srv::SubmitOrder>("/dispatcher/submit_order");
    cancel_cli_ = create_client<amr_dispatcher_interfaces::srv::CancelOrder>("/dispatcher/cancel_order");

    state_sub_ = create_subscription<amr_dispatcher_interfaces::msg::DispatcherState>(
        "/dispatcher/state", 10,
        [this](const amr_dispatcher_interfaces::msg::DispatcherState::SharedPtr msg) {
          latest_state_ = *msg;
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
    char buffer[4096] = {0};
    ssize_t bytes_read = read(client_fd, buffer, sizeof(buffer) - 1);
    if (bytes_read <= 0) return;

    std::string request(buffer, bytes_read);
    std::istringstream req_stream(request);
    std::string method, path;
    req_stream >> method >> path;

    std::string response_body;
    std::string status_code = "200 OK";

    if (method == "GET" && path == "/api/v1/state") {
      response_body = FormatStateJson();
    } else if (method == "POST" && path.find("/api/v1/orders") != std::string::npos) {
      // 简单参数解析：/api/v1/orders?id=m1&from=A&to=B&prio=5&idempotency_key=k123
      auto q_pos = path.find('?');
      if (q_pos != std::string::npos) {
        std::string query = path.substr(q_pos + 1);
        std::string mid = GetQueryParam(query, "id");
        std::string from = GetQueryParam(query, "from");
        std::string to = GetQueryParam(query, "to");
        int prio = std::stoi(GetQueryParam(query, "prio", "0"));
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
    } else {
      status_code = "404 Not Found";
      response_body = "{\"error\":\"not_found\"}";
    }

    std::ostringstream resp;
    resp << "HTTP/1.1 " << status_code << "\r\n";
    resp << "Content-Type: application/json\r\n";
    resp << "Content-Length: " << response_body.size() << "\r\n";
    resp << "Connection: close\r\n\r\n";
    resp << response_body;

    std::string resp_str = resp.str();
    write(client_fd, resp_str.c_str(), resp_str.size());
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

  std::string FormatStateJson() {
    std::ostringstream os;
    os << "{"
       << "\"comparator\":\"" << latest_state_.current_comparator << "\","
       << "\"queue_size\":" << latest_state_.queue_size << ","
       << "\"queue_capacity\":" << latest_state_.queue_capacity << ","
       << "\"active_count\":" << latest_state_.active_missions_count << ","
       << "\"deadlock\":" << (latest_state_.deadlock_detected ? "true" : "false")
       << "}";
    return os.str();
  }

  int port_ = 8080;
  int server_fd_ = -1;
  std::atomic<bool> running_{true};
  std::thread server_thread_;
  std::unordered_set<std::string> seen_idempotency_keys_;

  rclcpp::Client<amr_dispatcher_interfaces::srv::SubmitOrder>::SharedPtr submit_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::CancelOrder>::SharedPtr cancel_cli_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::DispatcherState>::SharedPtr state_sub_;
  amr_dispatcher_interfaces::msg::DispatcherState latest_state_;
};

}  // namespace amr_dispatcher_tools

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_dispatcher_tools::RestGatewayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
