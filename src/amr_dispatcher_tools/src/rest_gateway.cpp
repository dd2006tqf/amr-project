#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cctype>
#include <cstring>
#include <filesystem>
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
    } else if (method == "GET" && path == "/api/v1/vehicle_links") {
      response_body = FormatVehicleLinksJson();
    } else if (method == "POST" && path == "/api/v1/force_clear_link") {
      // 危险操作：请求体 JSON {vehicle_id, confirm_vehicle_id, operator_id, reason}
      // 服务端会复验全部准入条件（前端隐藏按钮不是安全边界）。
      size_t body_start = request.find("\r\n\r\n");
      const std::string body =
          (body_start != std::string::npos) ? request.substr(body_start + 4) : "";
      const std::string vehicle_id = JsonStringField(body, "vehicle_id");
      const std::string confirm_id = JsonStringField(body, "confirm_vehicle_id");
      const std::string operator_id = JsonStringField(body, "operator_id");
      const std::string reason = JsonStringField(body, "reason");
      if (vehicle_id.empty() || confirm_id.empty() || operator_id.empty() || reason.empty()) {
        status_code = "400 Bad Request";
        response_body =
            "{\"error\":\"missing_fields\",\"detail\":\"vehicle_id, confirm_vehicle_id, "
            "operator_id, reason are all required\"}";
      } else {
        auto req = std::make_shared<amr_dispatcher_interfaces::srv::ForceClearVda5050Link::Request>();
        req->vehicle_id = vehicle_id;
        req->confirm_vehicle_id = confirm_id;
        req->operator_id = operator_id;
        req->reason = reason;
        if (force_clear_cli_->wait_for_service(1s)) {
          auto fut = force_clear_cli_->async_send_request(req);
          if (fut.wait_for(2s) == std::future_status::ready) {
            auto res = fut.get();
            response_body = std::string("{\"status\":\"") +
                            (res->success ? "ok" : "rejected") + "\",\"message\":\"" +
                            res->message + "\",\"quarantined\":" +
                            (res->quarantined ? "true" : "false") + ",\"released_resources\":[";
            for (std::size_t i = 0; i < res->released_resources.size(); ++i) {
              if (i) response_body += ",";
              response_body += "\"" + res->released_resources[i] + "\"";
            }
            response_body += "]}";
            if (!res->success) {
              status_code = "409 Conflict";
            }
          } else {
            status_code = "504 Gateway Timeout";
            response_body = "{\"error\":\"timeout\"}";
          }
        } else {
          status_code = "503 Service Unavailable";
          response_body = "{\"error\":\"service_unavailable\"}";
        }
      }
    } else if (method == "POST" && path == "/api/v1/restore_link") {
      // 请求体 JSON {vehicle_id, confirm_vehicle_id, operator_id, reason, position_verified}
      size_t body_start = request.find("\r\n\r\n");
      const std::string body =
          (body_start != std::string::npos) ? request.substr(body_start + 4) : "";
      const std::string vehicle_id = JsonStringField(body, "vehicle_id");
      const std::string confirm_id = JsonStringField(body, "confirm_vehicle_id");
      const std::string operator_id = JsonStringField(body, "operator_id");
      const std::string reason = JsonStringField(body, "reason");
      const bool position_verified = JsonBoolField(body, "position_verified");
      if (vehicle_id.empty() || confirm_id.empty() || operator_id.empty() || reason.empty()) {
        status_code = "400 Bad Request";
        response_body =
            "{\"error\":\"missing_fields\",\"detail\":\"vehicle_id, confirm_vehicle_id, "
            "operator_id, reason are all required\"}";
      } else {
        auto req = std::make_shared<amr_dispatcher_interfaces::srv::RestoreVda5050Link::Request>();
        req->vehicle_id = vehicle_id;
        req->confirm_vehicle_id = confirm_id;
        req->operator_id = operator_id;
        req->reason = reason;
        req->position_verified = position_verified;
        if (restore_cli_->wait_for_service(1s)) {
          auto fut = restore_cli_->async_send_request(req);
          if (fut.wait_for(2s) == std::future_status::ready) {
            auto res = fut.get();
            response_body = std::string("{\"status\":\"") +
                            (res->success ? "ok" : "rejected") + "\",\"message\":\"" +
                            res->message + "\"}";
            if (!res->success) {
              status_code = "409 Conflict";
            }
          } else {
            status_code = "504 Gateway Timeout";
            response_body = "{\"error\":\"timeout\"}";
          }
        } else {
          status_code = "503 Service Unavailable";
          response_body = "{\"error\":\"service_unavailable\"}";
        }
      }
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
    } else if (method == "POST" && path == "/api/v1/map/upload") {
      // 一键热上传与地图骨架拓扑生成
      size_t body_start = request.find("\r\n\r\n");
      std::string body = (body_start != std::string::npos) ? request.substr(body_start + 4) : "";

      try {
        // 解析简单 JSON：width, height, resolution, origin_x, origin_y, pgm_base64 / grid, anchors
        int width = 0, height = 0;
        double res = 0.05, origin_x = 0.0, origin_y = 0.0;

        auto get_int = [&body](const std::string& key, int def) -> int {
          auto pos = body.find("\"" + key + "\"");
          if (pos == std::string::npos) return def;
          pos = body.find(':', pos);
          if (pos == std::string::npos) return def;
          return std::stoi(body.substr(pos + 1));
        };
        auto get_double = [&body](const std::string& key, double def) -> double {
          auto pos = body.find("\"" + key + "\"");
          if (pos == std::string::npos) return def;
          pos = body.find(':', pos);
          if (pos == std::string::npos) return def;
          return std::stod(body.substr(pos + 1));
        };

        width = get_int("width", 0);
        height = get_int("height", 0);
        res = get_double("resolution", 0.05);
        origin_x = get_double("origin_x", 0.0);
        origin_y = get_double("origin_y", 0.0);

        if (width <= 0 || height <= 0) {
          status_code = "400 Bad Request";
          response_body = "{\"error\":\"invalid_dimensions\"}";
        } else {
          amr_dispatcher_core::planning::MapGrid grid;
          grid.width = static_cast<uint32_t>(width);
          grid.height = static_cast<uint32_t>(height);
          grid.resolution = res;
          grid.origin_x = origin_x;
          grid.origin_y = origin_y;
          grid.origin_yaw = 0.0;
          grid.data.assign(width * height, 0); // 默认为空闲通行区

          // 提取 grid_data 数组 (如 [0, 100, -1, ...])
          auto grid_pos = body.find("\"grid_data\"");
          if (grid_pos != std::string::npos) {
            auto arr_start = body.find('[', grid_pos);
            auto arr_end = body.find(']', arr_start);
            if (arr_start != std::string::npos && arr_end != std::string::npos) {
              std::string arr_str = body.substr(arr_start + 1, arr_end - arr_start - 1);
              std::stringstream ss(arr_str);
              std::string item;
              size_t idx = 0;
              while (std::getline(ss, item, ',') && idx < grid.data.size()) {
                try {
                  grid.data[idx++] = static_cast<int8_t>(std::stoi(item));
                } catch (...) {}
              }
            }
          }

          // 提取 anchors: [{"id":"A","type":"pickup","x":1.0,"y":2.0}, ...]
          std::vector<amr_dispatcher_core::planning::AnchorStation> anchors;
          auto anchors_pos = body.find("\"anchors\"");
          if (anchors_pos != std::string::npos) {
            auto arr_start = body.find('[', anchors_pos);
            auto arr_end = body.find(']', arr_start);
            if (arr_start != std::string::npos && arr_end != std::string::npos) {
              std::string arr_str = body.substr(arr_start + 1, arr_end - arr_start - 1);
              size_t obj_start = 0;
              while ((obj_start = arr_str.find('{', obj_start)) != std::string::npos) {
                size_t obj_end = arr_str.find('}', obj_start);
                if (obj_end == std::string::npos) break;
                std::string obj_str = arr_str.substr(obj_start, obj_end - obj_start + 1);

                auto extract_str = [&obj_str](const std::string& k) -> std::string {
                  auto p = obj_str.find("\"" + k + "\"");
                  if (p == std::string::npos) return "";
                  p = obj_str.find(':', p);
                  if (p == std::string::npos) return "";
                  auto q1 = obj_str.find('\"', p);
                  if (q1 == std::string::npos) return "";
                  auto q2 = obj_str.find('\"', q1 + 1);
                  if (q2 == std::string::npos) return "";
                  return obj_str.substr(q1 + 1, q2 - q1 - 1);
                };
                auto extract_num = [&obj_str](const std::string& k) -> double {
                  auto p = obj_str.find("\"" + k + "\"");
                  if (p == std::string::npos) return 0.0;
                  p = obj_str.find(':', p);
                  if (p == std::string::npos) return 0.0;
                  return std::stod(obj_str.substr(p + 1));
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
          auto safe_grid = amr_dispatcher_core::planning::FilterSafetySpace(grid, sf_cfg);
          auto skel = amr_dispatcher_core::planning::ZhangSuenThinning(safe_grid);

          std::vector<amr_dispatcher_core::planning::PixelCoord> protected_points;
          for (const auto& a : anchors) {
            amr_dispatcher_core::planning::WorldCoord w{a.x, a.y};
            protected_points.push_back(amr_dispatcher_core::planning::WorldToPixel(w, grid));
          }
          amr_dispatcher_core::planning::PruneConfig pr_cfg;
          pr_cfg.min_branch_length_m = 0.5;
          pr_cfg.resolution = res;
          auto pruned = amr_dispatcher_core::planning::PruneSkeletonProtected(skel, protected_points, pr_cfg);

          amr_dispatcher_core::planning::TopologyBuilderConfig tb_cfg;
          tb_cfg.max_snap_dist_m = 3.0;
          tb_cfg.map_hash = "web_upload_" + std::to_string(width) + "x" + std::to_string(height);
          tb_cfg.graph_version = "v1_web";
          auto build_res = amr_dispatcher_core::planning::BuildTopologyCatalog(pruned, grid, anchors, tb_cfg);

          if (!build_res.success) {
            status_code = "422 Unprocessable Entity";
            response_body = "{\"status\":\"error\",\"message\":\"" + build_res.error_message + "\"}";
          } else {
            // 写入 config/generated/stations.yaml (tmp + rename 原子替换)
            std::filesystem::path final_path("config/generated/stations.yaml");
            std::filesystem::create_directories(final_path.parent_path());
            std::filesystem::path tmp_path("config/generated/stations.yaml.tmp");

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
            std::filesystem::rename(tmp_path, final_path);

            response_body = "{\"status\":\"ok\",\"stations_count\":" + std::to_string(build_res.catalog.stations.size()) +
                            ",\"road_nodes_count\":" + std::to_string(build_res.catalog.road_nodes.size()) +
                            ",\"edges_count\":" + std::to_string(build_res.catalog.edges.size()) +
                            ",\"map_hash\":\"" + build_res.catalog.map_hash + "\"}";
          }
        }
      } catch (const std::exception& e) {
        status_code = "500 Internal Server Error";
        response_body = "{\"error\":\"" + std::string(e.what()) + "\"}";
      }
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

  // 极简 JSON 字符串字段提取（请求体由本项目自己的控制台产生，格式可控）：
  // 形如 "key": "value" -> value；缺失或非字符串返回空串。
  std::string JsonStringField(const std::string& body, const std::string& key) {
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
      if (body[pos] == '\\' && pos + 1 < body.size()) ++pos;  // 跳过转义字符
      value.push_back(body[pos]);
      ++pos;
    }
    return value;
  }

  // 极简 JSON 布尔字段提取：true / false。
  bool JsonBoolField(const std::string& body, const std::string& key) {
    const std::string target = "\"" + key + "\"";
    auto pos = body.find(target);
    if (pos == std::string::npos) return false;
    pos = body.find(':', pos);
    if (pos == std::string::npos) return false;
    ++pos;
    while (pos < body.size() && std::isspace(static_cast<unsigned char>(body[pos]))) ++pos;
    return body.compare(pos, 4, "true") == 0;
  }

  std::string FormatVehicleLinksJson() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::ostringstream os;
    os << "{\"vehicle_links\":[";
    const auto& links = latest_state_.vehicle_links;
    for (std::size_t i = 0; i < links.size(); ++i) {
      const auto& v = links[i];
      if (i) os << ",";
      os << "{\"vehicle_id\":\"" << v.vehicle_id << "\""
         << ",\"backend\":\"" << v.backend << "\""
         << ",\"online\":" << (v.online ? "true" : "false")
         << ",\"busy\":" << (v.busy ? "true" : "false")
         << ",\"quarantined\":" << (v.quarantined ? "true" : "false")
         << ",\"position_valid\":" << (v.position_valid ? "true" : "false")
         << ",\"current_station_id\":\"" << v.current_station_id << "\""
         << ",\"battery_percent\":" << v.battery_percent
         << ",\"state_age_ms\":" << v.state_age_ms
         << ",\"last_error\":\"" << v.last_error << "\"}";
    }
    os << "]}";
    return os.str();
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
       // 未收到任何 SafetyState 时不渲染急停（默认值会误报 estop）
       << "\"estop_active\":" << ((safety_seen_ && !latest_safety_.motion_allowed) ? "true" : "false") << ","
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
