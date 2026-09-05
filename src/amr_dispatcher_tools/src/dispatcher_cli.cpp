#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "amr_dispatcher_interfaces/msg/dispatcher_state.hpp"
#include "amr_dispatcher_interfaces/srv/cancel_order.hpp"
#include "amr_dispatcher_interfaces/srv/pause_mission.hpp"
#include "amr_dispatcher_interfaces/srv/resume_mission.hpp"
#include "amr_dispatcher_interfaces/srv/submit_order.hpp"

using namespace std::chrono_literals;

namespace amr_dispatcher_tools {

class DispatcherCliClient {
 public:
  explicit DispatcherCliClient(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    submit_cli_ = node_->create_client<amr_dispatcher_interfaces::srv::SubmitOrder>("/dispatcher/submit_order");
    cancel_cli_ = node_->create_client<amr_dispatcher_interfaces::srv::CancelOrder>("/dispatcher/cancel_order");
    pause_cli_ = node_->create_client<amr_dispatcher_interfaces::srv::PauseMission>("/dispatcher/pause_mission");
    resume_cli_ = node_->create_client<amr_dispatcher_interfaces::srv::ResumeMission>("/dispatcher/resume_mission");

    state_sub_ = node_->create_subscription<amr_dispatcher_interfaces::msg::DispatcherState>(
        "/dispatcher/state", 10,
        [this](const amr_dispatcher_interfaces::msg::DispatcherState::SharedPtr msg) {
          latest_state_ = *msg;
        });
  }

  void Submit(const std::string& mid, const std::string& from, const std::string& to, int prio) {
    if (!submit_cli_->wait_for_service(1s)) {
      std::cout << "[ERROR] /dispatcher/submit_order service not available" << std::endl;
      return;
    }
    auto req = std::make_shared<amr_dispatcher_interfaces::srv::SubmitOrder::Request>();
    req->mission_id = mid;
    req->pickup_station = from;
    req->dropoff_station = to;
    req->priority = prio;

    auto future = submit_cli_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, future, 2s) == rclcpp::FutureReturnCode::SUCCESS) {
      auto res = future.get();
      std::cout << "[SUCCESS] Submitted mission " << res->mission_id << ": " << res->message << std::endl;
    } else {
      std::cout << "[ERROR] Service call timed out" << std::endl;
    }
  }

  void Cancel(const std::string& mid) {
    if (!cancel_cli_->wait_for_service(1s)) {
      std::cout << "[ERROR] /dispatcher/cancel_order service not available" << std::endl;
      return;
    }
    auto req = std::make_shared<amr_dispatcher_interfaces::srv::CancelOrder::Request>();
    req->mission_id = mid;

    auto future = cancel_cli_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, future, 2s) == rclcpp::FutureReturnCode::SUCCESS) {
      auto res = future.get();
      std::cout << "[RESULT] " << res->message << " (canceled count=" << res->canceled_count << ")" << std::endl;
    } else {
      std::cout << "[ERROR] Service call timed out" << std::endl;
    }
  }

  void Pause(const std::string& mid) {
    if (!pause_cli_->wait_for_service(1s)) {
      std::cout << "[ERROR] /dispatcher/pause_mission service not available" << std::endl;
      return;
    }
    auto req = std::make_shared<amr_dispatcher_interfaces::srv::PauseMission::Request>();
    req->mission_id = mid;
    auto future = pause_cli_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, future, 2s) == rclcpp::FutureReturnCode::SUCCESS) {
      std::cout << "[RESULT] " << future.get()->message << std::endl;
    }
  }

  void Resume(const std::string& mid) {
    if (!resume_cli_->wait_for_service(1s)) {
      std::cout << "[ERROR] /dispatcher/resume_mission service not available" << std::endl;
      return;
    }
    auto req = std::make_shared<amr_dispatcher_interfaces::srv::ResumeMission::Request>();
    req->mission_id = mid;
    auto future = resume_cli_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, future, 2s) == rclcpp::FutureReturnCode::SUCCESS) {
      std::cout << "[RESULT] " << future.get()->message << std::endl;
    }
  }

  void Status() {
    rclcpp::spin_some(node_);
    std::cout << "=== Dispatcher State ===" << std::endl;
    std::cout << " Comparator:  " << latest_state_.current_comparator << std::endl;
    std::cout << " Queue:       " << latest_state_.queue_size << "/" << latest_state_.queue_capacity << std::endl;
    std::cout << " Active:      " << latest_state_.active_missions_count << std::endl;
    std::cout << " Deadlock:    " << (latest_state_.deadlock_detected ? "YES" : "NO") << std::endl;
    std::cout << " Active IDs:  ";
    for (const auto& id : latest_state_.active_mission_ids) {
      std::cout << id << " ";
    }
    std::cout << std::endl;
  }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::SubmitOrder>::SharedPtr submit_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::CancelOrder>::SharedPtr cancel_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::PauseMission>::SharedPtr pause_cli_;
  rclcpp::Client<amr_dispatcher_interfaces::srv::ResumeMission>::SharedPtr resume_cli_;
  rclcpp::Subscription<amr_dispatcher_interfaces::msg::DispatcherState>::SharedPtr state_sub_;
  amr_dispatcher_interfaces::msg::DispatcherState latest_state_;
};

}  // namespace amr_dispatcher_tools

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dispatcher_cli_node");
  amr_dispatcher_tools::DispatcherCliClient client(node);

  std::cout << "============================================" << std::endl;
  std::cout << "   AMR Dispatcher Interactive CLI Client    " << std::endl;
  std::cout << "Commands:" << std::endl;
  std::cout << "  submit <mid> <from> <to> <prio>" << std::endl;
  std::cout << "  cancel <mid>" << std::endl;
  std::cout << "  pause <mid>" << std::endl;
  std::cout << "  resume <mid>" << std::endl;
  std::cout << "  status" << std::endl;
  std::cout << "  exit" << std::endl;
  std::cout << "============================================" << std::endl;

  std::string cmd;
  while (std::cout << "amr_dispatcher> " && std::cin >> cmd) {
    if (cmd == "exit" || cmd == "quit") {
      break;
    } else if (cmd == "status") {
      client.Status();
    } else if (cmd == "submit") {
      std::string mid, from, to;
      int prio = 0;
      std::cin >> mid >> from >> to >> prio;
      client.Submit(mid, from, to, prio);
    } else if (cmd == "cancel") {
      std::string mid;
      std::cin >> mid;
      client.Cancel(mid);
    } else if (cmd == "pause") {
      std::string mid;
      std::cin >> mid;
      client.Pause(mid);
    } else if (cmd == "resume") {
      std::string mid;
      std::cin >> mid;
      client.Resume(mid);
    } else {
      std::cout << "Unknown command. Type 'status', 'submit', 'cancel', 'pause', 'resume', or 'exit'." << std::endl;
    }
  }

  rclcpp::shutdown();
  return 0;
}
