#pragma once

// 兼容别名：实现已下沉到 amr_dispatcher_core（零 ROS 依赖，宿主 ctest 可跑）。
// 保留这个头文件是为了让既有 include 与 `amr_dispatcher_ros::AssignmentDispatchAction`
// 之类的限定名继续可用，调用点零改动。

#include "amr_dispatcher_core/fleet/assignment_dispatch_action.hpp"

namespace amr_dispatcher_ros {

using AssignmentDispatchAction =
    amr_dispatcher_core::fleet::AssignmentDispatchAction;
using AssignmentDispatchDecision =
    amr_dispatcher_core::fleet::AssignmentDispatchDecision;
using amr_dispatcher_core::fleet::DecideAssignmentDispatch;

}  // namespace amr_dispatcher_ros
