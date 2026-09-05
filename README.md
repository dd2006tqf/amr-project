# AMR Dispatcher & Fleet Controller

High-reliability Autonomous Mobile Robot (AMR) task scheduling, multi-tier hardware communication, safety gating, and path tracking system.

Developed with C++17/C++20, ROS 2 Jazzy, and ros2_control.

## 🌟 Highlights & Differentiators

1. **Fully Decoupled Core Library (`amr_dispatcher_core`)**:
   - Zero ROS dependencies.
   - Built and tested with pure standard CMake & GoogleTest.
   - 120/120 unit tests with 100% pass rate.
2. **Hardware Link Quality & Auto-Degradation**:
   - Real-time sliding-window loss rate, CRC errors, and RTT monitoring.
   - Automatic hardware failover across preferred backend list: `Serial` -> `UDP` -> `Mock`.
   - Exponential backoff with random jitter for connection recovery.
3. **Flexible Pluggable Dispatching**:
   - Strategy pattern scheduler: `PriorityFIFO`, `EarliestDeadlineFirst (EDF)`, `ShortestJobFirst (SJF)`, `WeightedScore`.
   - Atomic resource reservation (`route_edge` and `route_node`).
   - Deadlock detection & recovery policy for multi-robot conflicts.
4. **Safety Multi-Source Arbitration & Watchdog**:
   - Software watchdog tracking node heartbeats.
   - Multi-source priority gate arbitrating E-Stop, bumper, and speed limits.
5. **Operational Tooling**:
   - Interactive CLI client (`dispatcher_cli`).
   - Lightweight REST Gateway (`rest_gateway`) supporting idempotency keys.

## 🏗️ Architecture

```
amr_dispatcher/
├── src/
│   ├── amr_dispatcher_core/          # Pure C++17/20 core logic & algorithms
│   │   ├── chassis/                  # Serial/UDP/Mock backends, link quality & recovery
│   │   ├── path_tracking/            # Pure Pursuit & Stanley controllers
│   │   ├── dispatcher/               # Pluggable mission queue, traffic locks, deadlock detector
│   │   ├── safety/                   # CmdVel safety gate, watchdog
│   │   ├── config/                   # Strongly typed YAML loader & hot-reload
│   │   └── logging/                  # Async lock-free logger
│   ├── amr_dispatcher_interfaces/    # ROS 2 msg (x5), srv (x6), action (x1)
│   ├── amr_dispatcher_ros/           # ROS 2 Jazzy Lifecycle & Standard nodes
│   └── amr_dispatcher_tools/         # CLI client & REST gateway
├── config/                           # YAML configs for chassis, safety, dispatcher, scenarios
├── launch/                           # Launch scripts (full_system, core_only, bringup)
├── docker/                           # Dockerfile & compose for Jazzy
└── scripts/                          # Build and validation scripts
```

## 🚀 Quick Start

### Build Core & Run Tests (Pure CMake)
```bash
./scripts/build.sh
```

### Run Full System in Docker (ROS 2 Jazzy)
```bash
cd docker
docker compose up --build
```
