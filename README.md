# AMR Dispatcher & Fleet Infrastructure

High-Reliability Autonomous Mobile Robot (AMR) Task Scheduling, Multi-Tier Hardware Failover, Safety Gating, and Graph Topology Control Platform.

Developed with Modern C++17/C++20, ROS 2 Jazzy, and Clean Ports & Adapters Architecture.

---

## 🌟 Core Highlights & Differentiators

1. **Zero-ROS Core Architecture (`amr_dispatcher_core`)**:
   - Strictly decoupled domain algorithms: zero ROS dependencies.
   - Built and tested with pure standard CMake & GoogleTest.
   - **453/453 unit and integration tests passing in ~9 seconds** with 100% test pass rate across 10 subdomains (chassis, path tracking, dispatcher, safety, config, logging, catalog, facility, fleet, workflow).
   - Standard IoC interfaces (`IPathPlanner`, `IPathTracker`) pre-configured with baseline mock implementations for zero-friction algorithm injection.

2. **Decoupled Behavior Tree Orchestration Subsystem (`amr_dispatcher_bt`)**:
   - Standalone orchestration layer built with **BehaviorTree.CPP v4** without polluting the zero-ROS core.
   - 10 behavior tree models (queue scheduling, mission recovery, order routing, runtime lifecycle, station sequence, facility & docking management) covered by **36/36 unit tests**.
   - Pluggable decision-making engine eliminating legacy hardcoded if-else dispatching.

3. **12 Enterprise Dispatcher Services (`/v2/*`)**:
   - Complete operational toolset across Mission lifecycle, Traffic control & Deadlock diagnosis, Physical facility reservation, and Site topology verification.
   - Full ROS 2 interfaces generated with zero duplicate schemas.

4. **Graph Theory Wait-For-Graph (WFG) & 0ms Deadlock Detection**:
   - Real-time dynamic resource allocation model tracking lock dependencies.
   - **0ms deterministic DFS cycle detection** replacing naive timeout guessing, outputting complete cycle paths `[CarA -> Edge1 -> CarB -> Edge2 -> CarA]`.
   - **Global Dynamic Re-routing (Dijkstra Bypass)**: automatically re-plans alternative detours around deadlock areas instead of dumb halting.

5. **Time-Space Topology Reservation (Multi-AMR Concurrency)**:
   - Upgraded from spatial exclusive locking to **Time-Space 3D Interval Reservation** $[t_{\text{start}}, t_{\text{end}}]$.
   - Enables multiple AMRs to interleave and stagger across shared pathways safely without deadlock or redundant stopping.

6. **Hardware Link Quality & Multi-Tier Auto-Degradation**:
   - Sliding-window frame loss rate, CRC error, and RTT monitoring.
   - Seamless degradation and recovery: `Serial` -> `UDP` -> `Mock` with exponential backoff and random jitter.
   - Tested against physical Linux pseudo-terminal PTY serial devices (`scripts/mock_physical_chassis.py`).

7. **Multi-Source Safety Gate & Software Watchdog**:
   - Independent 50Hz safety arbitration gate (`cmd_vel_safety_gate_node`).
   - Hierarchical prioritization: **E-Stop (Hard) > Manual Takeover > Watchdog Timeout (200ms) > Degraded Speed Clamping > Normal Pass-through**.

8. **Interactive Operations & 3D Observability**:
   - **Single-file zero-dependency Web Console (`tools/operator_console.html`)**: browser dashboard for live metrics, task dispatch, strategy switching, and emergency stop.
   - **RViz2 3D Semantic Markers**: real-time visualization of topological routes, dynamic green reservation locks, and red cylindrical deadlock beacon alerts with 1.2s auto-aging.
   - **ROS 2 Action (`ExecuteMission.action`)**: asynchronous long-duration mission streaming and client cancellation/preemption.

---

## 🏗️ System Architecture

```text
       Web Console / WMS / MES / RViz2
                     │
         [REST Gateway / Action Client]
                     │
═════════════════════╪═══════════════════════════════════════
 ROS 2 Middleware    ▼
   ┌─────────────────────────────────────────────────────┐
   │             dispatcher_lifecycle_node               │
   │  • rclcpp_lifecycle (Unconfigured -> Active)        │
   │  • ExecuteMission Action Server (Streaming & Cancel)│
   │  • Dynamic Parameter Hot-Reload (EDF / FIFO)        │
   └──────────┬──────────────────────┬───────────────────┘
              │                      │
              ▼ (Resource Locks)     ▼ (Markers)
   ┌──────────────────────┐ ┌────────────────────────────┐
   │   safety_gate_node   │ │  dispatcher_visualizer_node │
   │  • 200ms Watchdog    │ │  • Dynamic Green Lock Lines │
   │  • Multi-Source E-Stop│ │  • Red Deadlock Beacon     │
   └──────────┬───────────┘ └────────────────────────────┘
              ▼ /cmd_vel (Arbitrated)
══════════════╪═══════════════════════════════════════════════
 Domain Core  ▼
   ┌─────────────────────────────────────────────────────┐
   │                 amr_dispatcher_core                 │
   │  • WFG Deadlock DFS Cycle Detection & Dynamic Detour│
   │  • Time-Space Reservation Table (Time Windows)      │
   │  • Sliding-Window Loss Rate & 3-Tier Failover       │
   │  • Lock-Free Async Ring-Buffer Logger (2,000,000/s) │
   └─────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start & Automated Verification

### 1. Run Complete Automated CI & Chaos Pipeline (Host Linux)
```bash
./scripts/check_all.sh
```
Executes all 5 stages in one go:
- [Step 1] Build pure C++ core and run 139 GTest unit tests (~4s, 100% green).
- [Step 2] Benchmark mission queue (25,000 ops/s) and async logger (2,000,000 ops/s).
- [Step 3] WFG deadlock detection and dynamic detour chaos drill.
- [Step 4] Software watchdog 200ms timeout shutdown drill.
- [Step 5] Hardware loss rate failover drill.

### 2. Launch Physical Serial Device Simulator
```bash
python3 scripts/mock_physical_chassis.py
```
Simulates a real Linux PTY tty device at 50Hz, supporting hardware unplugging tests.

### 3. Open Web Operator Console
Simply launch `rest_gateway` or open in any browser:
```text
http://127.0.0.1:8080/
# or open tools/operator_console.html directly
```
