# AMR Dispatcher & Fleet Infrastructure

High-Reliability Autonomous Mobile Robot (AMR) Task Scheduling, Multi-Tier Hardware Failover, Safety Gating, and Graph Topology Control Platform.

Developed with Modern C++17/C++20, ROS 2 Jazzy, and Clean Ports & Adapters Architecture.

---

## 🌟 Core Highlights & Differentiators

1. **Zero-ROS Core Architecture (`amr_dispatcher_core`)**:
   - Strictly decoupled domain algorithms: zero ROS dependencies.
   - Built and tested with pure standard CMake & GoogleTest.
   - **659/659 unit and integration tests passing in ~5 seconds** with 100% test pass rate across 10 subdomains (chassis, path tracking, dispatcher, safety, config, logging, catalog, facility, fleet, workflow, planning).
   - Standard IoC interfaces (`IPathPlanner`, `IPathTracker`) with `SkeletonPlanner` for dense equal-arc-length polyline expansion and baseline mock implementations.

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
   - Independent 50Hz safety arbitration gate (`safety_gate_node`).
   - Hierarchical prioritization: **E-Stop (Hard) > Manual Takeover > Watchdog Timeout (200ms) > Degraded Speed Clamping > Normal Pass-through**.

8. **Interactive Operations & 3D Observability**:
   - **Single-file zero-dependency Web Console (`tools/operator_console.html`)**: browser dashboard for live metrics, task dispatch, strategy switching, and emergency stop.
   - **RViz2 3D Semantic Markers**: real-time visualization of topological routes, dynamic green reservation locks, and red cylindrical deadlock beacon alerts with 1.2s auto-aging.
   - **ROS 2 Action (`ExecuteMission.action`)**: asynchronous long-duration mission streaming and client cancellation/preemption.

9. **VDA 5050 Native Fleet Adapter (MQTT)**:
   - **`vda5050_gateway_node`**: bidirectional bridge between ROS 2 topics and standard VDA 5050 v2 JSON over MQTT (`uagv/v2/{manufacturer}/{serialNumber}/{order|instantActions|state|connection}`), with QoS1 heartbeats and LWT `CONNECTIONBROKEN` detection.
   - **Direct order dispatch**: dispatcher publishes serialized VDA 5050 orders (`orderId = mission_id`, full polyline trajectory control points) to `/fleet/{vid}/send_order`; vehicle identity is validated against every state report.
   - **Spec 5.3 trailing lock release**: arriving at drop-off releases only the already-traversed trailing locks; **Spec 5.2 disconnect guard**: on LWT/heartbeat loss all locks are retained and fleet dispatch is suspended until the vehicle reconnects and its position is re-verified — verified end-to-end by `scripts/check_vda5050_e2e.sh` (8 stages, incl. kill -9 chaos drill).
   - **Operator force-clear (`/dispatcher/force_clear_vda5050_link`)**: closes the Spec 5.2 "manual release" branch — for a vehicle that is permanently dead and will never reconnect, the operator can atomically detach its offline link, release the locks bound to that vehicle, abort its in-flight mission, and unblock fleet dispatch. Rejected for online vehicles (never kills a live link).
   - **Force-clear = quarantine, not removal**: communication loss never implies "safe to release". The force-clear requires an operator id, a mandatory reason, and a `confirm_vehicle_id` that must match the target exactly; the vehicle is then moved into a **quarantined** state — still visible and auditable, but excluded from vehicle selection until an explicit `restore_link` (which requires the operator to have physically verified the vehicle's position).

10. **True Multi-Vehicle Fleet Concurrency (Two Execution Backends)**:
   - **Backend separation**: `local_sim` vehicles run the local motion chain (`/plan/{vid}` -> `path_tracker` -> `safety_gate` -> `chassis`); `vda5050` vehicles own their controllers and receive **only** the MQTT order (no local `/plan`, no local execution slot) — their completion is judged by `lastNodeId` reaching the order's final node. The two chains never impersonate each other.
   - **Per-vehicle topic & TF isolation**: `/plan/{vid}`, `/odom/{vid}`, `/tracking/error/{vid}`, `/cmd_vel_raw/{vid}`, `/cmd_vel_safe/{vid}` plus `{vid}/odom` / `{vid}/base_link` frames. E-stop and bumper remain **global** safety signals — any vehicle's emergency stop halts the whole fleet. Compatibility red line: with no managed-vehicle list, every topic name stays byte-identical to before.
   - **Vehicle selection with three-state outcome**: `kAssigned` / `kDeferred` (temporarily unavailable -> the mission **stays queued**) / `kIndeterminate` (state unknown -> auto-assignment paused) / `kRejected` (permanently unsatisfiable -> mission failed with a reason). Explicitly bound vehicles (`assigned_vehicle_id`) get **no exemption** from the hard constraints: online, enabled, not busy, not quarantined, capability-matched, battery above threshold, valid position, fresh state.
   - **Head-of-line avoidance**: a busy head mission no longer blocks the queue — the scheduler scans the first N queued missions and dispatches the first one that can actually be assigned.
   - **Time-space traffic coordination (`traffic_mode: time_space`)**: per-resource enter/exit windows (edge = channel, node = intersection) with accel/turn/safety-margin overhead, all-or-nothing commit, extension on deviation, and confirmed-departure release — a window expiring is **not** a resource being released. `static` remains the default.
   - **Verified end-to-end**: `scripts/check_vda5050_e2e.sh` Stage 7 (two VDA 5050 vehicles, isolated ROS domain, no ROS trackers allowed to stand in) and Stage 8 (two local simulated vehicles, each with its own tracker/gate/chassis on two **disjoint** corridors) both assert genuine concurrency — simultaneous in-flight windows, no cross-talk, no cross-release.

---

## 🗺️ Custom Map Ingestion & Topology Engine

The platform natively supports importing official ROS 2 OccupancyGrid maps (`nav2_map_server` standard yaml/pgm) and automatically converting free space into executable dispatch topologies:

1. **Map Package Layout**:
   ```text
   custom_warehouse/
   ├── map.yaml         # Official Nav2 map configuration
   ├── map.pgm          # P5 binary occupancy grid (254=free, 0=wall, 205/-1=unknown)
   └── markers.yaml     # Semantic business stations (anchors)
   ```
2. **Markers Configuration (`markers.yaml`)**:
   ```yaml
   anchors:
     - id: "station_pickup_A"
       type: "pickup"
       x: 1.5
       y: 2.0
       yaw: 0.0
     - id: "station_dropoff_B"
       type: "dropoff"
       x: 8.0
       y: 5.5
       yaw: 1.57
   ```
3. **Launch with Custom Map Ingestion**:
   ```bash
   ros2 launch amr_dispatcher_ros full_system.launch.py \
     use_map_server:=true \
     map_yaml:=/path/to/custom_warehouse/map.yaml \
     markers_file:=/path/to/custom_warehouse/markers.yaml
   ```
4. **Automated Zero-ROS Processing Pipeline**:
   - **Safety Inflation & Unknown Handling**: Erodes free space using circular kernel ($R = R_{\text{robot}} + R_{\text{margin}}$); strictly treats unknown cells (`-1`) as impassable obstacles.
   - **Zhang-Suen Thinning**: Extracts a 1-pixel-wide corridor skeleton.
   - **Protected Pruning**: Removes dead-end whiskers shorter than `min_branch_length_m` (default 0.5m) while strictly exempting branches hosting target stations.
   - **Orthogonal Edge Splitting**: Snaps stations to the nearest skeleton edge, dynamically splitting the edge into sub-edges with preserved `polyline` geometry.
   - **Atomic Hot-Reload & Dispatch Execution**: Emits `config/generated/stations.yaml` via atomic `tmp+rename`, watches `map_hash` for version-gating, and executes dense 0.1m equal-arc-length trajectory tracking via `SkeletonPlanner` without cutting corner walls.
   - **3D RViz Topology**: Publishes white road-node spheres, type-colored station cylinders, and cyan polyline edge strips on `/dispatcher/topology_markers`.

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
   │             dispatcher_node                         │
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
   │  • Async Ring-Buffer Logger (Low-Contention)        │
   └─────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start & Automated Verification

### 1. Run Complete Automated CI & Chaos Pipeline (Host Linux)
```bash
./scripts/check_all.sh
```
Executes all 6 stages in one go, recording every stage honestly into `test_report_matrix.json` (stages that cannot run on this machine are marked SKIPPED, never faked):
- [Step 1] Build pure C++ core and run 659 GTest unit & integration tests via ctest (~5s, 100% green; count evolves with the test suite — your run's exact numbers land in the matrix).
- [Step 2] Benchmark mission queue and async logger throughput (measured live on your machine).
- [Step 3] Run amr_dispatcher_bt behavior-tree tests via ctest (SKIPPED on hosts without a colcon build of the BT package).
- [Step 4] WFG deadlock detection and dynamic detour chaos drill.
- [Step 5] Software watchdog 200ms timeout shutdown drill.
- [Step 6] Hardware loss rate failover drill.

### 2. Run Map Ingestion & Topology Dispatch E2E Verification
```bash
./scripts/check_map_ingest_e2e.sh
```
Automated end-to-end integration test verifying `nav2_map_server` -> `map_ingest_node` (skeletonization, pruning, polyline extraction) -> `dispatcher_node` (atomic route locks, SkeletonPlanner execution along L-shaped corridor) -> Goal reached without cutting wall corners.

### 3. Run VDA 5050 Fleet Adapter E2E Verification
```bash
./scripts/check_vda5050_e2e.sh
```
Self-contained, idempotent (self-cleaning) acceptance harness for the MQTT fleet adapter — spins up an isolated stack (mosquitto :18884, dispatcher, `vda5050_gateway_node`, REST gateway, MQTT packet capturer, virtual AMR) on `ROS_DOMAIN_ID=77`, then asserts across 9 stages:
- Stage 0: `scripts/mock_vda5050_vehicle.py --self-test` (T1-T7 virtual-vehicle contract tests: order parsing, polyline stepping, released=false braking, state-file resume).
- Stage 4: full closed loop for mission `e2e_m1` — wire-contract validation (orderUpdateId, monotonic sequenceId, >=2 control points), physical trajectory continuity along the polyline, Spec 5.3 trailing unlock, mission completion with zero residual locks.
- Stage 5: protocol-level `released=false` halt at an unreleased edge boundary, then resume after an order update carrying `released=true`.
- Stage 6: disconnect guard chaos drill — `kill -9` the vehicle, LWT `CONNECTIONBROKEN`, locks retained (never released while offline), dispatch suspended, state persisted across restart, automatic re-verification, mission resumption, and queued mission auto-dispatch wakeup.
- Stage 6.8: operator force-clear — force-clearing an **online** vehicle is refused; a second `kill -9` puts the vehicle offline, the retained locks are proven still held (iron rule), the force-clear service then releases exactly that vehicle's locks, aborts its in-flight mission, and unblocks dispatch for the next queued mission.
- Stage 7: **dual VDA 5050 vehicle concurrency** (isolated `ROS_DOMAIN_ID=78`) — two standard-protocol vehicles, each explicitly bound to one mission; asserts they fly **simultaneously**, that each vehicle receives only its own order, that both missions complete, and that the lock table drains. No ROS tracker is allowed to stand in for a VDA 5050 vehicle here.
- Stage 8: **dual local simulated vehicle concurrency** (isolated `ROS_DOMAIN_ID=80`) — two `local_sim` vehicles, each with its own `path_tracker` / `safety_gate` / `chassis` on two **disjoint** corridors of the demo map; asserts simultaneous dispatch, that each vehicle really moved (odometry travel) with non-crossed starting poses, both missions completed, and zero residual locks.
On a host machine the script automatically re-executes inside the `amr_dispatcher` Docker container (requires `python3-paho-mqtt`, `mosquitto`, `mosquitto-clients`; all preinstalled by `docker/Dockerfile.jazzy`).

### 4. Launch Physical Serial Device Simulator
```bash
python3 scripts/mock_physical_chassis.py
```
Simulates a real Linux PTY tty device at 50Hz, supporting hardware unplugging tests.

### 5. Open Web Operator Console
Simply launch `rest_gateway` or open in any browser:
```text
http://127.0.0.1:8080/
# or open tools/operator_console.html directly
```
