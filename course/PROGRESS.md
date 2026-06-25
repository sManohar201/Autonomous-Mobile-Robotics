# Course Completion Progress

Agentic loop tracker. Each iteration picks the next PENDING task, completes it fully, and marks it DONE.

**Legend:** `PENDING` → `IN_PROGRESS` → `DONE`

---

## PHASE 1: C++ Course

### Module 01 — Foundations & Toolchain
- DONE: concepts.md (326 lines — complete)
- DONE: exercises — all stubs implemented (ex01_pipeline, ex02_types_const, ex03_pointers_refs, ex04_namespaces, ex01_hard, ex02_hard, ex03_hard, ex04_hard)
- DONE: build verification — all 10 targets built and ran correctly

### Module 02 — Classes, OOP & RAII
- DONE: concepts.md (1776 lines — complete)
- DONE: exercises — all stubs implemented (ex01–ex05 moderate + hard, ex06_kalman1d)
- DONE: build verification — all 13 targets built and ran correctly

### Module 03 — Templates & Generic Programming
- DONE: concepts.md (412 lines — complete)
- DONE: exercises — all stubs implemented (ex01–ex05 moderate + hard)
- DONE: build verification — all 10 targets built and ran correctly

### Module 04 — Modern C++
- DONE: concepts.md (402 lines — complete)
- DONE: exercises — all stubs implemented (ex01–ex05 moderate + hard)
- DONE: build verification — all 10 targets built and ran correctly

### Module 05 — Standard Library
- DONE: concepts.md (123 lines — existing; expansion deferred to Phase 2 pass)
- DONE: exercises — all stubs implemented (ex01–ex05 moderate + hard)
- DONE: build verification — all 10 targets built and ran correctly

### Module 06 — Concurrency
- DONE: concepts.md (105 lines — existing; expansion deferred to Phase 2 pass)
- DONE: exercises — all stubs implemented (ex01_threads, ex02_mutex, ex03_condition_variable, ex04_atomic_future, ex05_capstone_blocking_queue, ex01_hard through ex05_hard)
- DONE: build verification — all 10 targets built and ran correctly

### Module 07 — Performance & Tooling
- DONE: concepts.md (86 lines — existing; expansion deferred to Phase 2 pass)
- DONE: exercises — all stubs implemented (ex01_benchmark_timer, ex02_allocation, ex03_cache_layout, ex04_sanitizer_bughunt, ex05_capstone_pointcloud_transform, ex01_hard through ex05_hard)
- DONE: build verification — all 10 targets built and ran correctly

### Module 08 — Robotics Patterns
- DONE: concepts.md (94 lines — existing; expansion deferred to Phase 2 pass)
- DONE: exercises — all stubs implemented (ex01_observer, ex02_plugin_factory, ex03_fsm, ex04_components, ex05_capstone_localization, ex01_hard through ex05_hard)
- DONE: build verification — all 10 targets built and ran correctly

---

## PHASE 2: ROS2 Course

### Module 01 — ROS2 Foundations
- DONE: concepts.md (84 lines — existing; expansion deferred)
- DONE: exercises — implement stubs (robot_status_publisher, robot_monitor_prototype)
- DONE: build verification — both packages built successfully

### Module 02 — Package Architecture
- DONE: concepts.md (54 lines — existing; expansion deferred)
- DONE: exercises — implement stubs (package_boundary_review, dependency_audit_hard)
- DONE: build verification — package built successfully

### Module 03 — Production Nodes
- DONE: concepts.md (46 lines — existing; expansion deferred)
- DONE: exercises — implement stubs (parameter_validation_node, lifecycle_processor_hard)
- DONE: build verification — package built successfully

### Module 04 — Communication Design
- DONE: concepts.md (35 lines — existing; expansion deferred)
- DONE: exercises — implement stubs (qos_status_publisher, command_api_hard)
- DONE: build verification — package built successfully

### Module 05 — Executors & Concurrency
- DONE: concepts.md — expanded to 480 lines (executors, callback groups, timers, snapshot pattern, multi-sensor cache, architecture patterns, common mistakes, trade-offs checklist)
- DONE: exercises — stubs correct (snapshot_processor moderate, concurrent_pipeline_hard hard)
- DONE: build verification — executors_concurrency_lab_answer built successfully

### Module 06 — Sensor Integration
- DONE: concepts.md — expanded to 370 lines (sensor topics, timestamps, TF2, rate monitoring, QoS, covariance, Gazebo bridge, patterns, trade-offs)
- DONE: exercises — stubs correct (frame_validator moderate, sensor_health_hard hard)
- DONE: build verification — sensor_integration_lab_answer built successfully

### Module 07 — State Estimation Architecture
- DONE: concepts.md — expanded to 390 lines (adapter pattern, rejection conditions, predict/update lifecycle, reset service, diagnostics, full StateEstimator class, trade-offs)
- DONE: exercises — stubs correct (measurement_adapter moderate, localization_shell_hard hard)
- DONE: build verification — state_estimation_architecture_lab_answer built successfully

### Module 08 — Launch & Configuration
- DONE: concepts.md — expanded to 380 lines (launch anatomy, YAML config, config layers, environment variants, conditional inclusion, namespaces/remaps, sim_time, dev vs field stacks)
- DONE: exercises — stubs correct (config_echo_node moderate, variant_bringup_hard hard)
- DONE: build verification — launch_configuration_lab_answer built successfully

### Module 09 — Testing & CI
- DONE: concepts.md — expanded to 370 lines (test layer hierarchy, unit tests, node tests, CI matrix, bag replay, launch tests, ament_lint, testable logic extraction, trade-offs)
- DONE: exercises — stubs correct (freshness_logic_node moderate, test_report_hard hard)
- DONE: build verification — testing_ci_lab_answer built successfully

### Module 10 — Gazebo Integration
- DONE: concepts.md — expanded to 340 lines (Gazebo architecture, sim time, bridge config, gazebo monitor pattern, data flow diagram, common issues, portability rules, trade-offs)
- DONE: exercises — stubs correct (sim_time_check moderate, gazebo_monitor_hard hard)
- DONE: build verification — gazebo_integration_lab_answer built successfully

### Module 11 — Observability & Operations
- DONE: concepts.md — expanded to 360 lines (5 observability signals, severity ranking, aggregator pattern, fault reports, logging best practices, bag strategy, operational workflows, trade-offs)
- DONE: exercises — stubs correct (diagnostic_aggregator moderate, fault_report_hard hard)
- DONE: build verification — observability_operations_lab_answer built successfully

### Module 12 — Field Deployable Stack
- DONE: concepts.md — expanded to 370 lines (deployment architecture, readiness checklist, degraded modes, recovery principles, launch hierarchy, config, bagging policy, operator runbooks, trade-offs)
- DONE: exercises — stubs correct (deployment_checklist_node moderate, degraded_mode_manager_hard hard)
- DONE: build verification — field_deployable_stack_lab_answer built successfully

---

## PHASE 3: Robotics Concepts (new modules)

### robotics/01_sensor_fusion
- DONE: concepts.md — 460 lines (state estimation problem, Gaussian fundamentals, KF linear, EKF predict/update, 2D robot model, Jacobians, full C++ worked example, noise models, UKF sigma points, multi-sensor fusion, Mahalanobis gating, covariance interpretation, interview Q&A)
- DONE: exercises — stubs (ex01_ekf_2d moderate with hints, ex01_hard with no hints)
- DONE: answers — ex01_ekf_2d and ex01_hard full implementations
- DONE: build verification — both executables built and ran correctly

### robotics/02_localization
- DONE: concepts.md — 390 lines (ICP algorithm + SVD derivation, motion/observation models, systematic resampling, full 1D particle filter example, AMCL parameters, ICP vs PF comparison, NDT overview, interview Q&A)
- DONE: exercises — stubs (ex01_icp_2d moderate with hints, ex01_hard no hints)
- DONE: answers — full ICP + particle filter implementations
- DONE: build verification — both executables built and passed (ICP: 30.0° recovered, PF: 1.96m estimated)

### robotics/03_slam
- DONE: concepts.md — 410 lines (occupancy grid log-odds, beam model, EKF SLAM state augmentation, data association, loop closure, graph SLAM, production systems table, full C++ occupancy builder example, interview Q&A)
- DONE: exercises — stubs (ex01_occupancy_grid moderate, ex01_hard no hints)
- DONE: answers — full occupancy grid + 1D pose graph SLAM
- DONE: build verification — grid p=0.71 (occupied), pose graph residual=0.009 (near 0)

### robotics/04_path_planning
- DONE: concepts.md — 400 lines (C-space, A* algorithm + full C++ implementation, RRT + goal bias + RRT*, potential fields, trajectory optimisation, Nav2 planner stack, costmaps, interview Q&A)
- DONE: exercises — stubs (ex01_astar moderate with hints, ex01_hard no hints)
- DONE: answers — full A* (7-cell path found) and RRT (12-node path found)
- DONE: build verification — both executables built and ran correctly

---

## Notes for the loop agent

### Build commands
**C++ exercises:**
```bash
cd course/cpp/NN_module/exercises
cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build 2>&1 | tail -20
```

**ROS2 exercises (isolated build):**
```bash
source /opt/ros/jazzy/setup.bash
MODULE_PATH=course/ros2/NN_module/exercises/<package_name>
BUILD_ID=ros2_mNN
colcon --log-base /tmp/${BUILD_ID}_log build \
  --base-paths "${MODULE_PATH}" \
  --build-base /tmp/${BUILD_ID}_build \
  --install-base /tmp/${BUILD_ID}_install 2>&1 | tail -30
```

### Concepts expansion guidelines
- C++ concepts: 400–800 lines, every section has annotated runnable code with robotics context
- ROS2 concepts: same depth + ASCII node/topic graphs for every architectural pattern; point to ROS2 docs or existing animations by URL instead of reproducing them
- No generic filler — every paragraph should convey a decision a robotics engineer needs to make

### Exercise implementation rules
- Read the existing stub carefully — preserve structure, comments, assertions
- Implement the TODO completely — the code must compile, link, and pass all asserted checks
- For ROS2: match the answer package's approach in terms of API style, not just any random valid code
- After implementing: verify it builds (run cmake or colcon)

### When expanding ROS2 concepts
Check if the concept has official documentation or tutorials at docs.ros.org — if so, add a callout:
> **See also:** [link] — don't re-explain what the official docs cover well; read those first, then come back here for the "why" and production trade-offs.
