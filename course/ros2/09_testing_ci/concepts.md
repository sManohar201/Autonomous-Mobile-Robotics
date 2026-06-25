# ROS2 Module 09 — Testing & CI

## Goal

Test ROS2 systems at multiple layers — pure C++ logic, node behaviour, process
wiring, bag replay, and simulation integration. Each layer catches different bugs
at different costs. Use the cheapest test that catches a given class of bug.

> **See also:** https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html
> for the official ROS2 testing guide. This module covers the architectural
> decisions behind each test layer.

---

## 1. Test Layer Hierarchy

```
                          cost / realism
                              high ▲
                                   │
                    5. Simulation tests (Gazebo)
                    4. Bag replay tests
                    3. Launch tests (process-level)
                    2. Node tests (single node + synthetic topics)
                    1. Unit tests (pure C++, no ROS2)
                                   │
                              low  ▼
```

Use the cheapest test that catches the bug. Simulation tests are expensive —
don't write one to test a pure function.

---

## 2. Unit Tests — Pure C++ Logic

Logic that doesn't need a ROS2 node can be tested with standard C++ test
frameworks (GTest is the ROS2 standard).

### Freshness logic example

```cpp
bool is_stale(const rclcpp::Time& now,
              const rclcpp::Time& last_seen,
              double timeout_s) {
    if (last_seen.nanoseconds() == 0) {
        return true;  // never seen
    }
    return (now - last_seen).seconds() > timeout_s;
}
```

This function takes no `rclcpp::Node*` — it's a pure computation and can be
tested without spinning an executor:

```cpp
// GTest
TEST(FreshnessTest, NeverSeenIsStale) {
    rclcpp::Time now{1000000000LL, RCL_ROS_TIME};
    rclcpp::Time never{0, 0, RCL_ROS_TIME};
    EXPECT_TRUE(is_stale(now, never, 1.0));
}

TEST(FreshnessTest, FreshIsNotStale) {
    rclcpp::Time t0{1000000000LL, RCL_ROS_TIME};
    rclcpp::Time t1{1000500000LL, RCL_ROS_TIME};  // 0.5 s later
    EXPECT_FALSE(is_stale(t1, t0, 1.0));
}

TEST(FreshnessTest, StaleAfterTimeout) {
    rclcpp::Time t0{1000000000LL, RCL_ROS_TIME};
    rclcpp::Time t1{2000000000LL, RCL_ROS_TIME};  // 1 s later
    EXPECT_TRUE(is_stale(t1, t0, 0.5));
}
```

Key rule: if you need to call `rclcpp::init()` to test it, it's not a unit test.
Extract logic into free functions or classes that take clock values as arguments.

---

## 3. Node Tests — Single Node with Synthetic Topics

Node tests spin a real node and inject synthetic input via `ros2 topic pub` or
a test publisher. They verify that the node's topic graph behaves correctly.

### The freshness_logic_node

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

bool is_stale(const rclcpp::Time& now, const rclcpp::Time& last_seen,
              double timeout_s) {
    return last_seen.nanoseconds() == 0 ||
           (now - last_seen).seconds() > timeout_s;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("freshness_logic_node");
    node->declare_parameter<double>("timeout_s", 1.0);

    auto last_seen = std::make_shared<rclcpp::Time>(0, 0, RCL_ROS_TIME);

    auto input = node->create_subscription<std_msgs::msg::String>(
        "test/input", 10,
        [node, last_seen](const std_msgs::msg::String::SharedPtr) {
            *last_seen = node->now();
        });

    auto output = node->create_publisher<std_msgs::msg::String>(
        "test/freshness", 10);

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(200),
        [node, last_seen, output] {
            const double timeout = node->get_parameter("timeout_s").as_double();
            std::ostringstream s;
            s << "state="
              << (is_stale(node->now(), *last_seen, timeout) ? "STALE" : "OK")
              << " timeout_s=" << timeout
              << " last_seen_ns=" << last_seen->nanoseconds();
            std_msgs::msg::String msg;
            msg.data = s.str();
            output->publish(msg);
        });

    (void)input; (void)timer;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Testing this node manually

```bash
# Terminal 1: run the node
ros2 run testing_ci_lab freshness_logic_node

# Terminal 2: inject input
ros2 topic pub /test/input std_msgs/msg/String '{}' --rate 2

# Terminal 3: watch output
ros2 topic echo /test/freshness

# Expected: state=OK while /test/input is publishing
# Expected: state=STALE after > 1 s of silence
```

---

## 4. CI Test Matrix

Production CI pipelines have distinct jobs with different triggers:

| Job | Trigger | Artifact |
|---|---|---|
| `build` | Every PR | Install log, no error exit |
| `unit` | Every PR | GTest/JUnit XML — visible in CI |
| `launch` | Every PR | Launch test log |
| `lint` | Every PR | ament_lint report |
| `simulation` | Nightly or gated | Bag replay summary |

### Why separate simulation tests?

Simulation tests start Gazebo, which takes 10–30 seconds. Running them on every
PR slows the feedback loop. They are better as nightly jobs or triggered
explicitly for merge candidates.

### CI job structure (GitHub Actions style)

```yaml
jobs:
  build_and_test:
    steps:
      - name: Build
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -20

      - name: Unit tests
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon test --packages-select testing_ci_lab
          colcon test-result --verbose

      - name: Lint
        run: |
          source /opt/ros/jazzy/setup.bash
          ament_cpplint --filter=-whitespace/newline src/

  nightly_sim:
    runs-on: ...
    if: github.event_name == 'schedule'
    steps:
      - name: Bag replay test
        run: python3 test/bag_replay_test.py
```

---

## 5. The Test Report Node

The `test_report_hard` node models the CI matrix as structured data. In a real
system it could query a CI API; here it demonstrates the pattern of publishing
structured diagnostic information:

```cpp
struct CiJob {
    const char* name;
    const char* trigger;
    const char* required_artifact;
};

constexpr std::array<CiJob, 5> jobs{{
    {"build",      "every PR",             "install log"},
    {"unit",       "every PR",             "gtest/junit"},
    {"launch",     "every PR",             "launch test log"},
    {"lint",       "every PR",             "ament lint report"},
    {"simulation", "nightly or gated",     "bag replay summary"},
}};

// Timer publishes the matrix to /ci/test_matrix every 1 s
```

---

## 6. Bag Replay Testing

A ROS2 bag records all topics to disk. Replay is deterministic — the same bag
always produces the same sequence of messages.

```bash
# Record a bag
ros2 bag record -a -o my_test_bag

# Replay
ros2 bag play my_test_bag

# List what's in a bag
ros2 bag info my_test_bag
```

### Bag replay test pattern

1. Record a bag with known input (from simulation or real sensor)
2. Start the node under test
3. Replay the bag
4. Subscribe to the output topic and assert expected values

```python
# Simplified bag replay test pseudocode
import subprocess, rclpy, threading

def test_freshness_node_with_bag():
    rclpy.init()
    node = rclpy.create_node('test_harness')
    results = []
    sub = node.create_subscription(String, '/test/freshness',
                                   lambda m: results.append(m.data), 10)
    # Start the node under test
    proc = subprocess.Popen(['ros2', 'run', 'testing_ci_lab', 'freshness_logic_node'])
    # Replay a pre-recorded bag
    subprocess.run(['ros2', 'bag', 'play', 'test_bags/normal_input.bag3'])
    # Spin briefly then check
    rclpy.spin_once(node, timeout_sec=2.0)
    assert any('state=OK' in r for r in results)
    proc.terminate()
```

> **Production trade-off:** Bags drift over time as message definitions change.
> Store bags in a separate versioned artifact store, not in the git repo.
> Keep bags small — record only the topics your test needs.

---

## 7. Launch Tests

Launch tests start a full process graph (via a launch file) and assert that
topics appear, services respond, and nodes don't crash.

```python
# test/test_freshness_launch.py
import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from std_msgs.msg import String

@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='testing_ci_lab',
            executable='freshness_logic_node',
        ),
        launch_testing.actions.ReadyToTest(),
    ])

class TestFreshnessNodeLaunch(launch_testing.TestCase):
    def test_node_publishes(self, proc_output):
        rclpy.init()
        node = rclpy.create_node('test')
        messages = []
        sub = node.create_subscription(String, '/test/freshness',
                                        lambda m: messages.append(m), 10)
        rclpy.spin_once(node, timeout_sec=3.0)
        self.assertTrue(len(messages) > 0, "No messages on /test/freshness")
```

Run with:
```bash
ros2 launch --test testing_ci_lab test_freshness_launch.py
# or via colcon test
```

---

## 8. ament_lint

`ament_lint_auto` runs several linting tools automatically:

```bash
ament_cpplint --filter=-whitespace/newline src/
ament_cppcheck src/
ament_uncrustify src/
```

Add to `CMakeLists.txt`:
```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

And to `package.xml`:
```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
```

---

## 9. Extracting Testable Logic

The most important testing pattern: **extract pure functions from node callbacks**.

```cpp
// Testable pure function
bool is_stale(const rclcpp::Time& now,
              const rclcpp::Time& last_seen,
              double timeout_s);

// Node thin wrapper — tests only that wiring is correct
auto timer = node->create_wall_timer(200ms, [node, last_seen, output] {
    const auto verdict = is_stale(node->now(), *last_seen,
                                  node->get_parameter("timeout_s").as_double());
    // ...publish verdict
});
```

If `is_stale` is tested by 5 unit tests, the node test only needs to verify
one case: that the output topic reflects the function's verdict. The unit tests
cover all the edge cases far more cheaply.

---

## 10. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Unit vs node test | Unit: fast, no ROS2 startup. Node: slower, tests integration |
| GTest vs pytest | GTest: C++ native, no subprocess. pytest: Python, easier async |
| Bags vs synthetic publishers | Bags: real data, bit-for-bit deterministic. Synthetic: controllable, no storage |
| Lint on every PR | Catches style drift early. Some false positives need filter configuration |
| Simulation in CI | High realism but slow and flaky. Gate behind label or run nightly |

---

## 11. Checklist

- [ ] Every pure function (freshness, rejection, adapter) has GTest unit tests
- [ ] Node tests use wall-time with explicit timeouts — no `sleep(3)`
- [ ] Bags stored outside git (git-lfs or separate artifact store)
- [ ] CI runs build + unit + lint on every PR; simulation is gated or nightly
- [ ] `colcon test-result --verbose` is the final CI check — zero failures required
