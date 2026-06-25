# ROS2 Module 05 — Executors & Concurrency

## Goal

Understand how ROS2 dispatches callbacks, when callbacks can overlap, what
"thread-safe" means in a ROS2 node, and how to protect shared sensor caches
without either deadlocking or busy-waiting.

> **See also:** https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html
> The official page covers the executor lifecycle and the distinction between
> spinning strategies. Come back here for the production trade-offs that page
> doesn't address.

---

## 1. The Executor Model

### What an executor does

A ROS2 node does not run its own threads. It registers callbacks (subscriptions,
timers, services, action servers) and hands them to an **executor**. The executor
owns one or more threads and calls your callbacks on those threads.

```
rclcpp::Node  →  Executor  →  OS thread(s)  →  your callbacks
```

Without spinning the executor, **no callbacks fire**. `rclcpp::spin(node)` is
syntactic sugar for:

```cpp
auto exec = rclcpp::executors::SingleThreadedExecutor();
exec.add_node(node);
exec.spin();          // blocks here until shutdown
```

### The three built-in executors

| Executor | Threads | When to use |
|---|---|---|
| `SingleThreadedExecutor` | 1 | Default — no shared-state hazards, simplest mental model |
| `MultiThreadedExecutor(N)` | N | When you need concurrent callback processing |
| `StaticSingleThreadedExecutor` | 1 | Lower overhead — graph cannot change after spin starts |

```cpp
// Multi-threaded: 4 worker threads
rclcpp::executors::MultiThreadedExecutor exec{
    rclcpp::ExecutorOptions{}, 4};
exec.add_node(node);
exec.spin();
```

> **Production trade-off:** MultiThreadedExecutor only helps if the callback
> groups allow concurrency (see §3). Adding threads without configuring groups
> gives you race conditions for free without any throughput gain.

---

## 2. The Single-Threaded Execution Model

With `SingleThreadedExecutor` (the default), the executor processes one
callback at a time in round-robin order. This has a critical implication:

```
Thread 0:  [subscription_cb]  [timer_cb]  [service_cb]  [timer_cb]  ...
               ↑ one at a time, no overlap
```

**Consequence:** If your subscription callback takes 50 ms and your timer fires
every 20 ms, the timer will be late. Single-threaded execution serialises all
work.

```cpp
// Checking elapsed time in a timer — single-threaded is fine here
auto timer = node->create_wall_timer(20ms, [this] {
    // This runs every 20ms *if* other callbacks finish in time
    publish_status();
});
```

### When single-threaded is enough

- All callbacks are short (< 1 ms typical, < 5 ms worst-case)
- No blocking calls (no sleep, no file I/O, no slow network calls)
- Sensor fusion at 50–100 Hz: callbacks reading from the DDS queue are fast

Most production ROS2 nodes use `SingleThreadedExecutor` and simply keep
callbacks short.

---

## 3. Callback Groups

Callback groups control which callbacks are **allowed to run concurrently**
within a `MultiThreadedExecutor`.

```
                    MutuallyExclusive group
                    ┌─────────────────────┐
                    │  sub_A  sub_B  timer│  ← at most one runs at a time
                    └─────────────────────┘

                    Reentrant group
                    ┌─────────────────────┐
                    │  sub_A  sub_B  timer│  ← any combination may overlap
                    └─────────────────────┘
```

### MutuallyExclusive (default)

All callbacks in the group form a critical section. Even with 4 executor threads,
only one callback from this group runs at a time.

```cpp
auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

rclcpp::SubscriptionOptions opts;
opts.callback_group = group;

auto sub = node->create_subscription<std_msgs::msg::String>(
    "/scan", 10,
    [](const std_msgs::msg::String::SharedPtr msg) { /* safe: no overlap */ },
    opts);
```

Use this when callbacks share mutable state that a mutex would protect.
The group does the locking implicitly — no explicit mutex needed.

> **Caveat:** if two **different** MutuallyExclusive groups share state, you
> still need a mutex. A group only serialises within itself.

### Reentrant

Multiple callbacks from this group can run simultaneously on different threads.
Use only when:
- All shared state is protected by atomics or mutexes
- Or the callbacks are genuinely stateless

```cpp
auto reentrant_group = node->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
```

> **Production trade-off:** Reentrant groups are rarely the right answer.
> They require every callback to be thread-safe, which is hard to audit.
> Prefer MutuallyExclusive groups + explicit mutexes for shared sensor caches.

### Default group

Every callback you create without specifying a group goes into the node's
default MutuallyExclusive group. With `SingleThreadedExecutor`, this is
invisible — only one thread exists. With `MultiThreadedExecutor`, this default
group still serialises all unassigned callbacks.

---

## 4. Timers

Timers fire a callback periodically. They are callbacks like any other — subject
to executor scheduling and callback group rules.

```cpp
using namespace std::chrono_literals;

// Wall timer: fires every 100ms of real time
auto timer = node->create_wall_timer(100ms, [this] {
    publish_diagnostics();
});

// Timer with a lambda capturing shared state
auto pub = node->create_publisher<std_msgs::msg::String>("/status", 10);
auto timer2 = node->create_wall_timer(1s, [pub] {
    std_msgs::msg::String msg;
    msg.data = "alive";
    pub->publish(msg);
});
```

### Timer accuracy

`create_wall_timer` uses real (wall) time, not sim time. For simulation:

```cpp
// Sim-time aware timer — only fires when /clock advances
auto timer = node->create_timer(
    node->get_clock(),
    rclcpp::Duration::from_seconds(0.1),
    [this] { check_health(); });
```

> **See also:** https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros.html
> for sim-time patterns with TF2.

### Timer jitter sources

1. **Callback queuing delay** — if a long subscription callback is running,
   the timer callback waits in the ready queue.
2. **OS scheduling** — the executor thread is preempted.
3. **DDS discovery storms** — brief CPU spikes on graph changes.

Keep timer callbacks short. If processing takes > timer period, consider
off-loading to a separate thread with a bounded queue.

---

## 5. Shared State and the Snapshot Pattern

The most common concurrency pattern in sensor fusion nodes:

```
         Subscription callback (fired by executor)
              │
              ▼
         [Lock mutex] → update shared cache → [Unlock]
         
         Timer callback (fired by executor)
              │
              ▼
         [Lock mutex] → copy snapshot → [Unlock]
              │
         (process snapshot without holding lock)
              │
         [publish result]
```

### Why copy-then-release?

If you hold the mutex during processing, the subscription callback blocks
waiting for the lock. For a 50 Hz LiDAR, a 20 ms processing stall drops a scan.

```cpp
// WRONG: holds lock during processing
auto timer = node->create_wall_timer(100ms, [cache, mutex, pub] {
    std::lock_guard<std::mutex> lock(*mutex);
    auto result = expensive_process(*cache);  // lock held here — blocks subs!
    pub->publish(result);
});

// CORRECT: snapshot pattern — lock only for the copy
auto timer = node->create_wall_timer(100ms, [cache, mutex, pub] {
    std::optional<std::string> snapshot;
    { std::lock_guard<std::mutex> lock(*mutex); snapshot = *cache; }  // brief lock
    // Process outside the lock — subscription callbacks can update cache now
    std_msgs::msg::String out;
    out.data = snapshot ? "processed " + *snapshot : "missing input";
    pub->publish(out);
});
```

### Full snapshot processor example

```cpp
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("snapshot_processor");

    // Shared state — accessed by subscription + timer callbacks
    auto latest = std::make_shared<std::optional<std::string>>();
    auto mutex  = std::make_shared<std::mutex>();

    auto pub = node->create_publisher<std_msgs::msg::String>("/processed/status", 10);

    // Subscription: write under lock
    auto sub = node->create_subscription<std_msgs::msg::String>(
        "/input/status", 10,
        [latest, mutex](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(*mutex);
            *latest = msg->data;
        });

    // Timer: copy snapshot, release lock, then publish
    auto timer = node->create_wall_timer(100ms, [latest, mutex, pub] {
        std::optional<std::string> snapshot;
        { std::lock_guard<std::mutex> lock(*mutex); snapshot = *latest; }

        std_msgs::msg::String out;
        out.data = snapshot ? "processed " + *snapshot : "missing input";
        pub->publish(out);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

---

## 6. Multi-Sensor Cache with Callback Groups

When you have multiple independent sensor subscriptions feeding one output
topic, use a shared struct and a single mutex:

```
/scan_status  ──┐
/imu_status   ──┼──► [Cache struct] ──► [Timer 200ms] ──► /pipeline/summary
/odom_status  ──┘       (mutex)              (snapshot)
```

```cpp
struct Cache {
    std::optional<std::string> scan, imu, odom;
    std::mutex mutex;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node  = std::make_shared<rclcpp::Node>("sensor_pipeline");
    auto cache = std::make_shared<Cache>();

    // One MutuallyExclusive group for all subscribers
    auto group = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group;

    // Helper to create a subscriber with a field setter
    auto make_sub = [&](const std::string& topic, auto setter) {
        return node->create_subscription<std_msgs::msg::String>(
            topic, 10,
            [cache, setter](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(cache->mutex);
                setter(*cache, msg->data);
            },
            opts);
    };

    auto scan = make_sub("/scan_status",
        [](Cache& c, const std::string& v) { c.scan = v; });
    auto imu  = make_sub("/imu_status",
        [](Cache& c, const std::string& v) { c.imu  = v; });
    auto odom = make_sub("/odom_status",
        [](Cache& c, const std::string& v) { c.odom = v; });

    auto pub = node->create_publisher<std_msgs::msg::String>("/pipeline/summary", 10);

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(200),
        [cache, pub] {
            // Snapshot under lock
            std::optional<std::string> s, i, o;
            {
                std::lock_guard<std::mutex> lock(cache->mutex);
                s = cache->scan; i = cache->imu; o = cache->odom;
            }
            std_msgs::msg::String msg;
            msg.data = (s && i && o) ? "OK" : "MISSING_INPUTS";
            pub->publish(msg);
        });

    (void)scan; (void)imu; (void)odom;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

> **Production trade-off:** With `SingleThreadedExecutor`, the MutuallyExclusive
> group and the mutex are both redundant — only one callback runs at a time.
> But keeping both means the code is correct if you later switch executors.
> Cost: near-zero. Benefit: safe refactor later.

---

## 7. Executor Architecture Patterns

### Pattern A — Single node, single thread (most common)

```
main() → spin(node)
             │
    SingleThreadedExecutor
             │
     ┌───────┴───────┐
     │ sub_cb   timer│  serialised
     └───────────────┘
```

Use for: most sensor drivers, publishers, monitors.

### Pattern B — Single node, multi-thread with groups

```
main() → MultiThreadedExecutor(2)
               │
    ┌──────────┴──────────┐
    thread-0           thread-1
       │                  │
  [group_A_cb]       [group_B_cb]   ← can run simultaneously
```

Use for: high-frequency sensor ingestion + slow processing in same node.

### Pattern C — Multiple nodes, one executor

```cpp
auto exec = rclcpp::executors::SingleThreadedExecutor();
exec.add_node(driver_node);
exec.add_node(filter_node);
exec.spin();
```

Use for: composable node containers — both nodes share one thread, reducing
context-switch overhead versus separate processes.

---

## 8. Common Mistakes

### Mistake 1 — Blocking inside a callback

```cpp
auto sub = node->create_subscription<std_msgs::msg::String>(
    "/cmd", 10, [](const std_msgs::msg::String::SharedPtr msg) {
        std::this_thread::sleep_for(std::chrono::seconds(1));  // BLOCKS executor
    });
```

One blocked callback stalls every other callback sharing that executor thread.

**Fix:** Do heavy work in a separate std::thread or std::async. Post the result
back via a thread-safe queue read by a timer callback.

### Mistake 2 — Not protecting shared state

```cpp
// timer and subscription both write latest_ without a mutex — DATA RACE
std::string latest_;
auto sub = node->create_subscription<...>("/topic", 10,
    [this](const std_msgs::msg::String::SharedPtr m) { latest_ = m->data; });
auto timer = node->create_wall_timer(100ms,
    [this] { RCLCPP_INFO(get_logger(), latest_); });  // race!
```

Even with `SingleThreadedExecutor` this is *currently* safe, but becomes a race
the moment you add a `MultiThreadedExecutor`. Write it correctly from the start.

### Mistake 3 — Holding a lock while publishing

```cpp
// WRONG
std::lock_guard<std::mutex> lock(mutex_);
pub_->publish(msg);   // publish acquires internal DDS locks — may deadlock
```

Always release your mutex before calling `publish()`.

---

## 9. Production Trade-offs Summary

| Decision | Trade-off |
|---|---|
| SingleThreaded vs Multi | Single: simple, no races. Multi: concurrent but requires explicit sync |
| MutuallyExclusive group | Serialises group — safe for shared state, no mutex needed within group |
| Reentrant group | Parallel — requires every callback to be independently thread-safe |
| Snapshot vs in-place | Snapshot: extra copy cost, but lock held < 1 µs. In-place: stalls subs |
| Mutex vs atomic | Atomic only for single POD values. Mutex for struct or compound state |
| Timer period | Shorter = fresher output, higher CPU. Match to downstream consumer rate |

---

## 10. Checklist for a Production Concurrent Node

- [ ] All shared mutable state is protected by a mutex or lives in a MutuallyExclusive callback group
- [ ] Lock scope is as short as possible — no publishing, no heavy computation under the lock
- [ ] Timer callbacks have bounded worst-case runtime (no blocking calls)
- [ ] Executor choice is documented in a comment near `spin()`
- [ ] Callback group assignments are explicit (don't rely on default group behaviour changing)
- [ ] Node tested with `--ros-args --log-level DEBUG` to confirm callback firing rates
