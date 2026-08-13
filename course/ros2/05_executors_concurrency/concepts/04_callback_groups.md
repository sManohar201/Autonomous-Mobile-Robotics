# 3. Callback Groups

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
