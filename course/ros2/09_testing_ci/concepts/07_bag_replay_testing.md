# 6. Bag Replay Testing

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
