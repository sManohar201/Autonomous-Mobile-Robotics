# 7. Launch Tests

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
