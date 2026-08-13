# 4. CI Test Matrix

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
