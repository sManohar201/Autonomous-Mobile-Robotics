# 1. Test Layer Hierarchy

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
