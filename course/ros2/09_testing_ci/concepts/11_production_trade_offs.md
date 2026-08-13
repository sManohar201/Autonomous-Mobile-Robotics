# 10. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Unit vs node test | Unit: fast, no ROS2 startup. Node: slower, tests integration |
| GTest vs pytest | GTest: C++ native, no subprocess. pytest: Python, easier async |
| Bags vs synthetic publishers | Bags: real data, bit-for-bit deterministic. Synthetic: controllable, no storage |
| Lint on every PR | Catches style drift early. Some false positives need filter configuration |
| Simulation in CI | High realism but slow and flaky. Gate behind label or run nightly |
