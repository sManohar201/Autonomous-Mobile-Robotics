# 5. The Test Report Node

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
