# Q2 — RAII and Exception Safety

Explain why the following function is not exception-safe, then rewrite it using a RAII wrapper. Your rewrite must guarantee the file is closed even if `process(f)` throws.

```cpp
void read_sensor_log(const char* path) {
    FILE* f = std::fopen(path, "r");
    if (!f) throw std::runtime_error("open failed");

    process(f);    // may throw

    std::fclose(f);
}
```

<details>
<summary>Answer</summary>

**Why it's unsafe:** If `process(f)` throws an exception, the call stack unwinds and `std::fclose(f)` is never reached. The file handle leaks for the lifetime of the process. A `return` statement added before `fclose` for any early-exit path causes the same problem.

**RAII rewrite:**

```cpp
class FileGuard {
public:
    explicit FileGuard(const char* path, const char* mode)
        : file_{std::fopen(path, mode)}
    {
        if (!file_) throw std::runtime_error("open failed");
    }

    ~FileGuard() {
        if (file_) std::fclose(file_);
    }

    FileGuard(const FileGuard&)            = delete;
    FileGuard& operator=(const FileGuard&) = delete;

    FILE* get() { return file_; }

private:
    FILE* file_;
};

void read_sensor_log(const char* path) {
    FileGuard f{path, "r"};   // acquired here
    process(f.get());          // may throw — doesn't matter
}   // FileGuard destructor runs here whether we exit normally or via exception
```

The destructor is guaranteed to run on any scope exit — normal return, exception, or early return. The file cannot leak.

</details>
