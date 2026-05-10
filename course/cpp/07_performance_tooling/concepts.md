# Module 7 - Performance & Tooling

**Prerequisites:** Modules 1-6
**Goal:** Improve robotics C++ performance by measuring first, understanding memory behavior, and using compiler/runtime tools correctly.

---

## 7.1 Measure First

Performance work without measurement is guessing. Start with a clear workload, a baseline, and repeatable timing.

Use `std::chrono::steady_clock` for simple local measurements. Use profilers such as `perf`, `callgrind`, or tracing tools for real systems.

---

## 7.2 Big-O Is Not Enough

Two `O(n)` algorithms can differ massively because of cache locality, allocation, branching, and vectorization. Robotics pipelines often process large arrays of points, scans, or grid cells, where memory access dominates.

---

## 7.3 Allocation Discipline

Repeated allocation in hot paths causes jitter. Use:

- `reserve` for vectors
- object reuse for frame buffers
- fixed-size storage where dimensions are known
- bounded queues for real-time pipelines

---

## 7.4 Cache Locality

Contiguous memory is faster to traverse than scattered allocations. Prefer `std::vector<Point>` over `std::list<Point>` for point clouds and scans.

Structure-of-arrays can outperform array-of-structures for SIMD-heavy numeric kernels, but it complicates APIs. Measure before changing representation.

---

## 7.5 SIMD Basics

SIMD executes the same operation over multiple values. Compilers can auto-vectorize simple loops if data is contiguous and dependencies are clear. Write simple loops first; inspect compiler output only when performance matters.

---

## 7.6 Sanitizers

Sanitizers catch bugs early:

- AddressSanitizer: memory errors
- UndefinedBehaviorSanitizer: undefined behavior
- ThreadSanitizer: data races

They are not production runtime dependencies; they are development/test tools.

---

## 7.7 Debuggers and Profilers

Use GDB for correctness bugs and `perf` for CPU profiling. Logging can perturb timing and hide races. Prefer targeted instrumentation.

---

## 7.8 CMake Build Types

Debug builds are for debugging. Release builds are for performance measurement.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Do not compare performance numbers from Debug builds.

---

## 7.9 Practical Rules

- Measure before optimizing.
- Benchmark Release builds.
- Avoid allocations in hot callbacks.
- Prefer contiguous storage.
- Keep numeric loops simple.
- Use sanitizers before profiling.

