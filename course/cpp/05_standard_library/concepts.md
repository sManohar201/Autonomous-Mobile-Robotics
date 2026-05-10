# Module 5 - Standard Library

**Prerequisites:** Modules 1-4
**Goal:** Use the C++ standard library as the default toolbox for robotics data structures, algorithms, time handling, and text views.

---

## 5.1 Why the STL Matters

Robotics systems move data through queues, maps, windows, transforms, and filters. The standard library already provides well-tested building blocks for these jobs. Prefer STL containers and algorithms until measurement proves they are not enough.

---

## 5.2 Containers

Use `std::vector` for contiguous dynamic arrays and numeric data. Use `std::deque` when pushing and popping at both ends. Use `std::array` for fixed-size stack storage. Use `std::map` or `std::unordered_map` for key/value lookup.

```cpp
std::vector<double> ranges;
std::array<double, 3> accel{};
std::deque<ImuSample> imu_window;
std::unordered_map<std::string, double> rates;
```

Container choice is an engineering decision about access pattern, allocation, ordering, and invalidation.

---

## 5.3 Iterators

Iterators generalize positions in a sequence. Algorithms operate on iterator ranges:

```cpp
std::sort(values.begin(), values.end());
auto it = std::find(values.begin(), values.end(), 42);
```

The half-open range `[begin, end)` is standard: begin is included, end is one past the last element.

---

## 5.4 Algorithms

Algorithms express intent:

- `std::find_if` for search
- `std::count_if` for counting matches
- `std::transform` for mapping values
- `std::accumulate` for reductions
- `std::remove_if` plus `erase` for filtering containers
- `std::lower_bound` for binary search over sorted ranges

Prefer algorithms when they make the code easier to audit.

---

## 5.5 Erase-Remove

`std::remove_if` rearranges elements and returns the new logical end. It does not shrink the container. Pair it with `erase`:

```cpp
values.erase(std::remove_if(values.begin(), values.end(), pred), values.end());
```

This pattern appears often when dropping invalid sensor readings.

---

## 5.6 `std::chrono`

Use `std::chrono` instead of raw integers for time.

```cpp
using Clock = std::chrono::steady_clock;
auto start = Clock::now();
std::chrono::milliseconds timeout{100};
```

`steady_clock` is monotonic and appropriate for measuring durations. System time can jump and is not appropriate for elapsed-time logic.

---

## 5.7 `std::string_view`

`std::string_view` is a non-owning view into character data. It avoids copies but does not extend lifetime.

```cpp
void parse_key(std::string_view key);
```

Do not store a `string_view` to a temporary string. Store `std::string` when ownership is needed.

---

## 5.8 Custom Allocators

Allocators control how containers acquire memory. Most robotics code should not write custom allocators early. First use `reserve`, fixed-capacity containers, and clear ownership boundaries. Allocators matter in real-time systems when dynamic allocation must be bounded or eliminated.

---

## 5.9 Time-Based Windows

A time window stores recent samples and evicts old ones:

```cpp
while (!samples.empty() && now - samples.front().stamp > window) {
    samples.pop_front();
}
```

Use `std::deque` for efficient front removal and back insertion.

---

## 5.10 Practical Rules

- Default to `std::vector` unless another access pattern is clearly better.
- Call `reserve` when the approximate size is known.
- Use `std::array` for fixed dimensions.
- Use `std::chrono` for durations and deadlines.
- Use `std::string_view` only for non-owning parameters.
- Prefer algorithms for search, transform, reduction, and filtering.

