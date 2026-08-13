# 2.7 RAII (Resource Acquisition Is Initialization)

RAII is the single most important C++ idiom. The rule: **acquire a resource in the constructor, release it in the destructor**. Because destructors run deterministically (scope exit, including exceptions), resource leaks become structurally impossible.

### The Non-RAII Version (Fragile)

```cpp
// BAD: manual resource management — leaks on any early exit.
void process_scan_bad(Lidar& lidar) {
    lidar.activate();    // powers on, starts motor spinning

    auto scan = lidar.get_scan();

    if (scan.empty()) {
        return;          // LEAK: lidar.deactivate() never called, motor keeps spinning
    }

    run_icp(scan);       // if this throws, deactivate() is also skipped

    lidar.deactivate();  // only reached on the happy path
}
```

### The RAII Version (Safe)

```cpp
#include <chrono>
#include <iostream>
#include <string>

// ── Example 1: Scoped sensor activation ─────────────────────────────────────
// Sensor is expensive to run continuously — activate only during a processing
// window, guaranteed to deactivate even if processing throws.

class ScopedSensor {
public:
    explicit ScopedSensor(Lidar& sensor) : sensor_(sensor) {
        sensor_.activate();    // powers on, starts spinning
    }
    ~ScopedSensor() {
        sensor_.deactivate();  // always runs — scope exit, return, or exception
    }
    ScopedSensor(const ScopedSensor&)            = delete;
    ScopedSensor& operator=(const ScopedSensor&) = delete;
private:
    Lidar& sensor_;
};

void process_scan_good(Lidar& lidar) {
    ScopedSensor guard(lidar);   // activate here
    auto scan = lidar.get_scan();
    run_icp(scan);               // if this throws, deactivate still runs
}                                // deactivate here — guaranteed


// ── Example 2: TF2 transform snapshot lock ───────────────────────────────────
// In ROS2, reading multiple transforms without locking the buffer means the
// transform tree can update between lookups — inconsistent state mid-callback.

class ScopedTfSnapshot {
public:
    explicit ScopedTfSnapshot(tf2_ros::Buffer& tf_buffer)
        : tf_buffer_(tf_buffer)
    {
        tf_buffer_.lock();    // freeze the transform tree
    }
    ~ScopedTfSnapshot() {
        tf_buffer_.unlock();  // new transforms can flow in again
    }
    ScopedTfSnapshot(const ScopedTfSnapshot&)            = delete;
    ScopedTfSnapshot& operator=(const ScopedTfSnapshot&) = delete;
private:
    tf2_ros::Buffer& tf_buffer_;
};

void on_lidar_msg(const sensor_msgs::msg::LaserScan& msg) {
    ScopedTfSnapshot snap(tf_buffer_);
    // all lookups here see a consistent, frozen transform tree
    auto tf_map_base  = tf_buffer_.lookupTransform("map",  "base_link", msg.header.stamp);
    auto tf_map_laser = tf_buffer_.lookupTransform("map",  "laser",     msg.header.stamp);
}  // unlock here — even if a lookup throws


// ── Example 3: Scoped diagnostics timer ─────────────────────────────────────
// ROS2 nodes publish processing time via diagnostic_updater.
// RAII ensures the timing result is always published, even on exception.

class ScopedDiagTimer {
public:
    explicit ScopedDiagTimer(const std::string& task,
                             diagnostic_updater::Updater& updater)
        : task_(task)
        , updater_(updater)
        , start_(std::chrono::steady_clock::now())
    {}

    ~ScopedDiagTimer() {
        auto elapsed = std::chrono::steady_clock::now() - start_;
        double ms = std::chrono::duration<double, std::milli>(elapsed).count();
        updater_.broadcast(0, task_ + " took " + std::to_string(ms) + " ms");
    }

private:
    std::string task_;
    diagnostic_updater::Updater& updater_;
    std::chrono::steady_clock::time_point start_;
};

void EkfNode::on_imu_msg(const sensor_msgs::msg::Imu& msg) {
    ScopedDiagTimer timer("imu_callback", updater_);
    ekf_.update(msg);
}  // timing result published here — always


// ── Example 4: Rosbag write guard ───────────────────────────────────────────
// Bag files must be explicitly closed to flush buffers and write the index.
// A crash or early return without close() corrupts the bag.

class ScopedBagWriter {
public:
    explicit ScopedBagWriter(const std::string& path)
        : writer_(std::make_unique<rosbag2_cpp::Writer>())
    {
        writer_->open(path);
    }

    ~ScopedBagWriter() {
        writer_->close();   // flushes buffers and writes index — must not be skipped
    }

    void write(const rclcpp::SerializedMessage& msg,
               const std::string& topic,
               rclcpp::Time stamp) {
        writer_->write(msg, topic, stamp);
    }

    ScopedBagWriter(const ScopedBagWriter&)            = delete;
    ScopedBagWriter& operator=(const ScopedBagWriter&) = delete;

private:
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};
```

The pattern is identical across all four cases — **acquire in constructor, release in destructor, delete copy**. Only the meaning of acquire/release changes: power on/off, lock/unlock, start/stop timing, open/close bag.

### Why RAII Wrappers Delete Copy

Every example above has these two lines:

```cpp
ScopedSensor(const ScopedSensor&)            = delete;
ScopedSensor& operator=(const ScopedSensor&) = delete;
```

`= delete` tells the compiler: do not generate this function, and make it a **compile error** if anyone tries to use it.

If you allowed copying a `ScopedSensor`, you'd have two objects managing the same sensor:

```cpp
ScopedSensor a(lidar);   // lidar.activate()
ScopedSensor b = a;      // copy — b also holds a reference to lidar

// both destructors run on scope exit:
// b destroyed → lidar.deactivate()
// a destroyed → lidar.deactivate() called AGAIN  ← double-deactivate, bug
```

This is the same class of bug as double-free on a raw pointer. Deleting copy turns it into a compile error instead of a runtime crash:

```cpp
ScopedSensor b = a;   // COMPILE ERROR: use of deleted function
```

**What about move?** Deleting the copy constructor also suppresses the compiler-generated move. For simple scope guards that's fine — you create one, use it, it's gone. If you needed to transfer ownership (e.g. return a guard from a function), you'd explicitly define a move constructor that nulls out the source.

**Key takeaway:** Every resource (sensor power, transform lock, timer, bag file, ROS2 publisher) should be wrapped in a class whose destructor releases it. Delete copy to prevent two wrappers managing the same resource. You get exception safety and deterministic cleanup for free.
