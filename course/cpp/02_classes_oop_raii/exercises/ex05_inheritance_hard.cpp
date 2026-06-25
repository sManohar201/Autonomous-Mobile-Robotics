// Exercise 05 — Polymorphic sensor pipeline with factory and health monitoring
//
// Context
// -------
// Real robotics middleware (ROS2 sensor drivers, hardware abstraction layers)
// uses a plugin architecture: a generic pipeline holds a list of sensor
// objects through a common base pointer.  The pipeline doesn't know or care
// which concrete sensor is in each slot — it just calls the virtual interface.
//
// You will design and implement a complete sensor plugin system from scratch.
// The stubs below describe the required behaviour.  The class hierarchy
// structure — which classes exist, what they inherit from, what members they
// have — is YOUR design decision.
//
// ── Design questions — answer in comments before coding ──────────────────────
//
// Q1. Why must the base class destructor be virtual?  What goes wrong without
//     it when the pipeline calls delete on a base pointer?
//
// Q2. The factory returns std::unique_ptr<SensorPlugin>.  Why unique_ptr
//     rather than raw SensorPlugin*?  What would the caller have to do
//     differently with a raw pointer?
//
// Q3. run_once() checks health BEFORE calling read().  Why is it important
//     not to call read() on a sensor in FAILED state?  Give a concrete
//     example of what could go wrong in a real driver.
//
// Q4. The inner ScopedPipelineTimer is a class defined inside SensorPipeline.
//     What does this scoping mean?  Can code outside SensorPipeline use it
//     directly?  What is the inner class a good design choice here?
//
// ── Provided types (do not modify) ───────────────────────────────────────────

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>
#include <stdexcept>
#include <cmath>

enum class HealthStatus { OK, DEGRADED, FAILED };

struct Measurement {
    std::string        sensor_id;
    double             timestamp;   // seconds since epoch (use 0.0 for mock)
    std::vector<double> data;
};

// Simplified config entry — a real system would have a map of key→variant.
struct Config {
    std::string key;
    double      value;
};

static const char* health_str(HealthStatus h) {
    switch (h) {
        case HealthStatus::OK:       return "OK";
        case HealthStatus::DEGRADED: return "DEGRADED";
        case HealthStatus::FAILED:   return "FAILED";
    }
    return "UNKNOWN";
}

// ── ANSWERS ──────────────────────────────────────────────────────────────────
// A1 (virtual destructor):
//   Without a virtual destructor, deleting a derived object through a base
//   pointer calls only the base destructor. The derived members — including
//   any heap allocations — are never cleaned up. This is undefined behaviour
//   and causes memory/resource leaks. With a virtual destructor the correct
//   derived destructor is called first, then the base destructor.
//
// A2 (unique_ptr vs raw pointer):
//   unique_ptr enforces single ownership and calls delete automatically on
//   scope exit. With a raw pointer the caller must remember to call delete
//   (or use a smart pointer manually). unique_ptr also prevents accidental
//   copies (deleted copy constructor) making the ownership semantics explicit.
//
// A3 (health check before read):
//   A FAILED sensor may have an invalid device handle (null fd, disconnected
//   USB, etc.). Calling read() on it would dereference a null pointer or block
//   indefinitely waiting for I/O that will never arrive. In a real LiDAR
//   driver, read() might call ioctl() on a closed file descriptor — SIGSEGV
//   or EBADF, hanging the entire pipeline iteration.
//
// A4 (inner class scoping):
//   ScopedPipelineTimer is scoped to SensorPipeline — external code cannot
//   name the type without the outer class prefix (SensorPipeline::ScopedPipelineTimer).
//   If declared private, external code cannot use it at all. It is a good
//   design choice because the timer is an implementation detail of run_once();
//   exposing it globally would pollute the namespace and imply it is part of
//   the public API.

// ── SensorPlugin (abstract base) ─────────────────────────────────────────────

class MockLidar;  // forward declaration so factory can name it

class SensorPlugin {
public:
    virtual ~SensorPlugin() = default;

    virtual Measurement  read()                     const = 0;
    virtual bool         configure(const Config& cfg)     = 0;
    virtual HealthStatus check_health()             const = 0;
    virtual std::string  id()                       const = 0;

    static std::unique_ptr<SensorPlugin> create(const std::string& type);
};


// ── MockLidar ─────────────────────────────────────────────────────────────────

class MockLidar : public SensorPlugin {
public:
    MockLidar() : max_range_(10.0), failed_(false) {}

    bool configure(const Config& cfg) override {
        if (cfg.key == "max_range") {
            if (cfg.value < 0.0) return false;
            max_range_ = cfg.value;
            return true;
        }
        return false;
    }

    Measurement read() const override {
        Measurement m;
        m.sensor_id = id();
        m.timestamp = 0.0;
        m.data.assign(360, max_range_ * 0.5);
        return m;
    }

    HealthStatus check_health() const override {
        if (max_range_ <= 0.0) return HealthStatus::FAILED;
        if (failed_)           return HealthStatus::DEGRADED;
        return HealthStatus::OK;
    }

    std::string id() const override { return "lidar"; }

    void inject_failure() { failed_ = true; }

private:
    double max_range_;
    bool   failed_;
};


// ── MockIMU ──────────────────────────────────────────────────────────────────

class MockIMU : public SensorPlugin {
public:
    MockIMU() : noise_std_(0.01) {}

    bool configure(const Config& cfg) override {
        if (cfg.key == "noise_std") {
            noise_std_ = cfg.value;
        }
        return true;
    }

    Measurement read() const override {
        Measurement m;
        m.sensor_id = id();
        m.timestamp = 0.0;
        m.data.assign(6, noise_std_);
        return m;
    }

    HealthStatus check_health() const override {
        if (noise_std_ > 1.0) return HealthStatus::DEGRADED;
        return HealthStatus::OK;
    }

    std::string id() const override { return "imu"; }

private:
    double noise_std_;
};


// ── MockGPS ──────────────────────────────────────────────────────────────────

class MockGPS : public SensorPlugin {
public:
    MockGPS() : fix_quality_(0) {}

    bool configure(const Config& cfg) override {
        if (cfg.key == "fix_quality") {
            fix_quality_ = static_cast<int>(cfg.value);
        }
        return true;
    }

    Measurement read() const override {
        Measurement m;
        m.sensor_id = id();
        m.timestamp = 0.0;
        m.data = {0.0, 0.0, fix_quality_ * 10.0};
        return m;
    }

    HealthStatus check_health() const override {
        if (fix_quality_ == 0) return HealthStatus::FAILED;
        if (fix_quality_ == 1) return HealthStatus::DEGRADED;
        return HealthStatus::OK;
    }

    std::string id() const override { return "gps"; }

private:
    int fix_quality_;
};


// ── SensorPlugin factory body ─────────────────────────────────────────────────

std::unique_ptr<SensorPlugin> SensorPlugin::create(const std::string& type) {
    if (type == "lidar") return std::make_unique<MockLidar>();
    if (type == "imu")   return std::make_unique<MockIMU>();
    if (type == "gps")   return std::make_unique<MockGPS>();
    return nullptr;
}


// ── SensorPipeline ────────────────────────────────────────────────────────────

class SensorPipeline {
public:
    void register_sensor(std::unique_ptr<SensorPlugin> sensor) {
        sensors_.push_back(std::move(sensor));
    }

    std::vector<Measurement> run_once() {
        ScopedPipelineTimer timer("run_once");
        std::vector<Measurement> results;

        for (const auto& sensor : sensors_) {
            HealthStatus h = sensor->check_health();
            if (h == HealthStatus::OK) {
                results.push_back(sensor->read());
            } else if (h == HealthStatus::DEGRADED) {
                std::cout << "[pipeline] skipping degraded sensor: " << sensor->id() << "\n";
            } else {
                std::cout << "[pipeline] skipping failed sensor: " << sensor->id() << "\n";
            }
        }
        return results;
    }

    std::map<std::string, HealthStatus> diagnose() {
        std::map<std::string, HealthStatus> health;
        for (const auto& sensor : sensors_) {
            health[sensor->id()] = sensor->check_health();
        }
        return health;
    }

private:
    std::vector<std::unique_ptr<SensorPlugin>> sensors_;

    class ScopedPipelineTimer {
    public:
        explicit ScopedPipelineTimer(const std::string& label)
            : label_(label), start_(std::chrono::steady_clock::now())
        {}

        ~ScopedPipelineTimer() {
            auto end = std::chrono::steady_clock::now();
            auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                           end - start_).count();
            std::cout << "[pipeline] " << label_ << " completed in " << ms << " ms\n";
        }

        ScopedPipelineTimer(const ScopedPipelineTimer&)            = delete;
        ScopedPipelineTimer& operator=(const ScopedPipelineTimer&) = delete;

    private:
        std::string label_;
        std::chrono::steady_clock::time_point start_;
    };
};


// ── main ─────────────────────────────────────────────────────────────────────
int main() {
    SensorPipeline pipeline;

    // ── Build pipeline using factory ─────────────────────────────────────────
    auto lidar = SensorPlugin::create("lidar");
    auto imu   = SensorPlugin::create("imu");
    auto gps   = SensorPlugin::create("gps");

    if (!lidar || !imu || !gps) {
        std::cerr << "Factory failed\n";
        return 1;
    }

    // ── Configure sensors ────────────────────────────────────────────────────
    lidar->configure({"max_range", 20.0});
    imu->configure({"noise_std", 0.05});
    gps->configure({"fix_quality", 3.0});

    // Test: configure with invalid value
    bool ok = lidar->configure({"max_range", -5.0});
    std::cout << "configure(-5.0) returned: " << ok << " (expected 0)\n";
    // Reset to valid
    lidar->configure({"max_range", 20.0});

    pipeline.register_sensor(std::move(lidar));
    pipeline.register_sensor(std::move(imu));
    pipeline.register_sensor(std::move(gps));

    // Unknown type should return nullptr
    auto unknown = SensorPlugin::create("radar");
    std::cout << "unknown sensor factory: " << (unknown == nullptr ? "nullptr" : "non-null") << "\n";

    // ── First run — all sensors OK ────────────────────────────────────────────
    std::cout << "\n--- run 1 ---\n";
    auto results = pipeline.run_once();
    for (const auto& m : results) {
        std::cout << "  " << m.sensor_id << ": " << m.data.size() << " values\n";
    }

    // ── Diagnose ─────────────────────────────────────────────────────────────
    std::cout << "\n--- diagnose ---\n";
    auto health = pipeline.diagnose();
    for (const auto& [id, status] : health) {
        std::cout << "  " << id << ": " << health_str(status) << "\n";
    }

    // ── Inject lidar failure, diagnose again ─────────────────────────────────
    // We need a reference to the lidar — but we moved it into the pipeline.
    // Access it through the pipeline's diagnose() result is indirect.
    // To inject failure we need a non-const pointer.  In real code the pipeline
    // might expose a find_sensor(id) method.
    //
    // For this exercise: re-create a separate lidar just to show the health
    // transition, then inject failure through the pipeline's known second sensor
    // by casting.  NOTE: downcasting with dynamic_cast is safe here since we
    // know the type.  But for this exercise, implement inject_failure_on_sensor
    // by calling it before registering, using a raw observer pointer:

    // --- Re-create to demonstrate failure injection ---
    SensorPipeline pipeline2;
    auto lidar2 = SensorPlugin::create("lidar");
    lidar2->configure({"max_range", 15.0});

    // Obtain a raw observing pointer BEFORE moving into pipeline
    // (raw observer — pipeline2 owns the object via unique_ptr)
    auto* lidar2_ptr = dynamic_cast<MockLidar*>(lidar2.get());
    pipeline2.register_sensor(std::move(lidar2));
    pipeline2.register_sensor(SensorPlugin::create("imu"));

    std::cout << "\n--- pipeline2 run before failure ---\n";
    auto r2a = pipeline2.run_once();
    for (const auto& m : r2a) {
        std::cout << "  " << m.sensor_id << ": " << m.data.size() << " values\n";
    }

    // Inject failure
    if (lidar2_ptr) lidar2_ptr->inject_failure();

    std::cout << "\n--- pipeline2 diagnose after inject_failure ---\n";
    auto h2 = pipeline2.diagnose();
    for (const auto& [id, status] : h2) {
        std::cout << "  " << id << ": " << health_str(status) << "\n";
    }

    std::cout << "\n--- pipeline2 run after failure (lidar skipped) ---\n";
    auto r2b = pipeline2.run_once();
    for (const auto& m : r2b) {
        std::cout << "  " << m.sensor_id << ": " << m.data.size() << " values\n";
    }
    std::cout << "measurements returned: " << r2b.size() << " (expected 1 — imu only)\n";

    return 0;
}
