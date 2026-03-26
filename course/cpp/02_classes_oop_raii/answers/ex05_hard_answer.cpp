// Exercise 05 (Hard) — Polymorphic sensor pipeline with factory and health monitoring
// ANSWER FILE
//
// ── Design question answers ──────────────────────────────────────────────────
//
// A1 (virtual destructor):
//   Without a virtual destructor, deleting a derived object through a base
//   pointer calls ONLY the base class destructor — the derived destructor is
//   never invoked.  This is undefined behaviour per the C++ standard and in
//   practice causes resource leaks: any RAII members in the derived class
//   (open file handles, heap buffers, device registrations) are never freed.
//   With "virtual ~SensorPlugin() = default;", the vtable dispatch ensures the
//   correct derived destructor runs first, then the base destructor, in the
//   correct order.  Rule of thumb: if a class has any virtual function, give it
//   a virtual destructor.
//
// A2 (unique_ptr vs raw pointer):
//   std::unique_ptr<SensorPlugin> encodes single ownership in the type: the
//   pointer owns the object and will delete it automatically when it goes out
//   of scope.  The caller never needs to call delete, and accidental double-
//   delete is impossible.  With a raw SensorPlugin* the caller must:
//     (a) call delete explicitly (easy to forget on every exit path),
//     (b) ensure they don't double-delete (e.g. after a copy of the pointer),
//     (c) know the correct virtual destructor is called — which requires Q1
//         to be handled correctly.
//   unique_ptr also communicates intent: a factory that returns unique_ptr is
//   unambiguously handing over ownership.  A raw pointer says nothing about
//   whether the caller should free it.
//
// A3 (health check before read):
//   A FAILED sensor may have a null device file descriptor, a closed serial
//   port, or unresponsive hardware.  Concrete failure modes if read() is called
//   on a FAILED sensor:
//     - A null device handle is dereferenced → segfault / crash.
//     - The blocking read() call to the hardware never returns → pipeline hangs.
//     - Stale or zeroed memory is returned without error → corrupted
//       measurements flow silently into the EKF and produce a wrong state
//       estimate with no visible error signal.
//   The health check is the gate that prevents all three outcomes.  It is cheap
//   (a flag check), while the downstream consequences of skipping it are severe.
//
// A4 (inner class scoping):
//   ScopedPipelineTimer is defined inside SensorPipeline, so its full type name
//   is SensorPipeline::ScopedPipelineTimer.  Code outside SensorPipeline cannot
//   name or instantiate it without that qualifier (and since it is in the
//   private section, external code cannot use it at all).  This is good
//   encapsulation: the timer is an implementation detail of run_once(), not part
//   of the public API.  If the timing strategy changes (e.g. swap steady_clock
//   for a mock clock in tests), no callsites outside the class need to change.
//   Inner classes are a natural way to bundle a helper type tightly with the
//   class that owns its lifecycle.

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>
#include <stdexcept>
#include <cmath>

// ── Provided types (unchanged from exercise) ──────────────────────────────────

enum class HealthStatus { OK, DEGRADED, FAILED };

struct Measurement {
    std::string         sensor_id;
    double              timestamp;   // seconds since epoch (0.0 for mocks)
    std::vector<double> data;
};

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

// ── SensorPlugin — abstract base ─────────────────────────────────────────────
//
// The factory (create) is declared here but defined after the concrete classes
// so that it can name MockLidar, MockIMU, and MockGPS.  Separating declaration
// from definition is the standard pattern for factories in a single translation
// unit.

class SensorPlugin {
public:
    virtual ~SensorPlugin() = default;

    virtual Measurement  read()                      const = 0;
    virtual bool         configure(const Config& cfg)      = 0;
    virtual HealthStatus check_health()              const = 0;
    virtual std::string  id()                        const = 0;

    // Factory — implemented after concrete classes below.
    static std::unique_ptr<SensorPlugin> create(const std::string& type);
};

// ── MockLidar ─────────────────────────────────────────────────────────────────

class MockLidar : public SensorPlugin {
public:
    std::string id() const override { return "lidar"; }

    bool configure(const Config& cfg) override {
        if (cfg.key == "max_range") {
            if (cfg.value < 0.0) return false;   // negative range is invalid
            max_range_ = cfg.value;
            return true;
        }
        return false;   // unrecognised key
    }

    // 360 range readings, all at half the configured max range.
    Measurement read() const override {
        Measurement m;
        m.sensor_id = id();
        m.timestamp = 0.0;
        m.data.assign(360, max_range_ * 0.5);
        return m;
    }

    // FAILED takes priority over DEGRADED: check max_range_ first.
    HealthStatus check_health() const override {
        if (max_range_ <= 0.0) return HealthStatus::FAILED;
        if (failed_)           return HealthStatus::DEGRADED;
        return HealthStatus::OK;
    }

    // Simulates a hardware fault discovered at runtime (e.g. USB disconnect).
    void inject_failure() { failed_ = true; }

private:
    double max_range_ = 10.0;
    bool   failed_    = false;
};

// ── MockIMU ───────────────────────────────────────────────────────────────────

class MockIMU : public SensorPlugin {
public:
    std::string id() const override { return "imu"; }

    bool configure(const Config& cfg) override {
        if (cfg.key == "noise_std") {
            noise_std_ = cfg.value;
            return true;
        }
        return false;
    }

    // Six-axis: [ax, ay, az, gx, gy, gz] — all set to noise_std_ as a trivial mock.
    Measurement read() const override {
        Measurement m;
        m.sensor_id = id();
        m.timestamp = 0.0;
        m.data = {noise_std_, noise_std_, noise_std_,
                  noise_std_, noise_std_, noise_std_};
        return m;
    }

    HealthStatus check_health() const override {
        if (noise_std_ > 1.0) return HealthStatus::DEGRADED;
        return HealthStatus::OK;
    }

private:
    double noise_std_ = 0.01;
};

// ── MockGPS ───────────────────────────────────────────────────────────────────

class MockGPS : public SensorPlugin {
public:
    std::string id() const override { return "gps"; }

    bool configure(const Config& cfg) override {
        if (cfg.key == "fix_quality") {
            fix_quality_ = static_cast<int>(cfg.value);
            return true;
        }
        return false;
    }

    // Three values: [lat, lon, alt].  alt encodes fix quality for easy inspection.
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

private:
    int fix_quality_ = 0;
};

// ── SensorPlugin factory body ─────────────────────────────────────────────────
// Defined here, after the concrete classes, so that std::make_unique<MockLidar>
// etc. can resolve.  The declaration in the class body is sufficient for the
// compiler to accept calls to SensorPlugin::create() at any point in the file.

std::unique_ptr<SensorPlugin> SensorPlugin::create(const std::string& type) {
    if (type == "lidar") return std::make_unique<MockLidar>();
    if (type == "imu")   return std::make_unique<MockIMU>();
    if (type == "gps")   return std::make_unique<MockGPS>();
    return nullptr;   // unknown type
}

// ── SensorPipeline ────────────────────────────────────────────────────────────

class SensorPipeline {
public:
    // Transfer ownership of a sensor into the pipeline.
    void register_sensor(std::unique_ptr<SensorPlugin> sensor) {
        sensors_.push_back(std::move(sensor));
    }

    // Read from every healthy sensor.  Unhealthy sensors are logged and skipped.
    // A ScopedPipelineTimer is instantiated at entry; its destructor prints the
    // elapsed time when run_once() returns (normal or exceptional path).
    std::vector<Measurement> run_once() {
        ScopedPipelineTimer timer("run_once");
        std::vector<Measurement> results;
        for (auto& s : sensors_) {
            const HealthStatus h = s->check_health();
            if (h == HealthStatus::OK) {
                results.push_back(s->read());
            } else if (h == HealthStatus::DEGRADED) {
                std::cout << "[pipeline] skipping degraded sensor: " << s->id() << "\n";
            } else {
                std::cout << "[pipeline] skipping failed sensor: "   << s->id() << "\n";
            }
        }
        return results;   // NRVO / guaranteed copy elision in C++17
    }

    // Snapshot of sensor health keyed by sensor id.
    std::map<std::string, HealthStatus> diagnose() {
        std::map<std::string, HealthStatus> result;
        for (auto& s : sensors_) {
            result[s->id()] = s->check_health();
        }
        return result;
    }

private:
    std::vector<std::unique_ptr<SensorPlugin>> sensors_;

    // ── ScopedPipelineTimer (inner class) ─────────────────────────────────────
    // RAII wrapper: records a label and a start time on construction, then
    // prints the elapsed time in milliseconds when the destructor runs.
    // Non-copyable to prevent accidental double-reporting.
    //
    // Being defined in SensorPipeline's private section means:
    //   - Its full name is SensorPipeline::ScopedPipelineTimer.
    //   - External code cannot instantiate or even name it.
    //   - It has access to SensorPipeline's private members if needed.
    class ScopedPipelineTimer {
    public:
        explicit ScopedPipelineTimer(const std::string& label)
            : label_(label),
              start_(std::chrono::steady_clock::now()) {}

        ~ScopedPipelineTimer() {
            const auto end = std::chrono::steady_clock::now();
            const auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 end - start_).count();
            std::cout << "[pipeline] " << label_ << " completed in " << ms << " ms\n";
        }

        // Non-copyable: a timer should have exactly one owner.
        ScopedPipelineTimer(const ScopedPipelineTimer&)            = delete;
        ScopedPipelineTimer& operator=(const ScopedPipelineTimer&) = delete;

    private:
        std::string                            label_;
        std::chrono::steady_clock::time_point  start_;
    };
};

// ── main ─────────────────────────────────────────────────────────────────────
int main() {
    SensorPipeline pipeline;

    // Build pipeline using factory
    auto lidar = SensorPlugin::create("lidar");
    auto imu   = SensorPlugin::create("imu");
    auto gps   = SensorPlugin::create("gps");

    if (!lidar || !imu || !gps) {
        std::cerr << "Factory failed\n";
        return 1;
    }

    // Configure sensors
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

    // First run — all sensors OK
    std::cout << "\n--- run 1 ---\n";
    auto results = pipeline.run_once();
    for (const auto& m : results) {
        std::cout << "  " << m.sensor_id << ": " << m.data.size() << " values\n";
    }

    // Diagnose
    std::cout << "\n--- diagnose ---\n";
    auto health = pipeline.diagnose();
    for (const auto& [id, status] : health) {
        std::cout << "  " << id << ": " << health_str(status) << "\n";
    }

    // Demonstrate failure injection using a separate pipeline and a raw
    // observer pointer obtained BEFORE transferring ownership to the pipeline.
    SensorPipeline pipeline2;
    auto lidar2 = SensorPlugin::create("lidar");
    lidar2->configure({"max_range", 15.0});

    // Safe: pipeline2 owns the object via unique_ptr; lidar2_ptr is a non-owning
    // observer.  The pointed-to object lives as long as pipeline2 does.
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
