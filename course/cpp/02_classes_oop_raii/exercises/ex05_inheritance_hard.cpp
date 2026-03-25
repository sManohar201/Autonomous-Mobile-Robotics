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
//     directly?  Why is an inner class a good design choice here?
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
//   TODO
//
// A2 (unique_ptr vs raw pointer):
//   TODO
//
// A3 (health check before read):
//   TODO
//
// A4 (inner class scoping):
//   TODO

// ── SensorPlugin (abstract base) ─────────────────────────────────────────────
//
// TODO: Define the abstract base class with:
//   - pure virtual Measurement read() const = 0
//   - pure virtual bool configure(const Config& cfg) = 0
//   - pure virtual HealthStatus check_health() const = 0
//   - pure virtual std::string id() const = 0
//   - virtual destructor (must be virtual — answer Q1 above)
//   - static factory:
//       static std::unique_ptr<SensorPlugin> create(const std::string& type)
//       Supported types: "lidar", "imu", "gps"
//       Unknown type: return nullptr.
//       THINK: the factory must be defined AFTER the concrete classes so it
//       can name them.  Forward-declare the class, define concrete classes,
//       then define the factory body outside the class, OR define the factory
//       body after all concrete classes.


// ── MockLidar ─────────────────────────────────────────────────────────────────
//
// TODO: Implement MockLidar inheriting from SensorPlugin.
//
// Members:
//   double max_range_     (default 10.0)
//   bool   failed_        (default false) — set by inject_failure()
//
// configure(cfg):
//   If cfg.key == "max_range": store cfg.value as max_range_.
//     Return false if cfg.value < 0 (invalid range).
//   For any other key: return false (unrecognised).
//   Return true on success.
//
// read():
//   Returns a Measurement with:
//     sensor_id = id()
//     timestamp = 0.0
//     data      = 360 values, all equal to max_range_ * 0.5
//   THINK: a real LiDAR read() on a FAILED sensor might dereference a null
//   device handle.  In production code the pipeline guards this.
//
// check_health():
//   FAILED   if max_range_ <= 0
//   DEGRADED if failed_ == true
//   OK       otherwise
//   NOTE: FAILED takes priority over DEGRADED — check max_range_ first.
//
// id(): returns "lidar"
//
// inject_failure(): sets failed_ = true
//
// TRICKY: what is the right order to check health conditions?  If max_range_
// is -1 AND failed_ is true, should the result be FAILED or DEGRADED?
// The answer depends on which condition is more severe and checked first.


// ── MockIMU ──────────────────────────────────────────────────────────────────
//
// TODO: Implement MockIMU inheriting from SensorPlugin.
//
// Members:
//   double noise_std_   (default 0.01)
//
// configure(cfg):
//   If cfg.key == "noise_std": store cfg.value as noise_std_.
//   Return true.
//
// read():
//   Returns Measurement with 6 values: [ax, ay, az, gx, gy, gz]
//   All values = noise_std_  (a trivial mock)
//
// check_health():
//   noise_std_ > 1.0 → DEGRADED  (unrealistically high noise)
//   otherwise        → OK
//
// id(): returns "imu"


// ── MockGPS ──────────────────────────────────────────────────────────────────
//
// TODO: Implement MockGPS inheriting from SensorPlugin.
//
// Members:
//   int fix_quality_    (default 0)
//
// configure(cfg):
//   If cfg.key == "fix_quality": fix_quality_ = static_cast<int>(cfg.value)
//   Return true.
//
// read():
//   Returns Measurement with 3 values: [lat, lon, alt]
//   lat = 0.0, lon = 0.0, alt = fix_quality_ * 10.0
//
// check_health():
//   fix_quality_ == 0 → FAILED
//   fix_quality_ == 1 → DEGRADED
//   fix_quality_ >= 2 → OK
//
// id(): returns "gps"


// ── SensorPlugin factory body ─────────────────────────────────────────────────
// TODO: Implement SensorPlugin::create() here (after MockLidar/IMU/GPS are
// defined so the constructor calls resolve).
// Return the right concrete type as unique_ptr<SensorPlugin>.
// Return nullptr for unknown type strings.


// ── SensorPipeline ────────────────────────────────────────────────────────────
//
// TODO: Implement SensorPipeline with the following interface.
//
// Inner class ScopedPipelineTimer:
//   - Defined inside SensorPipeline (private or public — your choice, but
//     think about encapsulation).
//   - Constructor: records start time and a label.
//   - Destructor: prints "[pipeline] <label> completed in <ms> ms"
//   - Non-copyable.
//   Usage in run_once(): instantiate one at the top of the function body.
//
// register_sensor(std::unique_ptr<SensorPlugin> sensor):
//   Takes ownership.  Stores in a std::vector<std::unique_ptr<SensorPlugin>>.
//
// run_once():
//   Create a ScopedPipelineTimer at the start of the function.
//   For each sensor:
//     Check check_health().
//     If OK:       call read(), append measurement to results.
//     If DEGRADED: print "[pipeline] skipping degraded sensor: <id>\n"
//     If FAILED:   print "[pipeline] skipping failed sensor: <id>\n"
//   Return vector<Measurement>.
//   CRITICAL: call check_health() FIRST.  Never call read() on a FAILED sensor.
//
// diagnose():
//   Returns std::map<std::string, HealthStatus>
//   Iterates sensors, maps sensor_id → check_health().
//
// THINK: run_once() returns by value — this means the result vector is
// potentially moved on return (NRVO / move semantics).  With C++17 guaranteed
// copy elision, no copy occurs.  Just return the local vector normally.

class SensorPipeline {
public:
    // TODO: declare and implement all members described above.
    //
    // Skeleton to expand:
    void register_sensor(std::unique_ptr<SensorPlugin> sensor);

    std::vector<Measurement> run_once();

    std::map<std::string, HealthStatus> diagnose();

private:
    std::vector<std::unique_ptr<SensorPlugin>> sensors_;

    // TODO: define inner class ScopedPipelineTimer here.
    //   It may access SensorPipeline's private members if needed (inner classes
    //   have access to the enclosing class's privates in C++).
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
