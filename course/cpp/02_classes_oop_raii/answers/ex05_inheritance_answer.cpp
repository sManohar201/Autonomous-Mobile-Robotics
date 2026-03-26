// Exercise 05 — Inheritance & Virtual Functions
// ANSWER FILE
//
// NOTE ON EXPECTED OUTPUT:
// The exercise comment lists only 3 "SensorBase destroyed" lines, but the
// correct output is 6: the three unique_ptrs in `sensors` are destroyed first
// (reverse insertion order: camera, imu, lidar), then the three stack objects
// declared in main (reverse declaration order: cam, imu, lidar).  All six
// destructions go through SensorBase::~SensorBase() and print.
// The abbreviated expected output in the exercise is incomplete.

#include <iostream>
#include <vector>
#include <memory>
#include <string>

// ── Base class ───────────────────────────────────────────────────────────────

class SensorBase {
public:
    explicit SensorBase(std::string name) : name_(std::move(name)) {}

    // Pure virtual read — derived classes must override
    virtual void read() const = 0;

    // Virtual destructor: ensures correct cleanup through base pointer
    virtual ~SensorBase() {
        std::cout << "SensorBase destroyed: " << name_ << "\n";
    }

    std::string name() const { return name_; }

protected:
    std::string name_;
};


// ── Derived: Lidar ───────────────────────────────────────────────────────────

class Lidar : public SensorBase {
public:
    explicit Lidar(double range)
        : SensorBase("lidar"), range_(range)
    {}

    void read() const override {
        std::cout << "[Lidar]  reading: range=" << range_ << " m\n";
    }

private:
    double range_;
};


// ── Derived: IMU ─────────────────────────────────────────────────────────────

class IMU : public SensorBase {
public:
    IMU(double ax, double ay, double az)
        : SensorBase("imu"), ax_(ax), ay_(ay), az_(az)
    {}

    void read() const override {
        std::cout << "[IMU]    reading: ax=" << ax_
                  << " ay=" << ay_
                  << " az=" << az_ << "\n";
    }

private:
    double ax_, ay_, az_;
};


// ── Derived: Camera ──────────────────────────────────────────────────────────

class Camera : public SensorBase {
public:
    Camera(int w, int h)
        : SensorBase("camera"), width_(w), height_(h)
    {}

    void read() const override {
        std::cout << "[Camera] reading: " << width_ << "x" << height_ << " frame\n";
    }

private:
    int width_, height_;
};


int main() {
    // Direct calls
    Lidar  lidar(3.5);
    IMU    imu(0.1, -0.2, 9.8);
    Camera cam(640, 480);

    lidar.read();
    imu.read();
    cam.read();

    // Polymorphic dispatch through base pointer
    std::cout << "--- polymorphic dispatch ---\n";
    std::vector<std::unique_ptr<SensorBase>> sensors;
    sensors.push_back(std::make_unique<Lidar>(3.5));
    sensors.push_back(std::make_unique<IMU>(0.1, -0.2, 9.8));
    sensors.push_back(std::make_unique<Camera>(640, 480));

    for (const auto& s : sensors) {
        s->read();
    }
    // destructors called in reverse order as vector is destroyed
    return 0;
}
