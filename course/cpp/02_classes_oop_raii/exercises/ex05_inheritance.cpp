// Exercise 05 — Inheritance & Virtual Functions

#include <iostream>
#include <vector>
#include <memory>
#include <string>

class SensorBase {
public:
    explicit SensorBase(std::string name) : name_(std::move(name)) {}

    virtual void read() const = 0;

    virtual ~SensorBase() {
        std::cout << "SensorBase destroyed: " << name_ << "\n";
    }

    std::string name() const { return name_; }

protected:
    std::string name_;
};

class Lidar : public SensorBase {
public:
    explicit Lidar(double range) : SensorBase("lidar"), range_(range) {}

    void read() const override {
        std::cout << "[Lidar]  reading: range=" << range_ << " m\n";
    }

private:
    double range_;
};

class IMU : public SensorBase {
public:
    IMU(double ax, double ay, double az)
        : SensorBase("imu"), ax_(ax), ay_(ay), az_(az) {}

    void read() const override {
        std::cout << "[IMU]    reading: ax=" << ax_
                  << " ay=" << ay_ << " az=" << az_ << "\n";
    }

private:
    double ax_, ay_, az_;
};

class Camera : public SensorBase {
public:
    Camera(int w, int h) : SensorBase("camera"), width_(w), height_(h) {}

    void read() const override {
        std::cout << "[Camera] reading: " << width_ << "x" << height_ << " frame\n";
    }

private:
    int width_, height_;
};

int main() {
    Lidar  lidar(3.5);
    IMU    imu(0.1, -0.2, 9.8);
    Camera cam(640, 480);

    lidar.read();
    imu.read();
    cam.read();

    std::cout << "--- polymorphic dispatch ---\n";
    std::vector<std::unique_ptr<SensorBase>> sensors;
    sensors.push_back(std::make_unique<Lidar>(3.5));
    sensors.push_back(std::make_unique<IMU>(0.1, -0.2, 9.8));
    sensors.push_back(std::make_unique<Camera>(640, 480));

    for (const auto& s : sensors) {
        s->read();
    }
    return 0;
}
