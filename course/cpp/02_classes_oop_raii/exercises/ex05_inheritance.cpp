// Exercise 05 — Inheritance & Virtual Functions
//
// TASK:
//   Build a sensor hierarchy with a common abstract base class.
//   Do not change main().
//
// EXPECTED OUTPUT:
//   [Lidar]  reading: range=3.5 m
//   [IMU]    reading: ax=0.1 ay=-0.2 az=9.8
//   [Camera] reading: 640x480 frame
//   --- polymorphic dispatch ---
//   [Lidar]  reading: range=3.5 m
//   [IMU]    reading: ax=0.1 ay=-0.2 az=9.8
//   [Camera] reading: 640x480 frame
//   SensorBase destroyed: lidar
//   SensorBase destroyed: imu
//   SensorBase destroyed: camera

#include <iostream>
#include <vector>
#include <memory>
#include <string>

// ── Base class ───────────────────────────────────────────────────────────────
// TODO 1: Declare SensorBase with:
//   - protected string member name_
//   - Constructor taking std::string name
//   - Pure virtual method: virtual void read() const = 0
//   - Virtual destructor that prints "SensorBase destroyed: <name>\n"
//   - Non-virtual method: std::string name() const  (returns name_)


// ── Derived: Lidar ───────────────────────────────────────────────────────────
// TODO 2: Lidar inherits publicly from SensorBase.
//   Additional member: double range_
//   Constructor: Lidar(double range) — calls SensorBase("lidar"), stores range_
//   Override read(): prints "[Lidar]  reading: range=<range_> m\n"
//   Use the override keyword.


// ── Derived: IMU ─────────────────────────────────────────────────────────────
// TODO 3: IMU inherits publicly from SensorBase.
//   Members: double ax_, ay_, az_
//   Constructor: IMU(double ax, double ay, double az)
//   Override read(): prints "[IMU]    reading: ax=<ax_> ay=<ay_> az=<az_>\n"


// ── Derived: Camera ──────────────────────────────────────────────────────────
// TODO 4: Camera inherits publicly from SensorBase.
//   Members: int width_, height_
//   Constructor: Camera(int w, int h)
//   Override read(): prints "[Camera] reading: <width_>x<height_> frame\n"


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
