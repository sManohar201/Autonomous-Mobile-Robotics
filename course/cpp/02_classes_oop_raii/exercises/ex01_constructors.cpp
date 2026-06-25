// Exercise 01 — Constructors, Destructors & Member Initializer Lists
//
// EXPECTED OUTPUT:
//   Sensor created: lidar (id=1)
//   Sensor created: imu (id=2)
//   Sensor created: camera (id=3)
//   copy of: camera (id=3)
//   Active sensors: 4
//   Sensor destroyed: copy of: camera (id=3)
//   Sensor destroyed: camera (id=3)
//   Active sensors: 2
//   Sensor destroyed: imu (id=2)
//   Sensor destroyed: lidar (id=1)
//   Active sensors: 0

#include <iostream>
#include <string>

class Sensor {
public:
    static int instance_count_;

    Sensor(int id, std::string name) : id_(id), name_(std::move(name)) {
        ++instance_count_;
        std::cout << "Sensor created: " << name_ << " (id=" << id_ << ")\n";
    }

    ~Sensor() {
        std::cout << "Sensor destroyed: " << name_ << " (id=" << id_ << ")\n";
        --instance_count_;
    }

    explicit Sensor(int id) : Sensor(id, "unknown") {}

    Sensor(std::string name) : Sensor(0, std::move(name)) {}

    Sensor(const Sensor& other)
        : id_(other.id_), name_("copy of: " + other.name_)
    {
        ++instance_count_;
        std::cout << "Sensor created: " << name_ << " (id=" << id_ << ")\n";
    }

    static int active_count() { return instance_count_; }

private:
    int         id_;
    std::string name_;
};

int Sensor::instance_count_ = 0;


int main() {
    {
        Sensor s1(1, "lidar");
        Sensor s2(2, "imu");

        {
            Sensor s3(3, "camera");
            Sensor s4 = s3;
            std::cout << "Active sensors: " << Sensor::active_count() << "\n";
        }

        std::cout << "Active sensors: " << Sensor::active_count() << "\n";
    }

    std::cout << "Active sensors: " << Sensor::active_count() << "\n";
    return 0;
}
