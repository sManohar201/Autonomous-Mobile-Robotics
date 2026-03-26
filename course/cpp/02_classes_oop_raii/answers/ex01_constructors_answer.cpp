// Exercise 01 — Constructors, Destructors & Member Initializer Lists
// ANSWER FILE

#include <iostream>
#include <string>

class Sensor {
public:
    static int count_;  // declaration

    // Primary constructor: (int id, std::string name)
    Sensor(int id, std::string name)
        : id_(id), name_(std::move(name))
    {
        ++count_;
        std::cout << "Sensor created: " << name_ << " (id=" << id_ << ")\n";
    }

    // Destructor
    ~Sensor() {
        std::cout << "Sensor destroyed: " << name_ << " (id=" << id_ << ")\n";
        --count_;
    }

    // Explicit single-int constructor with default name
    explicit Sensor(int id)
        : Sensor(id, "unknown")
    {}

    // Delegating constructor: string-only → delegates to (0, name)
    explicit Sensor(std::string name)
        : Sensor(0, std::move(name))
    {}

    // Copy constructor: prefix name with "copy of: "
    Sensor(const Sensor& other)
        : Sensor(other.id_, "copy of: " + other.name_)
    {}

    static int active_count() { return count_; }

private:
    int         id_;
    std::string name_;
};

// Definition of static member
int Sensor::count_ = 0;


int main() {
    {
        Sensor s1(1, "lidar");
        Sensor s2(2, "imu");

        {
            Sensor s3(3, "camera");
            Sensor s4 = s3;   // copy constructor
            std::cout << "Active sensors: " << Sensor::active_count() << "\n";
        } // s3 and s4 destroyed here

        std::cout << "Active sensors: " << Sensor::active_count() << "\n";
    } // s1 and s2 destroyed here

    std::cout << "Active sensors: " << Sensor::active_count() << "\n";
    return 0;
}
