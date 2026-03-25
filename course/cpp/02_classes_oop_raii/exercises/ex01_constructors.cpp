// Exercise 01 — Constructors, Destructors & Member Initializer Lists
//
// TASK:
//   Implement the Sensor class below according to the specifications.
//   Do not change any function signatures or the main() function.
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
    // TODO 1: Declare a static int member to count active Sensor instances.
    //         It must be declared here and defined (initialised to 0) below
    //         the class definition.

    // TODO 2: Parameterised constructor — takes (int id, std::string name).
    //         Use a member initializer list.
    //         Print "Sensor created: <name> (id=<id>)\n".
    //         Increment the instance counter.

    // TODO 3: Destructor.
    //         Print "Sensor destroyed: <name> (id=<id>)\n".
    //         Decrement the instance counter.

    // TODO 4: Make the constructor below explicit so the compiler rejects:
    //             Sensor s = 42;
    //         It takes a single int id, name defaults to "unknown".
    //   Sensor(int id);

    // TODO 5: Delegating constructor.
    //         Sensor(std::string name) should delegate to Sensor(0, name).

    // TODO 6: Copy constructor.
    //         Prefix the copied name with "copy of: ".
    //         Increment the counter and print the creation message.

    static int active_count();   // returns the current instance count

private:
    int         id_;
    std::string name_;
};

// TODO: define the static member here (one line, outside the class)


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
