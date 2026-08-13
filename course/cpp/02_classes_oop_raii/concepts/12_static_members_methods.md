# 2.11 static Members & Methods

A `static` member belongs to the **class**, not to any particular instance. All objects of the class share the same static data member.

### Static Data Members

```cpp
#include <iostream>
#include <string>
#include <atomic>

class RosNode {
public:
    explicit RosNode(std::string name) : name_{std::move(name)} {
        ++instance_count_;   // increments the shared counter
        std::cout << "Node " << name_ << " created (total=" << instance_count_ << ")\n";
    }

    ~RosNode() {
        --instance_count_;
        std::cout << "Node " << name_ << " destroyed (total=" << instance_count_ << ")\n";
    }

    // Static member function: no 'this' pointer, can only access static members.
    static int instance_count() { return instance_count_; }

    // Static constant — can be defined inline (integral or constexpr).
    static constexpr int kMaxNodes = 16;

    std::string name() const { return name_; }

private:
    std::string name_;

    // Declaration inside the class.
    // Non-constexpr static data members must be DEFINED outside the class
    // in exactly one .cpp file (see below).
    static int instance_count_;
};

// Definition outside the class (in the .cpp file in a real project).
int RosNode::instance_count_ = 0;

int main() {
    std::cout << "count=" << RosNode::instance_count() << "\n";   // 0

    {
        RosNode n1{"camera_node"};
        RosNode n2{"lidar_node"};
        std::cout << "count=" << RosNode::instance_count() << "\n"; // 2
    }   // both destroyed

    std::cout << "count=" << RosNode::instance_count() << "\n";   // 0
    std::cout << "max="   << RosNode::kMaxNodes << "\n";          // 16
}
```

### Factory Method Pattern

Consider a `Quaternion` class. A quaternion has four components `(w, x, y, z)` but nobody thinks in those terms — you think in axis-angle, Euler angles, or identity. If you expose only a raw four-argument constructor, callers have to know the math just to construct one:

```cpp
// What does this mean? Is it valid? Hard to tell at a glance.
Quaternion q{0.707, 0.0, 0.0, 0.707};
```

A **factory method** is a static function that creates an object with a meaningful name describing *what you're asking for*, not *how it's stored internally*:

```cpp
// Crystal clear — no math knowledge required at the call site.
auto q = Quaternion::from_axis_angle(0.0, 0.0, 1.0, M_PI / 2.0);
```

The static function does the math internally and hands you back a valid object. The constructor can even be made private so callers are forced to go through the factories — guaranteeing the invariant is always established correctly.

```cpp
#include <cmath>
#include <iostream>

class Quaternion {
public:
    double w, x, y, z;

    // Factory: no rotation — the starting point for any rotation sequence.
    static Quaternion identity() {
        return Quaternion{1.0, 0.0, 0.0, 0.0};
    }

    // Factory: rotate by angle_rad around an arbitrary unit axis (ax, ay, az).
    // Caller thinks in geometry — the math stays inside.
    static Quaternion from_axis_angle(double ax, double ay, double az,
                                      double angle_rad)
    {
        double half = angle_rad * 0.5;
        double s    = std::sin(half);
        return Quaternion{std::cos(half), ax*s, ay*s, az*s};
    }

    // Factory: build from a ROS2 geometry_msgs quaternion message.
    // Keeps message-type knowledge isolated in one place.
    static Quaternion from_ros_msg(double mx, double my, double mz, double mw) {
        return Quaternion{mw, mx, my, mz};   // ROS uses (x,y,z,w) order — easy to get wrong
    }

    void print() const {
        std::cout << "Q(w=" << w << ", x=" << x
                  << ", y=" << y << ", z=" << z << ")\n";
    }

private:
    // Private: forces all construction through the named factories above.
    // Nobody can accidentally construct a quaternion with wrong component order.
    Quaternion(double w, double x, double y, double z)
        : w{w}, x{x}, y{y}, z{z} {}
};

int main() {
    auto q1 = Quaternion::identity();
    auto q2 = Quaternion::from_axis_angle(0.0, 0.0, 1.0, M_PI / 2.0);  // 90 deg around Z
    auto q3 = Quaternion::from_ros_msg(0.0, 0.0, 0.707, 0.707);

    q1.print();   // Q(w=1, x=0, y=0, z=0)
    q2.print();   // Q(w=0.707, x=0, y=0, z=0.707)
    q3.print();   // Q(w=0.707, x=0, y=0, z=0.707)

    // Quaternion q{1, 0, 0, 0};  // COMPILE ERROR — constructor is private
}
```

This pattern appears constantly in robotics libraries — `Transform::identity()`, `Pose::from_tf_msg()`, `Covariance::diagonal(...)` — anywhere the internal representation differs from how a human naturally specifies the value.

**Key takeaway:** Use factory methods when the same type can be created from multiple different sources (axis-angle, Euler, ROS message, identity) and you want the call site to read like intent, not math. Making the constructor private enforces that all objects are created through a controlled, named path.
