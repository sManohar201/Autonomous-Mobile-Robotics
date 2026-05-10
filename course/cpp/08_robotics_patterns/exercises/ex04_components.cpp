#include <iostream>
#include <optional>
#include <unordered_map>

using Entity = int;

struct Pose2d {
    double x, y, theta;
};

struct Velocity {
    double vx, vy;
};

// TODO: implement World with add_entity, set_pose, set_velocity,
// pose(entity), velocity(entity), and integrate(dt).

int main() {
    std::cout << "ex04_components passed\n";
}

