#include <iostream>
#include <optional>
#include <unordered_map>

using Entity = int;
struct Pose2d    { double x, y, theta; };
struct Velocity  { double vx, vy; };

// Exercise: implement a World entity-component store.
//
// add_entity()             — returns a new unique Entity ID (auto-increment)
// set_pose(e, Pose2d)      — attach a pose component to entity e
// set_velocity(e, Velocity) — attach a velocity component
// pose(e)                  — return optional<Pose2d>, nullopt if not set
// velocity(e)              — return optional<Velocity>
// integrate(dt)            — for each entity with both pose and velocity,
//                            advance x += vx*dt, y += vy*dt
//
// Hint: two separate unordered_maps are the simplest data structure here.

int main() {
    // TODO
    std::cout << "ex04_components passed\n";
}
