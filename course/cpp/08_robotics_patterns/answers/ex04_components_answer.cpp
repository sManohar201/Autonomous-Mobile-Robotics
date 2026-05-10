#include <cassert>
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

class World {
public:
    Entity add_entity() { return next_++; }

    void set_pose(Entity e, Pose2d pose) { poses_[e] = pose; }
    void set_velocity(Entity e, Velocity velocity) { velocities_[e] = velocity; }

    std::optional<Pose2d> pose(Entity e) const {
        auto it = poses_.find(e);
        if (it == poses_.end()) return std::nullopt;
        return it->second;
    }

    std::optional<Velocity> velocity(Entity e) const {
        auto it = velocities_.find(e);
        if (it == velocities_.end()) return std::nullopt;
        return it->second;
    }

    void integrate(double dt) {
        for (auto& [entity, pose] : poses_) {
            auto vel = velocity(entity);
            if (!vel) continue;
            pose.x += vel->vx * dt;
            pose.y += vel->vy * dt;
        }
    }

private:
    Entity next_{1};
    std::unordered_map<Entity, Pose2d> poses_;
    std::unordered_map<Entity, Velocity> velocities_;
};

int main() {
    World world;
    Entity robot = world.add_entity();
    world.set_pose(robot, {0.0, 0.0, 0.0});
    world.set_velocity(robot, {2.0, 1.0});
    world.integrate(3.0);
    auto pose = world.pose(robot);
    assert(pose);
    assert(pose->x == 6.0);
    assert(pose->y == 3.0);
    std::cout << "ex04_components_answer passed\n";
}

