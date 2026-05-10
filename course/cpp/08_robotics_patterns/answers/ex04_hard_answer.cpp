#include <cassert>
#include <iostream>
#include <optional>
#include <unordered_map>

using Entity = int;
struct Pose { double x, y; };
struct Velocity { double vx, vy; };

class World {
public:
    Entity create() { return next_++; }
    void destroy(Entity e) { poses_.erase(e); velocities_.erase(e); }
    void set_pose(Entity e, Pose p) { poses_[e] = p; }
    void set_velocity(Entity e, Velocity v) { velocities_[e] = v; }
    std::optional<Pose> pose(Entity e) const {
        auto it = poses_.find(e);
        if (it == poses_.end()) return std::nullopt;
        return it->second;
    }
    void integrate(double dt) {
        for (auto& [e, p] : poses_) {
            auto it = velocities_.find(e);
            if (it != velocities_.end()) {
                p.x += it->second.vx * dt;
                p.y += it->second.vy * dt;
            }
        }
    }
private:
    Entity next_{1};
    std::unordered_map<Entity, Pose> poses_;
    std::unordered_map<Entity, Velocity> velocities_;
};

int main() {
    World w;
    auto e = w.create();
    w.set_pose(e, {0, 0});
    w.set_velocity(e, {2, 3});
    w.integrate(2);
    assert(w.pose(e)->x == 4);
    w.destroy(e);
    assert(!w.pose(e));
    std::cout << "ex04_hard_answer passed\n";
}

