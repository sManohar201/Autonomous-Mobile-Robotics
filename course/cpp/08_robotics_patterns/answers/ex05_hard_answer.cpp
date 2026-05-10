#include <cassert>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

struct State1d { double x; double v; };

class Localizer1d {
public:
    using Observer = std::function<void(const State1d&)>;
    explicit Localizer1d(State1d initial) : state_{initial} { history_.push_back(state_); }
    void observe(Observer observer) { observers_.push_back(std::move(observer)); }
    void predict(double dt) { state_.x += state_.v * dt; record(); }
    void update(double z) { state_.x = 0.5 * state_.x + 0.5 * z; record(); }
    void reset(State1d state) { state_ = state; record(); }
    State1d state() const { return state_; }
    const std::vector<State1d>& history() const { return history_; }
private:
    void record() {
        history_.push_back(state_);
        for (const auto& observer : observers_) observer(state_);
    }
    State1d state_;
    std::vector<State1d> history_;
    std::vector<Observer> observers_;
};

int main() {
    Localizer1d loc{{0, 2}};
    int notifications = 0;
    loc.observe([&](const State1d&) { ++notifications; });
    loc.predict(2);
    loc.update(10);
    assert(loc.state().x == 7);
    assert(loc.history().size() == 3);
    assert(notifications == 2);
    loc.reset({0, 0});
    assert(loc.state().x == 0);
    std::cout << "ex05_hard_answer passed\n";
}
