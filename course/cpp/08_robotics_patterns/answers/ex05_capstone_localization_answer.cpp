#include <cassert>
#include <iostream>
#include <vector>

struct State1d {
    double x;
    double v;
};

class MotionModel {
public:
    State1d predict(State1d state, double dt) const {
        state.x += state.v * dt;
        return state;
    }
};

class MeasurementModel {
public:
    explicit MeasurementModel(double gain) : gain_{gain} {}

    State1d update(State1d state, double position) const {
        state.x += gain_ * (position - state.x);
        return state;
    }

private:
    double gain_;
};

class Localizer1d {
public:
    Localizer1d(State1d initial, double measurement_gain)
        : state_{initial}, measurement_{measurement_gain}
    {
        history_.push_back(state_);
    }

    void predict(double dt) {
        state_ = motion_.predict(state_, dt);
        history_.push_back(state_);
    }

    void update(double position) {
        state_ = measurement_.update(state_, position);
        history_.push_back(state_);
    }

    State1d state() const { return state_; }
    const std::vector<State1d>& history() const { return history_; }

private:
    State1d state_;
    MotionModel motion_;
    MeasurementModel measurement_;
    std::vector<State1d> history_;
};

int main() {
    Localizer1d loc{{0.0, 2.0}, 0.5};
    loc.predict(2.0);
    assert(loc.state().x == 4.0);
    loc.update(10.0);
    assert(loc.state().x == 7.0);
    assert(loc.history().size() == 3);
    std::cout << "ex05_capstone_localization_answer passed\n";
}

