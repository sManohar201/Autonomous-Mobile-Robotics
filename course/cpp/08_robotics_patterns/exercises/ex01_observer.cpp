#include <functional>
#include <iostream>
#include <vector>

struct ImuSample {
    double ax, ay, az;
};

// TODO: implement SensorBus with subscribe(callback) and publish(sample).
// Do not expose the internal callback vector.

int main() {
    std::cout << "ex01_observer passed\n";
}

