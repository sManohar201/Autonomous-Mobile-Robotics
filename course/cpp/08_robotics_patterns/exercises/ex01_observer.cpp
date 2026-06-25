#include <functional>
#include <iostream>
#include <vector>

struct ImuSample { double ax, ay, az; };

// Exercise: implement SensorBus — an observer/event-bus for IMU samples.
//
// subscribe(callback) — stores a std::function<void(const ImuSample&)>
// publish(sample)     — calls every stored callback with the sample
//
// The internal callback storage must not be accessible from outside the class.
// Hint: std::vector<Callback> as a private member is all you need here.

int main() {
    // TODO
    std::cout << "ex01_observer passed\n";
}
