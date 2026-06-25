#include <iostream>
#include <memory>
#include <string>

// Exercise: implement a Localizer plugin system using a factory function.
//
// Define an abstract Localizer interface with predict(dt), update(measurement), state().
// Implement two concrete types:
//   DeadReckoning — update sets velocity, predict advances position by v*dt
//   ConstantLocalizer — update sets position directly, predict is a no-op
//
// create_localizer(name) returns a unique_ptr<Localizer> or throws on unknown names.

int main() {
    // TODO
    std::cout << "ex02_plugin_factory passed\n";
}
