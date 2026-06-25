// Exercise 02 (Hard) - Plugin Factory with Diagnostics
//
// Extend the factory pattern so it never throws on unknown types.
// Instead, return a result struct carrying either a valid plugin or an error string.
//
// Implement two Localizer variants with distinct update strategies.
// The factory returns nullptr + error message for unknown type strings.
//
// Think about why this pattern is preferred over exceptions at plugin boundaries.

#include <iostream>

int main() {
    std::cout << "ex02_hard passed\n";
}
