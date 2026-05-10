#include <iostream>
#include <vector>

// TODO: fix the out-of-bounds bug below, then run with AddressSanitizer.

int main() {
    std::vector<int> values{1, 2, 3};
    // for (std::size_t i = 0; i <= values.size(); ++i) {
    //     std::cout << values[i] << "\n";
    // }
    std::cout << "ex04_sanitizer_bughunt passed\n";
}

