#include <cassert>
#include <iostream>
#include <vector>

// Exercise: the loop below has an out-of-bounds bug.
// Fix it and implement safe_sum(values) that returns the correct sum.
//
// Buggy code (do NOT copy — just observe the off-by-one):
//   for (std::size_t i = 0; i <= values.size(); ++i)  // <= reads one past end
//       sum += values[i];
//
// Compile with -fsanitize=address to catch this class of bug automatically.

int main() {
    // TODO
    std::cout << "ex04_sanitizer_bughunt passed\n";
}
