// ANSWER — Exercise 01: The Compilation Pipeline
// ─────────────────────────────────────────────────────────────────────────────
// TASK 1: Error before adding the definition:
//
//   undefined reference to `compute_hypotenuse(double, double)'
//
//   This is a LINKER error, not a compiler error.
//   - Compiler errors: file + line number, syntax/type problems
//   - Linker errors: "undefined reference to `symbol'", no line number
//
//   The compiler accepted the call site (saw the declaration).
//   The linker searched all .o files and found no definition.
//
// TASK 2: Definition in this same file — compiler and linker both succeed
//   because the definition is in the same translation unit.
//
// TASK 3 (Challenge): Separate file + CMake
//   CMakeLists.txt:
//     add_library(hypotenuse hypotenuse.cpp)
//     target_link_libraries(ex01_pipeline PRIVATE hypotenuse)
//   Keep only the declaration here. Linker finds the definition in hypotenuse.o.
//
// KEY RULE: Never #include a .cpp file.
//   #include "hypotenuse.cpp" pastes the definition via the preprocessor.
//   The linker is never involved. It "works" accidentally but teaches nothing
//   and causes multiple-definition errors if the .cpp is also compiled separately.
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <cmath>

// Forward declaration — linker resolves this to hypotenuse.o
double compute_hypotenuse(double a, double b);

int main() {
    std::cout << "hypotenuse(3, 4) = "  << compute_hypotenuse(3.0, 4.0)  << "\n";
    std::cout << "hypotenuse(5, 12) = " << compute_hypotenuse(5.0, 12.0) << "\n";
    return 0;
}
