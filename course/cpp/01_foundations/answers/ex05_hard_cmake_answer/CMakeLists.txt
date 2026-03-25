# ANSWER — Exercise 05 (Hard): Production CMake Multi-Library Dependency Graph
# ─────────────────────────────────────────────────────────────────────────────

# ── Q1: PUBLIC vs PRIVATE vs INTERFACE ───────────────────────────────────────
#
# These keywords control what gets propagated to consuming targets.
# They appear in target_link_libraries and target_include_directories.
#
# PRIVATE: The dependency is used only in this target's OWN source files
#          (the .cpp files in this target). Consuming targets do NOT see it.
#          Example: a utility library used in implementation .cpp files but
#          not mentioned in any public header.
#
# PUBLIC:  The dependency is used both in this target's sources AND in its
#          public headers (which consumers include). Consuming targets
#          automatically inherit the dependency.
#          Example: sensor_utils links robot_math PUBLIC because
#          lowpass.hpp includes robot_math/vec3.hpp — any consumer of
#          sensor_utils that includes lowpass.hpp also needs robot_math.
#
# INTERFACE: The dependency is NOT used in this target's own sources at all,
#            but IS used by consumers. Used for header-only libraries:
#            an INTERFACE target has no source files but exposes include dirs
#            and compile options to consumers.
#            Example: a header-only math utility with no .cpp files.
#
# Propagation table:
#   Keyword   | Affects self | Propagates to consumers
#   PRIVATE   |    YES       |    NO
#   PUBLIC    |    YES       |    YES
#   INTERFACE |    NO        |    YES

# ── Q2: Why PRIVATE robot_math fails for sensor_utils consumers ──────────────
#
# If sensor_utils links robot_math PRIVATE:
#   - sensor_utils's own .cpp files can see robot_math's include dirs.
#   - But robot_app (which links sensor_utils) does NOT inherit those include dirs.
#   - robot_app does: #include <sensor_utils/lowpass.hpp>
#   - lowpass.hpp contains: #include <robot_math/vec3.hpp>
#   - The compiler tries to resolve <robot_math/vec3.hpp> for robot_app.
#   - robot_app's include path does NOT contain libs/robot_math/include.
#   - Error: fatal error: robot_math/vec3.hpp: No such file or directory
#   - This is a COMPILE error, not a link error.
#
# Fix: link robot_math PUBLIC from sensor_utils.

# ── Q3: CMake INTERFACE library ──────────────────────────────────────────────
#
# An INTERFACE library is a CMake target with no compiled source files.
# It exists purely to propagate properties (include directories, compile
# definitions, link libraries) to other targets.
#
# Use cases:
#   - Header-only libraries: all code is in .hpp files; there is nothing to
#     compile. add_library(my_headers INTERFACE) +
#     target_include_directories(my_headers INTERFACE include/) lets any
#     target link my_headers and automatically get the include path.
#   - Compile option bundles: a target that sets -Wall -Wextra etc. for all
#     targets in a project, without duplicating the options everywhere.
#   - Feature flags: an INTERFACE target for a platform-specific set of
#     definitions (e.g., -DUSE_NEON) that multiple targets consume.

# ── Q4: Enabling AddressSanitizer ────────────────────────────────────────────
#
# AddressSanitizer requires flags on BOTH compilation AND linking:
#   Compile: -fsanitize=address -fno-omit-frame-pointer
#   Link:    -fsanitize=address
#
# Why both? The compiler inserts shadow memory instrumentation in each .o file
# at compile time. The linker must link in the ASan runtime library (libasan)
# which provides the actual memory tracking and reporting. If you only add the
# compile flag, the linker fails to find ASan symbols. If you only add the link
# flag, the object files have no instrumentation to report errors.
#
# In CMake:
#   target_compile_options(my_target PRIVATE -fsanitize=address -fno-omit-frame-pointer)
#   target_link_options(my_target    PRIVATE -fsanitize=address)

# ── Q5: compile_commands.json ────────────────────────────────────────────────
#
# cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON generates a file
# build/compile_commands.json containing one entry per source file: the
# exact compiler command (with all flags, include paths, defines) used to
# compile that file.
#
# Tools that consume compile_commands.json:
#   - clangd: the C++ language server; provides code completion, go-to-def,
#     hover docs, diagnostics in any editor (VS Code, Neovim, Emacs, CLion).
#   - clang-tidy: static analysis linter; uses compile commands to understand
#     include paths and definitions when linting.
#   - include-what-you-use (iwyu): checks for unnecessary #includes.
#   - ccls: another C++ language server.
#
# Workflow: symlink into project root for editor pickup:
#   ln -sf build/compile_commands.json compile_commands.json

# ── Q6: cmake --install ──────────────────────────────────────────────────────
#
# cmake --install build --prefix /tmp/robot_install copies build artifacts
# to the specified prefix directory, creating a relocatable installation:
#   /tmp/robot_install/lib/     → static/shared libraries
#   /tmp/robot_install/bin/     → executables
#   /tmp/robot_install/include/ → public headers
#
# For this to work, CMakeLists.txt must contain install() commands:
#   install(TARGETS ...)       — specifies what to install and where
#   install(DIRECTORY ...)     — copies header directories
#
# The installed package can then be consumed by other CMake projects using
# find_package() (if a cmake config file is also installed) or by manually
# setting CMAKE_PREFIX_PATH.

# Key insight: sensor_utils must link robot_math with PUBLIC because
# lowpass.hpp includes robot_math/vec3.hpp — every consumer of sensor_utils
# also needs robot_math's include directories transitively.
# The correct keyword is: PUBLIC

# ─────────────────────────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.16)
project(robot_system LANGUAGES CXX)

# R1: C++17, required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # -std=c++17 not -std=gnu++17

# Generate compile_commands.json for clangd/clang-tidy:
#   cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#   ln -sf build/compile_commands.json compile_commands.json

# ── R2: robot_math static library ────────────────────────────────────────────
add_library(robot_math STATIC
    libs/robot_math/src/vec3.cpp
)

target_include_directories(robot_math
    PUBLIC
        libs/robot_math/include
    # PUBLIC: consumers of robot_math (including sensor_utils and robot_app)
    # automatically get libs/robot_math/include on their include path.
)

# R5: Warnings (PRIVATE — compile-time only, not propagated to consumers)
target_compile_options(robot_math PRIVATE -Wall -Wextra)

# ── R3: sensor_utils static library ──────────────────────────────────────────
add_library(sensor_utils STATIC
    libs/sensor_utils/src/lowpass.cpp
)

target_include_directories(sensor_utils
    PUBLIC
        libs/sensor_utils/include
    # PUBLIC: consumers get sensor_utils/include on their include path.
)

target_link_libraries(sensor_utils
    PUBLIC
        robot_math
    # PUBLIC (not PRIVATE): sensor_utils/lowpass.hpp includes
    # <robot_math/vec3.hpp>. Every target that includes lowpass.hpp also needs
    # robot_math's include directories. PUBLIC propagates robot_math's PUBLIC
    # include dirs transitively to all consumers of sensor_utils.
    #
    # If this were PRIVATE, robot_app's compile step would fail with:
    #   fatal error: robot_math/vec3.hpp: No such file or directory
    # because robot_app would not have robot_math/include on its path.
)

target_compile_options(sensor_utils PRIVATE -Wall -Wextra)

# ── R4: robot_app executable ──────────────────────────────────────────────────
add_executable(robot_app
    app/src/main.cpp
)

target_link_libraries(robot_app
    PRIVATE
        sensor_utils
    # Does robot_app need to explicitly link robot_math? NO.
    # Because sensor_utils links robot_math PUBLIC, robot_math is a transitive
    # dependency of sensor_utils. CMake propagates robot_math's include
    # directories and link flags to robot_app automatically.
    #
    # main.cpp uses #include <robot_math/vec3.hpp> directly. This compiles
    # because robot_math's include dirs propagate through sensor_utils (PUBLIC).
    # The linker resolves robot_math symbols through the transitive dependency.
)

target_compile_options(robot_app PRIVATE -Wall -Wextra)

# ── R6: Optional AddressSanitizer ─────────────────────────────────────────────
option(WITH_ASAN "Enable AddressSanitizer and UBSan" OFF)

if(WITH_ASAN)
    message(STATUS "AddressSanitizer enabled")
    foreach(tgt robot_math sensor_utils robot_app)
        target_compile_options(${tgt} PRIVATE
            -fsanitize=address
            -fno-omit-frame-pointer   # better stack traces
        )
        target_link_options(${tgt} PRIVATE
            -fsanitize=address
        )
    endforeach()
endif()

# ── R8: Install targets ───────────────────────────────────────────────────────
install(
    TARGETS robot_math sensor_utils robot_app
    ARCHIVE DESTINATION lib       # static libraries (.a)
    LIBRARY DESTINATION lib       # shared libraries (.so) if any
    RUNTIME DESTINATION bin       # executables
)

install(
    DIRECTORY
        libs/robot_math/include/
        libs/sensor_utils/include/
    DESTINATION include
)

# Usage:
#   cmake -B build -DCMAKE_BUILD_TYPE=Debug
#   cmake --build build
#   ./build/robot_app
#
#   # Install to a local prefix:
#   cmake --install build --prefix /tmp/robot_install
#   ls /tmp/robot_install/include/robot_math/
#   ls /tmp/robot_install/bin/
#
#   # With sanitizers:
#   cmake -B build -DCMAKE_BUILD_TYPE=Debug -DWITH_ASAN=ON
#   cmake --build build
#   ./build/robot_app
