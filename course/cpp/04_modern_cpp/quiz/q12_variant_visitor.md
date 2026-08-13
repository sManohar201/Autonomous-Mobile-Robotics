# Q12 - Variant Visitor

Complete `measurement_name`.

```cpp
#include <string>
#include <variant>

struct Imu {};
struct Gps {};
struct Lidar {};

using Measurement = std::variant<Imu, Gps, Lidar>;

std::string measurement_name(const Measurement& m) {
    // TODO
}
```

<details>
<summary>Answer</summary>

```cpp
template <class... Ts>
struct Overload : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
Overload(Ts...) -> Overload<Ts...>;

std::string measurement_name(const Measurement& m) {
    return std::visit(Overload{
        [](const Imu&) { return std::string{"imu"}; },
        [](const Gps&) { return std::string{"gps"}; },
        [](const Lidar&) { return std::string{"lidar"}; }
    }, m);
}
```

</details>
