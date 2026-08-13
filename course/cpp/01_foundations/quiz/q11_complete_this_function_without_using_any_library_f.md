# **Q11.** Complete this function without using any library function other than `std::acos` and `std::sqrt`.

## Part D — Code Completion

```cpp
double angle_between(double ax, double ay, double bx, double by) {
    // TODO
}
```

<details>
<summary>Answer</summary>

```cpp
double angle_between(double ax, double ay, double bx, double by) {
    double dot   = ax*bx + ay*by;
    double mag_a = std::sqrt(ax*ax + ay*ay);
    double mag_b = std::sqrt(bx*bx + by*by);
    double cos_theta = dot / (mag_a * mag_b);
    // clamp to [-1,1] — floating-point can exceed bounds and cause NaN
    cos_theta = cos_theta < -1.0 ? -1.0 : (cos_theta > 1.0 ? 1.0 : cos_theta);
    return std::acos(cos_theta);
}
```

</details>
