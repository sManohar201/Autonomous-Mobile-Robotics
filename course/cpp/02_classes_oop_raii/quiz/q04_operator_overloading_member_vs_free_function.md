# Q4 — Operator Overloading: Member vs Free Function

You are implementing a `Pose2D` class with `x`, `y`, and `theta` fields.

(a) Should `operator+=` be a member or a free function? Why?
(b) Should `operator+` (binary addition) be a member or a free function? What advantage does the free form have?
(c) Should `operator<<` (stream output) be a member or a free function? Why can't it be a member of `Pose2D`?

<details>
<summary>Answer</summary>

**(a) `operator+=` — member function.**

The left operand is always a `Pose2D`; no implicit conversion on the left side is needed. Compound assignment operators are conventionally members because they modify `*this` and return a reference to it.

**(b) `operator+` — free function.**

A free function allows implicit conversions on **both** sides of the `+`. If `operator+` were a member, `myPose + something` works (implicit conversion on the right), but `something + myPose` fails if `something` is not a `Pose2D`. As a free function, the compiler can apply conversions to either operand. The canonical pattern is to implement `+` in terms of `+=`:

```cpp
Pose2D operator+(Pose2D lhs, const Pose2D& rhs) {
    lhs += rhs;
    return lhs;
}
```

**(c) `operator<<` — free function.**

The left operand of `<<` is `std::ostream&`, which is not `Pose2D`. A member operator is called as `lhs.operator<<(rhs)`, meaning the left operand's type must be `Pose2D`. Since you cannot modify `std::ostream`, `operator<<` must be a free function (or a friend):

```cpp
std::ostream& operator<<(std::ostream& os, const Pose2D& p) {
    return os << "Pose(" << p.x << ", " << p.y << ", " << p.theta << ")";
}
```

</details>
