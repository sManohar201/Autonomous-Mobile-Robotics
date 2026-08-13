# Q5 — Copy vs Move and `std::move`

## Predict-the-Output Questions

What does this program print? Explain why each line of output appears.

```cpp
#include <iostream>
#include <string>
#include <utility>

class Tag {
public:
    Tag(const char* s) : s_{s} {}

    Tag(const Tag& o)       : s_{o.s_}            { std::cout << "copy " << s_ << "\n"; }
    Tag(Tag&& o) noexcept   : s_{std::move(o.s_)} { std::cout << "move " << s_ << "\n"; }

    ~Tag() { std::cout << "dtor " << s_ << "\n"; }

    std::string s_;
};

Tag make_tag() {
    Tag t{"sensor"};
    return t;
}

int main() {
    Tag a{"alpha"};
    Tag b = a;
    Tag c = std::move(a);
    Tag d = make_tag();
}
```

<details>
<summary>Answer</summary>

```
copy alpha
move alpha
move sensor       <- may be elided; see note
dtor sensor
dtor alpha
dtor alpha
dtor alpha
```

**Line by line:**

1. `Tag a{"alpha"}` — calls the `const char*` converting constructor. No output (no copy/move).
2. `Tag b = a` — `a` is an lvalue, so the **copy constructor** is called: prints `copy alpha`.
3. `Tag c = std::move(a)` — `std::move(a)` casts `a` to an rvalue reference. The **move constructor** is selected: prints `move alpha`. After this, `a.s_` is in a valid but unspecified state (typically empty string).
4. `Tag d = make_tag()` — the compiler is likely to apply **NRVO** (Named Return Value Optimisation), constructing `t` directly into `d`'s storage. If NRVO fires, there is **no copy or move** printed. If NRVO is suppressed (e.g., `-fno-elide-constructors`), the move constructor fires: `move sensor`.

**Destructor order** (LIFO on scope exit): `d`, `c`, `b`, `a` — four `dtor` lines. `c.s_` is `"alpha"` (moved from), `a.s_` is likely `""` after the move, but the destructor still runs on `a`.

**Note:** Exact output for `d` depends on whether NRVO is applied. In practice with optimisations, no copy/move line appears for `make_tag()`.

</details>
