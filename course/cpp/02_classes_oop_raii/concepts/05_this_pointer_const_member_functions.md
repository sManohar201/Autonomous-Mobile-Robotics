# 2.4 `this` Pointer & const Member Functions

Inside a non-static member function, `this` is a pointer to the object the function was called on. Its type is `ClassName*` (or `const ClassName*` in a const member function).

### When You Need `this` Explicitly

```cpp
#include <iostream>

class Filter {
public:
    Filter(double cutoff, double gain)
        : cutoff_{cutoff}, gain_{gain}
    {}

    // Name shadowing: parameter shadows member — use this-> to disambiguate.
    void set_cutoff(double cutoff) {
        this->cutoff_ = cutoff;   // this-> resolves ambiguity
    }

    // Better style: use trailing underscore for members (avoids the problem).
    void set_gain(double gain) {
        gain_ = gain;   // no ambiguity: gain vs gain_
    }

    // Returning *this enables method chaining — call multiple methods in one line.
    // Return type must be Filter& (reference), not Filter (value).
    // Returning by value would return a copy — the chain would operate on
    // throwaway temporaries and the original object would never change.
    Filter& set(double cutoff, double gain) {
        cutoff_ = cutoff;
        gain_   = gain;
        return *this;   // hand the same object back so the next call has something to work on
    }

    // const member function: compiler ensures this function
    // cannot modify any non-mutable members.
    double output(double input) const {
        return input * gain_;   // OK: reading
        // gain_ = 2.0;        // compile error in a const function
    }

    void print() const {
        std::cout << "cutoff=" << cutoff_ << " gain=" << gain_ << "\n";
    }

private:
    double cutoff_;
    double gain_;
};

// A function accepting a const reference can only call const member functions.
void inspect(const Filter& f) {
    f.print();          // OK: print() is const
    // f.set_gain(1.0); // compile error: set_gain() is not const
}

int main() {
    Filter f{100.0, 0.5};

    // Method chaining via returning *this.
    f.set(200.0, 0.8).set(150.0, 0.6).print();

    inspect(f);
}
```

### `this` vs `*this` — and Why Reference, Not Pointer

`this` is a pointer (`Filter*`). `*this` dereferences it to get the object itself (`Filter`). When you return `*this` with return type `Filter&`, you're binding a reference to the object — which points back at the same memory as `this`.

```cpp
// What this means step by step:
f.set(200.0, 0.8).set(150.0, 0.6).print();
//  f.set(200.0, 0.8)  → modifies f, returns Filter& referring to f
//           .set(150.0, 0.6)  → called on f (same object), returns Filter& to f
//                    .print() → called on f
```

You could technically return `this` (a pointer) and chain with `->` instead:

```cpp
Filter* set(double cutoff, double gain) {
    cutoff_ = cutoff; gain_ = gain;
    return this;
}
f.set(200.0, 0.8)->set(150.0, 0.6)->print();   // works, but ugly
```

| Return type | Chain syntax | Signals to reader |
|---|---|---|
| `Filter&` (`*this`) | `f.a().b().c()` | same object, always valid |
| `Filter*` (`this`) | `f.a()->b()->c()` | might be null? |

References communicate "this is the same object, it always exists." Pointers communicate "this might be null, check before using." Method chaining is always the former, so `Filter&` is the right choice.

**Key takeaway:** Mark every member function that does not modify object state as `const`. This allows the function to be called on `const` objects and through `const` references, which is the norm in robotics APIs (sensor callbacks, read-only inspection).
