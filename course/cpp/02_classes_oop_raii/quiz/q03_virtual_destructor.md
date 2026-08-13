# Q3 — Virtual Destructor

Consider this code:

```cpp
class Sensor {
public:
    ~Sensor() { std::cout << "Sensor dtor\n"; }
    virtual void read() = 0;
};

class Lidar : public Sensor {
public:
    ~Lidar() { std::cout << "Lidar dtor\n"; }
    void read() override { std::cout << "Lidar reading\n"; }
private:
    float* scan_buffer_ = new float[1000];
};

int main() {
    Sensor* s = new Lidar{};
    delete s;
}
```

(a) What output does this program produce, and what resource problem occurs?
(b) What single change fixes it?

<details>
<summary>Answer</summary>

**(a)** Output:

```
Sensor dtor
```

Only `~Sensor()` runs. Because `Sensor::~Sensor` is not `virtual`, the compiler resolves the destructor call statically through the `Sensor*` pointer and calls `~Sensor()` directly — `~Lidar()` is **never called**. The `scan_buffer_` allocated by `Lidar` is never `delete[]`'d, causing a **memory leak**. This is undefined behaviour.

**(b)** Declare the destructor `virtual` in the base class:

```cpp
virtual ~Sensor() { std::cout << "Sensor dtor\n"; }
```

Now `delete s` performs virtual dispatch, calls `~Lidar()` first (which can release `scan_buffer_`), and then calls `~Sensor()`. Output becomes:

```
Lidar dtor
Sensor dtor
```

**Rule:** Any class with at least one virtual function must have a `virtual` destructor.

</details>
