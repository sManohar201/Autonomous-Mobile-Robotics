# 8.4 Plugin and Factory Architecture

A plugin interface defines behavior. A factory creates concrete implementations from configuration.

```cpp
class Localizer {
public:
    virtual ~Localizer() = default;
    virtual void predict(double dt) = 0;
    virtual void update(double z) = 0;
};
```

Factories centralize construction and keep user code independent of concrete types.
