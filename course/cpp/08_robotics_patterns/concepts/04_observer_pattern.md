# 8.3 Observer Pattern

The observer pattern broadcasts events to subscribers:

```cpp
class SensorBus {
public:
    using Callback = std::function<void(const ImuSample&)>;
    void subscribe(Callback cb);
    void publish(const ImuSample& sample);
};
```

Use it for diagnostics, logging, and event fan-out. Be careful with callback lifetime and lock scope.
