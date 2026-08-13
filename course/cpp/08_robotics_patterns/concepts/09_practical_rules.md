# 8.8 Practical Rules

- Use observer for event fan-out, not core ownership.
- Use factories for runtime selection of known implementations.
- Use FSMs for lifecycle and recovery.
- Use component storage when many entities share optional data.
- Keep math invariants close to the types that own the math.
