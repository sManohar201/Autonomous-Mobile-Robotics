# 6.8 Practical Rules

- Do not share mutable data without synchronization.
- Use RAII locks.
- Wait on condition variables with predicates.
- Use atomics for simple flags only.
- Define shutdown behavior early.
- Avoid holding locks while executing callbacks.
