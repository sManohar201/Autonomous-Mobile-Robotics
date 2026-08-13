# 10. Checklist for a Production Concurrent Node

- [ ] All shared mutable state is protected by a mutex or lives in a MutuallyExclusive callback group
- [ ] Lock scope is as short as possible — no publishing, no heavy computation under the lock
- [ ] Timer callbacks have bounded worst-case runtime (no blocking calls)
- [ ] Executor choice is documented in a comment near `spin()`
- [ ] Callback group assignments are explicit (don't rely on default group behaviour changing)
- [ ] Node tested with `--ros-args --log-level DEBUG` to confirm callback firing rates
