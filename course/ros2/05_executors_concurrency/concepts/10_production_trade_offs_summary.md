# 9. Production Trade-offs Summary

| Decision | Trade-off |
|---|---|
| SingleThreaded vs Multi | Single: simple, no races. Multi: concurrent but requires explicit sync |
| MutuallyExclusive group | Serialises group — safe for shared state, no mutex needed within group |
| Reentrant group | Parallel — requires every callback to be independently thread-safe |
| Snapshot vs in-place | Snapshot: extra copy cost, but lock held < 1 µs. In-place: stalls subs |
| Mutex vs atomic | Atomic only for single POD values. Mutex for struct or compound state |
| Timer period | Shorter = fresher output, higher CPU. Match to downstream consumer rate |
