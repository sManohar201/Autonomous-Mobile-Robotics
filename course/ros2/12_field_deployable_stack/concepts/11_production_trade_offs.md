# 10. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Readiness as parameters vs external signals | Parameters: simple. External: auto-confirms when system comes up |
| Mode as string vs enum | String: human-readable in logs. Enum: type-safe, no typo risk |
| Degraded mode published at 2 Hz | Low enough for nav stack to poll. High enough for operator awareness |
| Single aggregator topic vs per-subsystem | Single: one subscription for everything. Per-subsystem: finer-grained routing |
| Bounded recovery vs infinite retry | Bounded: operators informed quickly. Infinite: may self-heal but hides problems |
