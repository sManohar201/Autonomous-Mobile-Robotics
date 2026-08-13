# 10. Production Trade-offs

| Decision | Trade-off |
|---|---|
| `best_effort` vs `reliable` QoS | Best-effort: no buffering, minimal latency. Reliable: retries, adds latency and memory |
| Stale threshold | Too tight: false positives in bursty networks. Too loose: real failures slip through |
| Frame validation at every callback | Correct but adds per-message CPU. Alternative: validate once at startup via service |
| Separate health node vs inline | Separate: testable, reusable. Inline: simpler, fewer processes |
| `node->now()` vs `msg->header.stamp` for stale check | Use `node->now()` for receive-time staleness; `msg->header.stamp` for data-freshness (lag from sensor) |
