# 9. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Adapter vs direct filter access | Adapter: decoupled, testable. Direct: less code, harder to test |
| Rejection vs clipping | Rejection: clean, debug-visible via reason string. Clipping: filter keeps running, silently less accurate |
| `Trigger` vs custom service | Trigger: standard, no schema to define. Custom: carries parameters (reset pose, covariance) but adds complexity |
| Diagnostics rate | 0.5–2 Hz is typical — operators need it readable, not performance-critical |
| `initialized_` flag | Guards against large `outlier_jump` rejections before any update — correct filter behaviour |
| Covariance trace threshold | Too tight: rejects valid measurements in noisy environments. Too loose: accepts bad measurements. Tune per sensor spec. |
