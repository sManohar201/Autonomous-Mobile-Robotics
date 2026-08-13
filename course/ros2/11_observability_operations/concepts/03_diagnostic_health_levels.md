# 2. Diagnostic Health Levels

Use exactly three levels, consistently:

| Level | Meaning | Examples |
|---|---|---|
| `OK` | All checks pass | Sensor publishing at expected rate, frame IDs correct |
| `DEGRADED` | Reduced capability, still safe to operate | One of three sensors stale, navigation slower |
| `FAILED` | Cannot fulfil primary function | All localization measurements rejected |

The key principle: **a subsystem declares its own health, an aggregator collects
and rolls up to robot-level health**. Never hard-code the overall health string in
the monitoring node.

### Status string format

```
subsystem_name:status:detail
localization:DEGRADED:rejected=12 accepted=0 since=3.2s
```

The `:` separators make it easy to parse without a schema:
- `find(':')` extracts the subsystem name
- `severity_rank()` maps status text to an integer for rolling up
