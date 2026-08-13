# 9. Checklist

- [ ] Every subsystem publishes to `subsystem/status` with its name as prefix
- [ ] Severity uses exactly `OK`, `DEGRADED`, `FAILED` — not free-form text
- [ ] All warning-level logs in high-frequency callbacks use `RCLCPP_WARN_THROTTLE`
- [ ] Fault reports include `reason` + `action` — operators can act without reading source code
- [ ] Bag recording strategy is documented: what to record and for how long
- [ ] Aggregator outputs worst-of-all-subsystems health at operator-visible topic
- [ ] Health topic published at a rate operators can read (0.5–2 Hz) not sensor rate (50–100 Hz)
