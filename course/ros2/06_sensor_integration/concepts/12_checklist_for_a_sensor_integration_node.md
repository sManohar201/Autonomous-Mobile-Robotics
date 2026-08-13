# 11. Checklist for a Sensor Integration Node

- [ ] QoS profile matches publisher — verified with `ros2 topic info -v`
- [ ] Frame ID validation is parameterised, not hardcoded
- [ ] Stale detection threshold is proportional to expected sensor rate
- [ ] Node tested under both wall time and sim time
- [ ] Health status published at a rate independently readable by operators
- [ ] Covariance fields checked — nodes don't silently use `covariance[0] == -1`
