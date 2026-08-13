# 11. Checklist

- [ ] Every pure function (freshness, rejection, adapter) has GTest unit tests
- [ ] Node tests use wall-time with explicit timeouts — no `sleep(3)`
- [ ] Bags stored outside git (git-lfs or separate artifact store)
- [ ] CI runs build + unit + lint on every PR; simulation is gated or nightly
- [ ] `colcon test-result --verbose` is the final CI check — zero failures required
