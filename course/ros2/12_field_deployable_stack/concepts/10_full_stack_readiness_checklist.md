# 9. Full Stack Readiness Checklist

Before a production deployment:

**Software:**
- [ ] All packages built from a tagged release — not `main` branch HEAD
- [ ] All answers packages compile with zero warnings (`-Wall -Wextra`)
- [ ] CI passed on the release tag (build + unit + launch + lint)
- [ ] Simulation test bag replay passed for this release

**Configuration:**
- [ ] Robot-specific YAML loaded and verified (`ros2 param dump --all`)
- [ ] `use_sim_time: false` for all nodes in field launch
- [ ] Bag recording confirmed active before mission start

**Runtime:**
- [ ] `/deployment/readiness` shows `ready=true`
- [ ] `/operator/health` shows `robot_health=OK`
- [ ] `/robot/degraded_mode` shows `NORMAL`
- [ ] `ros2 topic hz` on all sensor topics shows expected rates
