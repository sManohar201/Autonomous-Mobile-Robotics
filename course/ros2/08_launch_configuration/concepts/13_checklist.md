# 12. Checklist

- [ ] Every tunable value is a declared parameter with a sensible default
- [ ] Config YAML files are installed by `CMakeLists.txt` via `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})`
- [ ] `use_sim_time` is set in the launch file, not in node code
- [ ] `environment` parameter is validated in node code — exit 2 on unknown value
- [ ] Development and field launch files are separate — no `if debug:` logic in launch Python
- [ ] Namespace used when multiple instances of the node can run simultaneously
