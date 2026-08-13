# Package Boundaries

A package should have a clear reason to exist. Common production package categories:

- `robot_interfaces`: custom messages, services, actions
- `robot_bringup`: launch files and top-level orchestration
- `robot_description`: URDF/Xacro, meshes, RViz configs
- `robot_monitoring`: health aggregation and diagnostics
- `robot_localization_app`: localization nodes and configuration
- `robot_navigation_app`: planning and command orchestration
