# 11. Production Trade-offs

| Decision | Trade-off |
|---|---|
| Inline params vs YAML files | Inline: visible in launch file. YAML: versionable, shareable, robot-specific |
| Single launch file vs variants | Single with args: fewer files. Variants: clearer intent, easier to review per environment |
| Hardcoded topic names vs remaps | Hardcoded: simpler. Remaps: flexible integration, harder to trace |
| `IfCondition` vs separate launch files | `IfCondition`: one file for all cases. Separate: cleaner, no conditional logic |
| Namespace per robot vs no namespace | Namespace: required for multi-robot. No namespace: simpler single-robot setups |
