# 6. Configuration at the Stack Level

Stack-level YAML ties robot identity to environment:

```yaml
# config/robots/pallet_jack.yaml
deployment_checklist_node:
  ros__parameters:
    config_loaded: false
    operator_confirmed: false

degraded_mode_manager:
  ros__parameters: {}

# config/envs/field.yaml
/**:
  ros__parameters:
    use_sim_time: false
    config_profile: field
```

The `/**:` wildcard applies parameters to ALL nodes in the stack — useful for
`use_sim_time` and environment identity.
