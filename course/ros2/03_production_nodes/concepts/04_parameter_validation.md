# Parameter Validation

Parameter validation should happen before activation. Invalid parameters should produce actionable error messages.

Examples:

- expected rate must be positive
- frame ID must be non-empty
- timeout must be greater than zero
- topic name must be absolute or documented as relative
