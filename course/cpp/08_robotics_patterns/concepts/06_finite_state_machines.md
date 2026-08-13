# 8.5 Finite State Machines

State machines are a good fit for drivers and lifecycle components: unconfigured, inactive, active, faulted. Represent states and events with `enum class`, and put transitions in one testable function.
