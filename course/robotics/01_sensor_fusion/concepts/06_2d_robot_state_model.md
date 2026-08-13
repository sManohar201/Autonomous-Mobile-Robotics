# 5. 2D Robot State Model

For a ground robot with 2D position and heading:

```
x = [px, py, θ]ᵀ
```

### Process model (differential drive)

Given linear velocity v and angular velocity ω over timestep Δt:

```
px' = px + v · cos(θ) · Δt
py' = py + v · sin(θ) · Δt
θ'  = θ + ω · Δt
```

Process Jacobian Fₜ = ∂f/∂x:

```
    [1   0   -v·sin(θ)·Δt]
F = [0   1    v·cos(θ)·Δt]
    [0   0    1           ]
```

### GPS measurement model

GPS measures position directly:

```
z = [px, py]ᵀ
h(x) = [px, py]ᵀ

H = [1  0  0]
    [0  1  0]
```

### Odometry measurement model

Odometry measures differential changes:

```
z = [Δpx, Δpy, Δθ]ᵀ — incremental pose change
```

This uses the full state; H = I₃.

### IMU measurement model

IMU measures angular velocity ω directly:

```
z = ω
h(x) = θ_dot (rate of change of heading)
```
