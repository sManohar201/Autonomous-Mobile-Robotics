# 3. Monte Carlo Localization (Particle Filter)

MCL represents the belief p(**x**t | ...) as a set of weighted particles:
```
{(xᵢ, wᵢ)}   i = 1..N
```

Each particle is a hypothesis about where the robot is.

### Algorithm (per timestep)

```
[Predict]
For each particle xᵢ:
    Sample xᵢ' ~ p(x | xᵢ, u)   (motion model + noise)

[Update]  
For each particle xᵢ':
    wᵢ = p(z | xᵢ', m)          (observation likelihood)

[Normalise]
wᵢ ← wᵢ / Σwⱼ

[Resample]
Draw N new particles with replacement, proportional to weights
```

### Motion model

For a 2D differential drive with odometry (Δd, Δθ):

```cpp
// Sample from motion model with Gaussian noise
double noise_d   = gauss(0, alpha1 * delta_d + alpha2 * delta_theta);
double noise_th  = gauss(0, alpha3 * delta_d + alpha4 * delta_theta);

particle.x     += (delta_d + noise_d) * std::cos(particle.theta);
particle.y     += (delta_d + noise_d) * std::sin(particle.theta);
particle.theta += delta_theta + noise_th;
```

`alpha1–alpha4` are robot-specific noise parameters (tuned from odometry slip).

### Observation model (beam model for LiDAR)

For each laser beam i:
```
p(zᵢ | x, m) = w_hit  · N(zᵢ; d_expected, σ²)    (correct beam)
             + w_short · λ · exp(-λ·zᵢ) · [zᵢ < d]  (unexpected obstacle)
             + w_max   · [zᵢ = z_max]               (missed beam)
             + w_rand  · 1/z_max                     (random)
```

In practice, AMCL uses the likelihood field model (faster, smoother):
```
p(zᵢ | x, m) ∝ exp(-d²endpoint / 2σ²)
```
where d is the distance from the endpoint to the nearest occupied cell in the map.

### Resampling

Systematic resampling (O(N), low variance):

```cpp
std::vector<Particle> resample(const std::vector<Particle>& particles) {
    const int N = particles.size();
    std::vector<double> cdf(N);
    cdf[0] = particles[0].weight;
    for (int i = 1; i < N; ++i)
        cdf[i] = cdf[i-1] + particles[i].weight;

    const double step = 1.0 / N;
    double r = uniform(0, step);
    std::vector<Particle> new_particles;
    new_particles.reserve(N);

    int j = 0;
    for (int i = 0; i < N; ++i) {
        while (r > cdf[j]) ++j;
        new_particles.push_back(particles[j]);
        r += step;
    }
    return new_particles;
}
```

### Particle count tuning

| N particles | Memory | CPU | Quality |
|---|---|---|---|
| 100 | Tiny | Minimal | Low accuracy, tracking only |
| 500 | Low | Low | Good tracking, rough global |
| 2000 | Moderate | Moderate | Good global localization |
| 10000+ | High | High | Required for large maps |

Adaptive MCL (KLD-Sampling) automatically adjusts N based on filter uncertainty.
