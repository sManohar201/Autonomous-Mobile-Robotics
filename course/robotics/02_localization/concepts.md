# Robotics Module 02 — Localization

## Goal

Understand the algorithms that answer "where is the robot?" given a map and
sensor data. Cover ICP for scan matching, Monte Carlo Localization (particle
filter) for global localization, and how Nav2's AMCL uses these ideas.

---

## 1. The Localization Problem

Localization: given a known map **m** and sensor history **z₁:t**, **u₁:t**,
compute p(**x**t | **m**, **z₁:t**, **u₁:t**).

Three sub-problems with increasing difficulty:

| Problem | Known? | Unknown? |
|---|---|---|
| Tracking | Initial pose | Future poses |
| Global localization | Map only | Initial pose + future poses |
| Kidnapped robot | Map only | Robot was moved, must re-localize |

EKF (Module 01) solves tracking. Particle filters solve all three.

---

## 2. Iterative Closest Point (ICP)

ICP finds the rigid-body transform (R, t) that best aligns a source point cloud
to a target cloud by iteratively minimising point-to-point distances.

### Algorithm

```
Input:  source cloud S, target cloud T, initial guess T₀
Output: transform (R, t) such that R·S + t ≈ T

Repeat until convergence:
  1. Correspondence:   for each point sᵢ ∈ S, find nearest neighbour tᵢ ∈ T
  2. Transformation:   compute (R, t) minimising Σ ||R·sᵢ + t - tᵢ||²
  3. Apply:            S ← R·S + t
  4. Check:            if RMS change < ε, stop
```

### Step 2: Optimal rotation via SVD

Given matched pairs {(sᵢ, tᵢ)}:

```
Compute centroids:  s̄ = mean(sᵢ),  t̄ = mean(tᵢ)
Demeaned:           sᵢ' = sᵢ - s̄,  tᵢ' = tᵢ - t̄
Cross-covariance:   H = Σ sᵢ' · tᵢ'ᵀ
SVD:                H = U · Σ · Vᵀ
Rotation:           R = V · Uᵀ
Translation:        t = t̄ - R · s̄
```

```cpp
// Compute cross-covariance H
Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
for (size_t i = 0; i < src.size(); ++i) {
    H += (src[i] - src_mean) * (tgt[i] - tgt_mean).transpose();
}
// SVD → rotation
Eigen::JacobiSVD<Eigen::Matrix2d> svd(H,
    Eigen::ComputeFullU | Eigen::ComputeFullV);
Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
// Correct reflection: det(R) must be +1
if (R.determinant() < 0) {
    Eigen::Matrix2d diag = Eigen::Matrix2d::Identity();
    diag(1, 1) = -1.0;
    R = svd.matrixV() * diag * svd.matrixU().transpose();
}
Eigen::Vector2d t = tgt_mean - R * src_mean;
```

### ICP failure modes

| Issue | Cause | Fix |
|---|---|---|
| Local minimum | Bad initial guess | Use odometry for initial guess, coarse-to-fine |
| Slow convergence | Dense clouds | Subsample source |
| Outlier sensitivity | Mismatched points | Trim outliers (TrimmedICP), use robust cost |
| Planar degeneracy | Corridor — no side walls | Add point-to-plane variant |

### Point-to-plane ICP

Instead of minimising point-to-point distance, minimise the component along the
surface normal of the target:

```
cost = Σ (nᵢ · (R·sᵢ + t - tᵢ))²    where nᵢ is the target surface normal
```

Converges in ~3× fewer iterations than point-to-point in structured environments.

---

## 3. Monte Carlo Localization (Particle Filter)

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

---

## 4. Worked Example: 1D Particle Filter

```cpp
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

struct Particle { double x; double weight; };

int main() {
    std::mt19937 rng(42);
    std::uniform_real_distribution<> uniform(0.0, 1.0);
    std::normal_distribution<> gauss(0.0, 1.0);

    const int N = 500;
    const double map_length = 10.0;

    // 1. Initialize uniformly — global localization
    std::vector<Particle> particles(N);
    for (auto& p : particles) {
        p.x = uniform(rng) * map_length;
        p.weight = 1.0 / N;
    }

    // 2. Motion update: robot moves 1.0 m with 0.1 m std noise
    for (auto& p : particles) {
        p.x += 1.0 + gauss(rng) * 0.1;
        p.x = std::clamp(p.x, 0.0, map_length);
    }

    // 3. Observation update: measured distance to wall = 8.0 m
    //    True position is ~ 2.0 m from left wall
    const double measured_dist = 8.0;  // distance to right wall
    const double sigma_sensor = 0.3;
    double weight_sum = 0.0;
    for (auto& p : particles) {
        const double expected_dist = map_length - p.x;
        const double err = measured_dist - expected_dist;
        p.weight = std::exp(-0.5 * err * err / (sigma_sensor * sigma_sensor));
        weight_sum += p.weight;
    }
    for (auto& p : particles) p.weight /= weight_sum;

    // 4. Resample (systematic)
    std::vector<double> cdf(N);
    cdf[0] = particles[0].weight;
    for (int i = 1; i < N; ++i) cdf[i] = cdf[i-1] + particles[i].weight;
    const double step = 1.0 / N;
    double r = uniform(rng) * step;
    std::vector<Particle> new_particles;
    new_particles.reserve(N);
    int j = 0;
    for (int i = 0; i < N; ++i) {
        while (r > cdf[j] && j < N-1) ++j;
        new_particles.push_back(particles[j]);
        new_particles.back().weight = 1.0 / N;
        r += step;
    }

    // 5. Estimate: weighted mean
    double est = 0.0;
    for (const auto& p : new_particles) est += p.x * p.weight;

    std::cout << "Estimated position: " << est << " m\n";
    std::cout << "True position: 2.0 m\n";

    return 0;
}
```

---

## 5. AMCL in Nav2

Nav2's `nav2_amcl` is a particle filter localiser for 2D LiDAR maps.

### Key parameters

```yaml
amcl:
  ros__parameters:
    min_particles: 500
    max_particles: 2000
    pf_err: 0.05           # KLD-sampling error bound
    pf_z: 0.99             # KLD-sampling confidence
    alpha1: 0.2            # motion noise: rotation from rotation
    alpha2: 0.2            # motion noise: rotation from translation
    alpha3: 0.2            # motion noise: translation from translation
    alpha4: 0.2            # motion noise: translation from rotation
    laser_model_type: likelihood_field
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_z_hit: 0.95
    laser_z_rand: 0.05
    set_initial_pose: true
    initial_pose:
      x: 0.0; y: 0.0; yaw: 0.0
```

### Topics

| Topic | Type | Role |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Input LiDAR |
| `/map` | `nav_msgs/OccupancyGrid` | Input map |
| `/odom` | `nav_msgs/Odometry` | Input odometry |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Output best estimate |
| `/particlecloud` | `nav2_msgs/ParticleCloud` | Output all particles |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Input: set initial pose |

### Re-localization

When the robot is kidnapped or loses track:
```bash
# Publish initial pose estimate via CLI
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{pose: {pose: {position: {x: 1.0, y: 0.5}}, covariance: [0.25, 0,0,0,0,0, ...]}}' --once

# Or from RViz: click "2D Pose Estimate" on the map
```

---

## 6. ICP vs Particle Filter

| Property | ICP | Particle Filter |
|---|---|---|
| Problem solved | Scan matching (local) | Global + tracking |
| Initial guess needed | Yes (within ~0.5 m) | No (global localization) |
| Map type | Point cloud | Occupancy grid |
| Computational cost | O(N log N) per iteration | O(N particles · beams) |
| Output | Transform (R, t) | Probability distribution |
| Handles multimodal belief | No | Yes |
| Used in | LiDAR SLAM, ICP-based localization | AMCL, MCL |

In practice: AMCL (particle filter) for 2D navigation, ICP or NDT for 3D SLAM.

---

## 7. Normal Distributions Transform (NDT)

NDT is an alternative to ICP that models the target cloud as a set of Gaussian
distributions over a voxel grid, then maximises the likelihood of source points:

```
score = Σᵢ pᵢᵀ · Σᵢ⁻¹ · pᵢ    where pᵢ = sᵢ - μcell, Σcell = covariance of target in cell
```

NDT advantages:
- More robust to outliers (Gaussian model vs. nearest-neighbour)
- Faster for dense 3D clouds (Autoware uses NDT for LiDAR localisation)
- No explicit correspondence needed

---

## 8. Interview Questions

**Q: How does AMCL handle the kidnapped robot problem?**
A: It doesn't by default — particles must cover the true position. With
`recovery_alpha_slow` and `recovery_alpha_fast`, it detects filter collapse
(low average weight) and injects random particles to restart global search.

**Q: Why does ICP converge to local minima?**
A: It uses greedy nearest-neighbour correspondence — once committed to a wrong
pairing, gradient descent can't escape. Multi-resolution approaches (coarse-to-fine)
and good initialisation from odometry mitigate this.

**Q: What does resampling do to particle diversity?**
A: Resampling concentrates particles near high-weight regions — which is correct
when converged, but kills diversity if the filter is uncertain. KLD-sampling
and adding process noise after resampling preserve diversity.

**Q: What is particle degeneracy and how do you detect it?**
A: All weight concentrates in one particle after resampling. Detect with
Effective Sample Size: N_eff = 1 / Σwᵢ². If N_eff < N/2, resample.

**Q: How is NDT different from ICP?**
A: ICP minimises point-to-point distances with explicit correspondences.
NDT models the target as Gaussians per voxel and maximises source point
likelihood under those models — no correspondences, more robust to outliers.
