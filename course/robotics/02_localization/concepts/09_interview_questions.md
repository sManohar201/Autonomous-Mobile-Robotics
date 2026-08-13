# 8. Interview Questions

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
