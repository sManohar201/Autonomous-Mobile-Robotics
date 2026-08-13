# 8.2 Eigen and Linear Algebra Boundaries

Production robotics often uses Eigen for vectors, matrices, transforms, and decompositions. Keep math types near math-heavy code and avoid leaking implementation details through every API. Even without Eigen, the boundary idea matters: pass typed poses, states, and covariances rather than raw arrays.
