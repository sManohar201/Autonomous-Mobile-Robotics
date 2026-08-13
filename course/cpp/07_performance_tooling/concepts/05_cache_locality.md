# 7.4 Cache Locality

Contiguous memory is faster to traverse than scattered allocations. Prefer `std::vector<Point>` over `std::list<Point>` for point clouds and scans.

Structure-of-arrays can outperform array-of-structures for SIMD-heavy numeric kernels, but it complicates APIs. Measure before changing representation.
