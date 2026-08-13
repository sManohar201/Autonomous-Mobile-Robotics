# 7.2 Big-O Is Not Enough

Two `O(n)` algorithms can differ massively because of cache locality, allocation, branching, and vectorization. Robotics pipelines often process large arrays of points, scans, or grid cells, where memory access dominates.
