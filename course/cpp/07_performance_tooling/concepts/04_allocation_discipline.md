# 7.3 Allocation Discipline

Repeated allocation in hot paths causes jitter. Use:

- `reserve` for vectors
- object reuse for frame buffers
- fixed-size storage where dimensions are known
- bounded queues for real-time pipelines
