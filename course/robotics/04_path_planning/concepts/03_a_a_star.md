# 2. A* (A-star)

A* is a best-first graph search algorithm guaranteed to find the shortest path
if the heuristic is **admissible** (never overestimates).

### Algorithm

```
Open:  priority queue sorted by f(n) = g(n) + h(n)
Closed: visited nodes

1. Add start to Open with f(start) = h(start)
2. Loop:
   a. Pop node n with lowest f from Open
   b. If n == goal: reconstruct and return path
   c. Add n to Closed
   d. For each neighbour m of n:
      - If m in Closed: skip
      - g_new = g(n) + cost(n, m)
      - If m not in Open OR g_new < g(m):
          g(m) = g_new
          f(m) = g_new + h(m)
          parent(m) = n
          Add m to Open
```

### Cost components

- `g(n)`: actual cost from start to n (sum of edge costs along best known path)
- `h(n)`: heuristic estimate of cost from n to goal
- `f(n) = g(n) + h(n)`: estimated total path cost through n

### Admissible heuristics for 2D grid

| Heuristic | Formula | Grid type |
|---|---|---|
| Manhattan | `|dx| + |dy|` | 4-connected |
| Euclidean | `√(dx²+dy²)` | Any |
| Chebyshev | `max(|dx|, |dy|)` | 8-connected |

Euclidean is always admissible and works for any connectivity.

### Dijkstra vs A*

Dijkstra: h(n) = 0 — explores all nodes equally. Complete and optimal.
A*: h(n) > 0 — prioritises nodes closer to goal. Faster but requires heuristic.
