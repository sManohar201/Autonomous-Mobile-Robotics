// Hard Exercise — RRT Path Planning in 2D
//
// Implement Rapidly-exploring Random Tree (RRT) path planning on a 2D integer grid.
//
// Setup:
//   Grid: 10x10, cells are 0=free or 1=obstacle
//   Start: (0, 0)    Goal: (9, 9)
//   Step size: 2 cells
//   Goal bias: 0.1 (sample goal with 10% probability)
//   Max iterations: 2000
//
// Components:
//   - Tree: set of nodes, each with position and parent index
//   - nearest(tree, sample): find node in tree closest to sample (Euclidean)
//   - extend(from, to, step): move from `from` toward `to` by at most `step` cells
//   - collision_free(from, to, grid): check that the straight line from→to is obstacle-free
//     (step along it at 1-cell intervals and check each cell)
//
// Algorithm:
//   1. Init tree with start node (parent = -1)
//   2. For each iteration:
//      a. Sample q_rand (goal with probability 0.1, random otherwise)
//      b. Find nearest node q_near in tree
//      c. Extend toward q_rand → q_new
//      d. If collision_free(q_near, q_new): add q_new to tree
//      e. If q_new is within 2 cells of goal: path found, reconstruct and return
//   3. Return empty if not found within max_iterations
//
// No hints on implementation — design the data structures and code yourself.

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

int main() {
    // TODO: implement RRT

    std::cout << "ex01_hard passed\n";
    return 0;
}
