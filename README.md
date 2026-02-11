# Motion Planning Algorithms

A C++ implementation of motion planning algorithms with SFML visualization.

### A* Search
<p align="center">
  <img src="assets/astar.gif" alt="A* pathfinding visualization" width="500">
</p>

### RRT (Rapidly-exploring Random Tree)
<p align="center">
  <img src="assets/rrt_animation.gif" alt="RRT visualization" width="500">
</p>

### RRT* (Optimal RRT)
<p align="center">
  <img src="assets/rrt_star_animation.gif" alt="RRT* visualization" width="500">
</p>

## Project Structure

```
MotionPlanning/
├── CMakeLists.txt
├── include/
│   ├── core/
│   │   ├── types.hpp              # Basic types (Vec2, Cell, AABB, etc.)
│   │   ├── grid.hpp               # Grid world representation
│   │   └── environment.hpp        # Continuous 2D space with geometric obstacles
│   ├── algorithms/
│   │   ├── search_algorithm.hpp   # Base class for grid search algorithms
│   │   ├── bfs.hpp                # Breadth-First Search
│   │   ├── dijkstra.hpp           # Dijkstra's algorithm
│   │   ├── astar.hpp              # A* with heuristics
│   │   ├── sampling_algorithm.hpp # Base class for sampling-based planners
│   │   ├── rrt.hpp                # Rapidly-exploring Random Tree
│   │   └── rrt_star.hpp           # RRT* (Optimal RRT with rewiring)
│   └── visualization/
│       ├── colors.hpp             # Color definitions
│       └── visualizer.hpp         # SFML visualization
├── src/
│   ├── core/
│   │   ├── grid.cpp
│   │   └── environment.cpp
│   ├── algorithms/
│   │   ├── search_algorithm.cpp
│   │   ├── bfs.cpp
│   │   ├── dijkstra.cpp
│   │   ├── astar.cpp
│   │   ├── rrt.cpp
│   │   └── rrt_star.cpp
│   ├── visualization/
│   │   └── visualizer.cpp
│   └── main.cpp
└── examples/
    ├── grid_search.cpp
    ├── rrt_demo.cpp
    └── rrt_star_demo.cpp
```

## Requirements

- CMake 3.16+
- C++17 compiler
- SFML 3.0+

## Installing SFML

### Windows (vcpkg)
```bash
vcpkg install sfml:x64-windows
```

### Ubuntu/Debian
```bash
sudo apt install libsfml-dev
```

### macOS
```bash
brew install sfml
```

## Building

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

If using vcpkg:
```bash
cmake -DCMAKE_TOOLCHAIN_FILE=[vcpkg-root]/scripts/buildsystems/vcpkg.cmake ..
```

## Running

```bash
./motion_planning      # Interactive grid search visualizer
./example_grid         # Grid search algorithm comparison
./example_rrt          # RRT demo with continuous space
./example_rrt_star     # RRT* demo with path optimization
```

## Controls

### Grid Search (`motion_planning` / `example_grid`)

| Key | Action |
|-----|--------|
| Left Click | Draw obstacle / Set start/goal |
| Right Click | Erase obstacle |
| S | Set Start mode |
| G | Set Goal mode |
| D | Draw Obstacle mode |
| 1 | BFS algorithm |
| 2 | Dijkstra algorithm |
| 3 | A* algorithm |
| Space | Run search |
| R | Reset visualization |
| C | Clear all |
| Esc | Quit |

### RRT (`example_rrt`)

| Key | Action |
|-----|--------|
| Left Click | Place obstacle / Set start/goal (depends on mode) |
| Right Click | Remove obstacle under cursor |
| S | Set Start mode |
| G | Set Goal mode |
| D | Draw Rectangle mode |
| O | Draw Circle mode |
| N | Normal mode (no drawing) |
| +/- | Adjust obstacle size (in draw mode) or step size |
| Space | Run RRT |
| R | Reset tree (keep obstacles) |
| C | Clear all obstacles |
| Esc | Quit |

### RRT* (`example_rrt_star`)

Same controls as RRT, plus:

| Key | Action |
|-----|--------|
| T | Toggle continue-after-goal mode |
| Space | Run RRT* |

## Implemented Algorithms

### Graph-Based Search (Discrete Grid)
- [x] BFS (Breadth-First Search)
- [x] Dijkstra's Algorithm
- [x] A* with multiple heuristics (Manhattan, Euclidean, Chebyshev, Octile)

### Sampling-Based Planning (Continuous Space)
- [x] RRT (Rapidly-exploring Random Tree)
- [x] RRT* (Optimal RRT with rewiring)

### Coming Soon
- [ ] RRT-Connect (Bidirectional RRT)
- [ ] PRM (Probabilistic Roadmap)
- [ ] Theta* (Any-angle planning)
- [ ] D* Lite (Dynamic replanning)
