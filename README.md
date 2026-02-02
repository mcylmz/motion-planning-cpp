# Motion Planning Algorithms

A C++ implementation of motion planning algorithms with SFML visualization.

## Project Structure

```
MotionPlanning/
├── CMakeLists.txt
├── include/
│   ├── core/
│   │   ├── types.hpp          # Basic types (Vec2, Cell, AABB, etc.)
│   │   └── grid.hpp           # Grid world representation
│   ├── algorithms/
│   │   ├── search_algorithm.hpp  # Base class for search algorithms
│   │   ├── bfs.hpp            # Breadth-First Search
│   │   ├── dijkstra.hpp       # Dijkstra's algorithm
│   │   └── astar.hpp          # A* with heuristics
│   └── visualization/
│       ├── colors.hpp         # Color definitions
│       └── visualizer.hpp     # SFML visualization
├── src/
│   ├── core/
│   │   └── grid.cpp
│   ├── algorithms/
│   │   ├── search_algorithm.cpp
│   │   ├── bfs.cpp
│   │   ├── dijkstra.cpp
│   │   └── astar.cpp
│   ├── visualization/
│   │   └── visualizer.cpp
│   └── main.cpp
└── examples/
    └── grid_search.cpp
```

## Requirements

- CMake 3.16+
- C++17 compiler
- SFML 2.5+

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
./motion_planning      # Interactive visualizer
./example_grid         # Grid search comparison
```

## Controls (Interactive Visualizer)

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

## Implemented Algorithms

### Phase 1: Graph-Based Search (Current)
- [x] BFS (Breadth-First Search)
- [x] Dijkstra's Algorithm
- [x] A* with multiple heuristics

### Phase 2: Coming Soon
- [ ] Theta* (any-angle planning)
- [ ] D* Lite (dynamic replanning)
- [ ] RRT (Rapidly-exploring Random Tree)
- [ ] RRT*
- [ ] PRM (Probabilistic Roadmap)
