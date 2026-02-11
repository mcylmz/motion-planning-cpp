# Software Architecture

## Overview

The project follows a layered architecture with four distinct layers: Application, Algorithm, Core, and Visualization. The two parallel algorithm families (grid-based and sampling-based) share common core types and visualization infrastructure but operate on different world representations.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              APPLICATION LAYER                                  │
│                                                                                 │
│  ┌──────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐  │
│  │     main.cpp         │  │  grid_search.cpp      │  │    rrt_demo.cpp      │  │
│  │  (Grid Search GUI)   │  │  (Algorithm Bench)    │  │  (RRT GUI)           │  │
│  │                      │  │                       │  │                      │  │
│  │  • Mode switching    │  │  • Run all algos      │  │  • Mode switching    │  │
│  │  • Obstacle drawing  │  │  • Compare stats      │  │  • Obstacle drawing  │  │
│  │  • Algo selection    │  │  • Print table         │  │  • Tree animation   │  │
│  └──────────┬───────────┘  └───────────┬───────────┘  └──────────┬───────────┘  │
└─────────────┼──────────────────────────┼──────────────────────────┼──────────────┘
              │                          │                          │
              ▼                          ▼                          ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                             ALGORITHM LAYER                                     │
│                                                                                 │
│   ┌─── Grid-Based (Discrete) ────────┐   ┌─── Sampling-Based (Continuous) ──┐  │
│   │                                   │   │                                  │  │
│   │  ┌─────────────────────────────┐  │   │  ┌────────────────────────────┐  │  │
│   │  │   GridSearchAlgorithm       │  │   │  │   SamplingAlgorithm        │  │  │
│   │  │   (search_algorithm.hpp)    │  │   │  │   (sampling_algorithm.hpp) │  │  │
│   │  │                             │  │   │  │                            │  │  │
│   │  │  virtual search(Grid&,      │  │   │  │  virtual plan(Env&,        │  │  │
│   │  │    Cell, Cell, Callback)    │  │   │  │    Vec2, Vec2, Callback)   │  │  │
│   │  │  virtual name()             │  │   │  │  virtual name()            │  │  │
│   │  │  reconstructPath()          │  │   │  │  getStats()                │  │  │
│   │  └──────────┬──────────────────┘  │   │  └──────────┬─────────────────┘  │  │
│   │             │                     │   │             │                    │  │
│   │    ┌────────┼────────┐            │   │             │                    │  │
│   │    ▼        ▼        ▼            │   │             ▼                    │  │
│   │ ┌──────┐┌───────┐┌──────┐        │   │       ┌──────────┐               │  │
│   │ │ BFS  ││Dijkst-││  A*  │        │   │       │   RRT    │               │  │
│   │ │      ││ ra    ││      │        │   │       │          │               │  │
│   │ │queue ││pqueue ││pqueue│        │   │       │• sample  │               │  │
│   │ │      ││g-cost ││f=g+h │        │   │       │• nearest │               │  │
│   │ └──────┘└───────┘└──────┘        │   │       │• steer   │               │  │
│   │                       │          │   │       │• extend  │               │  │
│   │              ┌────────┘          │   │       └──────────┘               │  │
│   │              ▼                   │   │                                   │  │
│   │    ┌──────────────────┐          │   │    ┌ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┐    │  │
│   │    │    Heuristics    │          │   │      Coming Soon:                 │  │
│   │    │  • manhattan     │          │   │    │ • RRT*                  │    │  │
│   │    │  • euclidean     │          │   │      • RRT-Connect                │  │
│   │    │  • chebyshev     │          │   │    │ • PRM                   │    │  │
│   │    │  • octile        │          │   │     ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─     │  │
│   │    └──────────────────┘          │   │                                  │  │
│   └───────────────────────────────────┘   └──────────────────────────────────┘  │
└───────────────────────────────┬──────────────────────────┬──────────────────────┘
                                │                          │
                                ▼                          ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              CORE LAYER                                         │
│                                                                                 │
│  ┌─────────────────────┐  ┌────────────────────────┐  ┌──────────────────────┐  │
│  │     types.hpp       │  │      grid.hpp          │  │  environment.hpp     │  │
│  │                     │  │                        │  │                      │  │
│  │  • Vec2             │  │  Discrete World        │  │  Continuous World    │  │
│  │  • Cell / CellHash  │◀─│                        │  │                      │  │
│  │  • AABB             │  │  • cells[row][col]     │  │  • rectangles[]      │  │
│  │  • Rectangle        │──▶  • getNeighbors4/8()  │  │  • circles[]         │  │
│  │  • Circle           │  │  • isObstacle()        │  │  • isColliding()     │  │
│  │  • Path / GridPath  │  │  • movementCost()      │  │  • isSegmentCollidng│  │
│  │  • SearchStats      │◀─│  • setStart/Goal()     │  │  • sampleRandom()    │  │
│  │                     │  │  • resetVisualization() │  │  • removeObstacleAt()│  │
│  └─────────────────────┘  └────────────────────────┘  └──────────────────────┘  │
│                                                                                 │
└───────────────────────────────────────────┬─────────────────────────────────────┘
                                            │
                                            ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          VISUALIZATION LAYER                                    │
│                                                                                 │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │                          Visualizer                                       │  │
│  │                                                                           │  │
│  │  Coordinate Mapping:                                                      │  │
│  │    setGrid(Grid*)  ──────▶  scale + offset from grid dimensions           │  │
│  │    setWorldBounds(w, h)──▶  scale + offset from environment dimensions    │  │
│  │    worldToScreen(Vec2)  ◀──▶  screenToWorld(x, y)                         │  │
│  │                                                                           │  │
│  │  Drawing:              Interaction:            Animation:                 │  │
│  │    drawGrid()            setClickCallback()      setAnimationDelay()      │  │
│  │    drawPath()            setKeyCallback()        delay()                  │  │
│  │    drawLine()            handleEvents()                                   │  │
│  │    drawPoint()                                                            │  │
│  │    drawCircle()                                                           │  │
│  │    drawRectangle()                                                        │  │
│  │    drawStats()                                                            │  │
│  └───────────────────────────────────────────────────────────────────────────┘  │
│                                                                                 │
│  ┌──────────────────────┐                                                       │
│  │    colors.hpp        │  FREE, OBSTACLE, START, GOAL, VISITED,                │
│  │    (color palette)   │  FRONTIER, PATH, TREE_EDGE, TREE_NODE, ...            │
│  └──────────────────────┘                                                       │
│                                                                                 │
│                                   ┌──────────┐                                  │
│                                   │   SFML   │                                  │
│                                   │ (extern) │                                  │
│                                   └──────────┘                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow

### Running an Algorithm

```
  User Input ──▶ Callback ──▶ Algorithm.search/plan()
                                      │
                    ┌─────────────────┼──────────────────┐
                    ▼                 ▼                   ▼
              Read neighbors    Update internal     Notify via callback
              / check collis.   data structures     (VisualizationCallback
              from Grid/Env     (frontier, visited,  or TreeVisualization
                                 cameFrom, cost)     Callback)
                                      │                   │
                                      ▼                   ▼
                                 Return Path/       Visualizer redraws
                                 GridPath           screen each step
```

### Grid Search (BFS / Dijkstra / A*)

```
User presses SPACE
      │
      ▼
┌────────────────────┐
│ keyCallback        │
│ selects algorithm  │
│ resets grid viz    │
└────────┬───────────┘
         │
         ▼
┌────────────────────────────────────────────────────────┐
│ algorithm.search(grid, start, goal, vizCallback)       │
│                                                        │
│  while (frontier not empty) {                          │
│      current = frontier.pop()                          │
│      vizCallback(current, VISITED)  ──▶ redraws screen │
│                                                        │
│      if (current == goal) break                        │
│                                                        │
│      for (neighbor : grid.getNeighbors8(current))      │
│          if (better path) {                            │
│              frontier.push(neighbor)                   │
│              vizCallback(neighbor, FRONTIER)            │
│          }                                             │
│  }                                                     │
│                                                        │
│  return reconstructPath(cameFrom, start, goal)         │
└────────────────────┬───────────────────────────────────┘
                     │
                     ▼
              ┌──────────────┐
              │ GridPath     │
              │ • cells[]    │
              │ • cost       │
              │ • found      │
              └──────────────┘
```

### RRT (Sampling-Based)

```
User presses SPACE
      │
      ▼
┌────────────────────┐
│ keyCallback        │
│ sets seed          │
│ clears prev tree   │
└────────┬───────────┘
         │
         ▼
┌────────────────────────────────────────────────────────┐
│ rrt.plan(env, start, goal, vizCallback)                │
│                                                        │
│  tree = {start}                                        │
│                                                        │
│  for (i = 0; i < maxIterations; i++) {                 │
│      sample = (random < goalBias)                      │
│               ? goal : env.sampleRandom()              │
│                                                        │
│      nearest = tree.nearestNode(sample)                │
│      newPoint = steer(nearest, sample, stepSize)       │
│                                                        │
│      if (env.isFree(newPoint) &&                       │
│          !env.isSegmentColliding(nearest, newPoint)) {  │
│                                                        │
│          tree.add(newPoint, parent=nearest)             │
│          vizCallback({nearest, newPoint}) ──▶ redraws  │
│                                                        │
│          if (newPoint.distanceTo(goal) < threshold)    │
│              return reconstructPath(newPoint)           │
│      }                                                 │
│  }                                                     │
└────────────────────┬───────────────────────────────────┘
                     │
                     ▼
              ┌──────────────┐
              │ Path         │
              │ • points[]   │
              │ • totalLength│
              │ • found      │
              └──────────────┘
```

## Design Patterns

| Pattern | Where | Purpose |
|---------|-------|---------|
| **Strategy** | `GridSearchAlgorithm` / `SamplingAlgorithm` | Swap algorithms via polymorphism |
| **Observer** | `VisualizationCallback` / `TreeVisualizationCallback` | Decouple algorithms from rendering |
| **Model-View** | `Grid`/`Environment` (model) ↔ `Visualizer` (view) | Separate data from presentation |

## Key Design Decisions

1. **Two world representations** — `Grid` for discrete cell-based algorithms, `Environment` for continuous space. They share `types.hpp` primitives but are otherwise independent.

2. **Callback-based visualization** — Algorithms accept an optional callback function rather than holding a reference to the Visualizer. This keeps the algorithm layer free of rendering dependencies.

3. **Coordinate mapping in Visualizer** — Both `setGrid()` and `setWorldBounds()` compute scale/offset to map world coordinates to screen pixels. All drawing methods use `worldToScreen()` internally.
