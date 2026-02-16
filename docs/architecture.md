# Software Architecture

## Overview

The project follows a layered architecture with four distinct layers: Application, Algorithm, Core, and Visualization. The two parallel algorithm families (grid-based and sampling-based) share common core types and visualization infrastructure but operate on different world representations.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              APPLICATION LAYER                                  │
│                                                                                 │
│  ┌────────────────┐  ┌────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │   main.cpp      │  │ grid_search.cpp│  │theta_star_demo  │  │ rrt_demo.cpp    │  │
│  │ (Grid Search)   │  │ (Algo Bench)   │  │ (Theta* GUI)    │  │ (RRT GUI)       │  │
│  │                 │  │                │  │                 │  │                 │  │
│  │ • Mode switch   │  │ • Run all algos│  │ • A* vs Theta*  │  │ • Mode switch   │  │
│  │ • Obstacle draw │  │ • Compare stats│  │ • Any-angle viz │  │ • Obstacle draw │  │
│  │ • Algo select   │  │ • Print table  │  │ • Line-of-sight │  │ • Tree animate  │  │
│  └───────┬─────────┘  └───────┬────────┘  └────────┬────────┘  └───────┬─────────┘  │
│          │                    │                     │                   │            │
│  ┌─────────────────┐  ┌──────────────────┐  ┌──────────────────┐                    │
│  │rrt_star_demo.cpp│  │rrt_connect_demo  │  │  prm_demo.cpp    │                    │
│  │ (RRT* GUI)      │  │ (RRT-Connect GUI)│  │  (PRM GUI)       │                    │
│  │                 │  │                  │  │                  │                    │
│  │ • Mode switch   │  │ • Mode switch    │  │ • Mode switch    │                    │
│  │ • Obstacle draw │  │ • Obstacle draw  │  │ • Obstacle draw  │                    │
│  │ • Tree animate  │  │ • Bidir. trees   │  │ • Roadmap viz    │                    │
│  │ • Toggle cont.  │  │                  │  │ • Adjust k/N     │                    │
│  └────────┬────────┘  └────────┬─────────┘  └────────┬─────────┘                    │
└───────────┼─────────────────────┼─────────────────────┼─────────────────────────────┘
            │                     │                     │
            ▼                     ▼                     ▼
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
│   │    ┌────────┼────────┬────────┐    │   │        ┌────┼────┬─────────┐    │  │
│   │    ▼        ▼        ▼        ▼    │   │        ▼    ▼    ▼         ▼    │  │
│   │ ┌──────┐┌───────┐┌──────┐┌──────┐ │   │  ┌──────┐┌──────┐┌───────┐┌──┐ │  │
│   │ │ BFS  ││Dijkst-││  A*  ││Theta*│ │   │  │ RRT  ││ RRT* ││RRT-Con││PRM│ │  │
│   │ │      ││ ra    ││      ││      │ │   │  │      ││      ││ nect  ││   │ │  │
│   │ │queue ││pqueue ││pqueue││pqueue│ │   │  │sample││sample││bidir. ││kNN│ │  │
│   │ │      ││g-cost ││f=g+h ││f=g+h │ │   │  │steer ││rewire││extend ││A* │ │  │
│   │ └──────┘└───────┘└──────┘│+LOS  │ │   │  └──────┘└──────┘└───────┘└──┘ │  │
│   │                       │  └──────┘ │   │                                  │  │
│   │              ┌────────┘           │   │                                  │  │
│   │              ▼                    │   │                                  │  │
│   │    ┌──────────────────┐           │   │                                  │  │
│   │    │    Heuristics    │           │   │                                  │  │
│   │    │  • manhattan     │           │   │                                  │  │
│   │    │  • euclidean     │           │   │                                  │  │
│   │    │  • chebyshev     │           │   │                                  │  │
│   │    │  • octile        │           │   │                                  │  │
│   │    └──────────────────┘           │   │                                  │  │
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

### Theta* (Any-Angle Planning)

```
User presses SPACE
      │
      ▼
┌────────────────────┐
│ keyCallback        │
│ resets grid viz    │
└────────┬───────────┘
         │
         ▼
┌────────────────────────────────────────────────────┐
│ thetaStar.search(grid, start, goal, vizCallback)   │
│                                                    │
│  cameFrom[start] = start   // parent of start      │
│                                                    │
│  while (frontier not empty) {                      │
│      current = frontier.pop()                      │
│      vizCallback(current, VISITED)  ──▶ redraws    │
│                                                    │
│      if (current == goal) break                    │
│                                                    │
│      for (neighbor : grid.getNeighbors8(current))  │
│                                                    │
│          parent = cameFrom[current]                │
│                                                    │
│          if (lineOfSight(parent, neighbor)) {      │
│              ┌─ Path 2: any-angle shortcut ─────┐  │
│              │ g = g(parent) + euclid(parent, n) │  │
│              │ cameFrom[n] = parent              │  │
│              └───────────────────────────────────┘  │
│          } else {                                  │
│              ┌─ Path 1: standard A* fallback ───┐  │
│              │ g = g(current) + moveCost(cur, n) │  │
│              │ cameFrom[n] = current             │  │
│              └───────────────────────────────────┘  │
│          }                                         │
│  }                                                 │
│                                                    │
│  return reconstructPath(cameFrom, start, goal)     │
│  // Uses Euclidean dist (cells may not be adjacent)│
└────────────────────┬───────────────────────────────┘
                     │
                     ▼
              ┌──────────────┐
              │ GridPath     │  Path has fewer waypoints
              │ • cells[]    │  connected by line-of-sight
              │ • cost       │  (shorter than A*)
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

### RRT* (Optimal Sampling-Based)

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
│ rrtStar.plan(env, start, goal, vizCallback)            │
│                                                        │
│  tree = {start}, cost[start] = 0                       │
│                                                        │
│  for (i = 0; i < maxIterations; i++) {                 │
│      sample = (random < goalBias)                      │
│               ? goal : env.sampleRandom()              │
│                                                        │
│      nearest = tree.nearestNode(sample)                │
│      newPoint = steer(nearest, sample, stepSize)       │
│                                                        │
│      if (!collision-free) continue                     │
│                                                        │
│      ┌─── CHOOSE PARENT (new vs RRT) ───────────────┐ │
│      │ nearNodes = findWithinRadius(newPoint, r)     │ │
│      │ bestParent = argmin(cost[n] + dist(n, new))   │ │
│      │             for n in nearNodes, collision-free │ │
│      └───────────────────────────────────────────────┘ │
│                                                        │
│      tree.add(newPoint, parent=bestParent)             │
│      cost[new] = cost[bestParent] + dist(bestP, new)  │
│                                                        │
│      ┌─── REWIRE (new vs RRT) ──────────────────────┐ │
│      │ for each n in nearNodes:                      │ │
│      │   if cost[new] + dist(new, n) < cost[n]:     │ │
│      │     rewire n's parent → newPoint              │ │
│      │     propagate cost update to descendants      │ │
│      └───────────────────────────────────────────────┘ │
│                                                        │
│      if (newPoint near goal && improves bestCost)      │
│          update best goal connection                   │
│          if (continueAfterGoal) keep iterating ──────┐ │
│          else break                                  │ │
│  }                                ◀──────────────────┘ │
└────────────────────┬───────────────────────────────────┘
                     │
                     ▼
              ┌──────────────┐
              │ Path         │
              │ • points[]   │  Converges toward optimal
              │ • totalLength│  as iterations increase
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
