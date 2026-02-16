#include "algorithms/theta_star.hpp"
#include <chrono>
#include <limits>
#include <unordered_set>
#include <cmath>
#include <algorithm>

namespace motion_planning {

ThetaStar::ThetaStar(Heuristic heuristic)
    : heuristic_(heuristic) {}

GridPath ThetaStar::search(Grid& grid, const Cell& start, const Cell& goal) {
    return search(grid, start, goal, nullptr);
}

GridPath ThetaStar::search(Grid& grid, const Cell& start, const Cell& goal,
                           VisualizationCallback callback) {
    auto startTime = std::chrono::high_resolution_clock::now();

    stats_ = SearchStats();
    GridPath result;
    result.found = false;

    if (!grid.isValid(start) || !grid.isValid(goal)) {
        return result;
    }

    if (grid.isObstacle(start) || grid.isObstacle(goal)) {
        return result;
    }

    // Priority queue: (f-score, cell)
    using PQEntry = std::pair<double, Cell>;
    auto compare = [](const PQEntry& a, const PQEntry& b) {
        return a.first > b.first;  // Min-heap
    };
    std::priority_queue<PQEntry, std::vector<PQEntry>, decltype(compare)> openSet(compare);

    std::unordered_map<Cell, double, CellHash> gScore;
    std::unordered_map<Cell, double, CellHash> fScore;
    std::unordered_map<Cell, Cell, CellHash> cameFrom;
    std::unordered_set<Cell, CellHash> closedSet;

    // Parent of start is itself (key Theta* initialization)
    cameFrom[start] = start;
    gScore[start] = 0.0;
    fScore[start] = heuristic_(start, goal);
    openSet.push({fScore[start], start});

    while (!openSet.empty()) {
        auto [currentF, current] = openSet.top();
        openSet.pop();

        // Skip if already processed
        if (closedSet.find(current) != closedSet.end()) {
            continue;
        }

        closedSet.insert(current);
        stats_.nodesExpanded++;

        // Visualization callback
        if (callback && !(current == start) && !(current == goal)) {
            callback(current, CellState::Visited);
        }

        // Goal check
        if (current == goal) {
            result = reconstructPath(cameFrom, start, goal);
            break;
        }

        // Always use 8-connected neighbors (any-angle planning requires diagonal movement)
        auto neighbors = grid.getNeighbors8(current);

        for (const Cell& neighbor : neighbors) {
            if (closedSet.find(neighbor) != closedSet.end()) {
                continue;
            }

            const Cell& parent = cameFrom[current];

            if (lineOfSight(grid, parent, neighbor)) {
                // Path 2: any-angle shortcut through parent of current
                int dr = parent.row - neighbor.row;
                int dc = parent.col - neighbor.col;
                double tentativeG = gScore[parent] + std::sqrt(dr * dr + dc * dc);

                auto it = gScore.find(neighbor);
                if (it == gScore.end() || tentativeG < it->second) {
                    cameFrom[neighbor] = parent;
                    gScore[neighbor] = tentativeG;
                    fScore[neighbor] = tentativeG + heuristic_(neighbor, goal);
                    openSet.push({fScore[neighbor], neighbor});
                    stats_.nodesVisited++;

                    if (callback && !(neighbor == goal)) {
                        callback(neighbor, CellState::Frontier);
                    }
                }
            } else {
                // Path 1: standard A* update
                double tentativeG = gScore[current] + Grid::movementCost(current, neighbor);

                auto it = gScore.find(neighbor);
                if (it == gScore.end() || tentativeG < it->second) {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;
                    fScore[neighbor] = tentativeG + heuristic_(neighbor, goal);
                    openSet.push({fScore[neighbor], neighbor});
                    stats_.nodesVisited++;

                    if (callback && !(neighbor == goal)) {
                        callback(neighbor, CellState::Frontier);
                    }
                }
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    stats_.searchTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    stats_.pathLength = result.cost;

    return result;
}

bool ThetaStar::lineOfSight(const Grid& grid, const Cell& a, const Cell& b) const {
    // Bresenham's line algorithm to check if all cells between a and b are free
    int x0 = a.col, y0 = a.row;
    int x1 = b.col, y1 = b.row;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (grid.isObstacle(y0, x0)) {
            return false;
        }

        if (x0 == x1 && y0 == y1) {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    return true;
}

GridPath ThetaStar::reconstructPath(
    const std::unordered_map<Cell, Cell, CellHash>& cameFrom,
    const Cell& start, const Cell& goal
) {
    GridPath path;
    path.found = true;

    Cell current = goal;
    while (!(current == start)) {
        path.cells.push_back(current);
        auto it = cameFrom.find(current);
        if (it == cameFrom.end()) {
            path.found = false;
            path.cells.clear();
            return path;
        }
        // Use Euclidean distance since cells may not be adjacent in Theta*
        int dr = it->second.row - current.row;
        int dc = it->second.col - current.col;
        path.cost += std::sqrt(dr * dr + dc * dc);
        current = it->second;
    }
    path.cells.push_back(start);

    // Reverse to get start-to-goal order
    std::reverse(path.cells.begin(), path.cells.end());

    return path;
}

}  // namespace motion_planning
