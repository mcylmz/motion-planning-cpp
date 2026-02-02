#include "algorithms/astar.hpp"
#include <chrono>
#include <limits>
#include <unordered_set>

namespace motion_planning {

GridPath AStar::search(Grid& grid, const Cell& start, const Cell& goal) {
    return search(grid, start, goal, nullptr);
}

GridPath AStar::search(Grid& grid, const Cell& start, const Cell& goal,
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
    // f = g + h, where g is cost-so-far and h is heuristic
    using PQEntry = std::pair<double, Cell>;
    auto compare = [](const PQEntry& a, const PQEntry& b) {
        return a.first > b.first;  // Min-heap
    };
    std::priority_queue<PQEntry, std::vector<PQEntry>, decltype(compare)> openSet(compare);

    std::unordered_map<Cell, double, CellHash> gScore;
    std::unordered_map<Cell, double, CellHash> fScore;
    std::unordered_map<Cell, Cell, CellHash> cameFrom;
    std::unordered_set<Cell, CellHash> closedSet;

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

        // Get neighbors
        auto neighbors = use8Connected_
            ? grid.getNeighbors8(current)
            : grid.getNeighbors4(current);

        for (const Cell& neighbor : neighbors) {
            if (closedSet.find(neighbor) != closedSet.end()) {
                continue;
            }

            double tentativeG = gScore[current] + Grid::movementCost(current, neighbor);

            auto it = gScore.find(neighbor);
            if (it == gScore.end() || tentativeG < it->second) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeG;
                fScore[neighbor] = tentativeG + heuristic_(neighbor, goal);
                openSet.push({fScore[neighbor], neighbor});
                stats_.nodesVisited++;

                // Mark frontier for visualization
                if (callback && !(neighbor == goal)) {
                    callback(neighbor, CellState::Frontier);
                }
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    stats_.searchTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    stats_.pathLength = result.cost;

    return result;
}

}  // namespace motion_planning
