#include "algorithms/dijkstra.hpp"
#include <chrono>
#include <limits>

namespace motion_planning {

GridPath Dijkstra::search(Grid& grid, const Cell& start, const Cell& goal) {
    return search(grid, start, goal, nullptr);
}

GridPath Dijkstra::search(Grid& grid, const Cell& start, const Cell& goal,
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

    // Priority queue: (cost, cell)
    using PQEntry = std::pair<double, Cell>;
    auto compare = [](const PQEntry& a, const PQEntry& b) {
        return a.first > b.first;  // Min-heap
    };
    std::priority_queue<PQEntry, std::vector<PQEntry>, decltype(compare)> frontier(compare);

    std::unordered_map<Cell, double, CellHash> costSoFar;
    std::unordered_map<Cell, Cell, CellHash> cameFrom;

    frontier.push({0.0, start});
    costSoFar[start] = 0.0;

    while (!frontier.empty()) {
        auto [currentCost, current] = frontier.top();
        frontier.pop();

        // Skip if we've already found a better path
        if (currentCost > costSoFar[current]) {
            continue;
        }

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
            double newCost = costSoFar[current] + Grid::movementCost(current, neighbor);

            auto it = costSoFar.find(neighbor);
            if (it == costSoFar.end() || newCost < it->second) {
                costSoFar[neighbor] = newCost;
                cameFrom[neighbor] = current;
                frontier.push({newCost, neighbor});
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
