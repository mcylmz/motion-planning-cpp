#include "algorithms/bfs.hpp"
#include <chrono>

namespace motion_planning {

GridPath BFS::search(Grid& grid, const Cell& start, const Cell& goal) {
    return search(grid, start, goal, nullptr);
}

GridPath BFS::search(Grid& grid, const Cell& start, const Cell& goal,
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

    std::queue<Cell> frontier;
    std::unordered_set<Cell, CellHash> visited;
    std::unordered_map<Cell, Cell, CellHash> cameFrom;

    frontier.push(start);
    visited.insert(start);

    while (!frontier.empty()) {
        Cell current = frontier.front();
        frontier.pop();

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
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                cameFrom[neighbor] = current;
                frontier.push(neighbor);
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
