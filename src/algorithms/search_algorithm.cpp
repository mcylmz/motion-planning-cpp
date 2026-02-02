#include "algorithms/search_algorithm.hpp"

namespace motion_planning {

GridPath GridSearchAlgorithm::reconstructPath(
    const std::unordered_map<Cell, Cell, CellHash>& cameFrom,
    const Cell& start,
    const Cell& goal
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
        path.cost += Grid::movementCost(it->second, current);
        current = it->second;
    }
    path.cells.push_back(start);

    // Reverse to get start-to-goal order
    std::reverse(path.cells.begin(), path.cells.end());

    return path;
}

}  // namespace motion_planning
