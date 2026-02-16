#pragma once

#include "search_algorithm.hpp"
#include "astar.hpp"
#include <queue>
#include <unordered_map>
#include <functional>

namespace motion_planning {

class ThetaStar : public GridSearchAlgorithm {
public:
    explicit ThetaStar(Heuristic heuristic = heuristics::euclidean);

    GridPath search(Grid& grid, const Cell& start, const Cell& goal) override;

    GridPath search(Grid& grid, const Cell& start, const Cell& goal,
                   VisualizationCallback callback) override;

    std::string name() const override { return "Theta*"; }

private:
    bool lineOfSight(const Grid& grid, const Cell& a, const Cell& b) const;
    GridPath reconstructPath(const std::unordered_map<Cell, Cell, CellHash>& cameFrom,
                             const Cell& start, const Cell& goal);
    Heuristic heuristic_;
};

}  // namespace motion_planning
