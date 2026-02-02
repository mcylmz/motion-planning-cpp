#pragma once

#include "search_algorithm.hpp"
#include <queue>
#include <unordered_set>
#include <unordered_map>

namespace motion_planning {

class BFS : public GridSearchAlgorithm {
public:
    // Use 4-connected (false) or 8-connected (true) grid
    explicit BFS(bool use8Connected = false) : use8Connected_(use8Connected) {}

    GridPath search(Grid& grid, const Cell& start, const Cell& goal) override;

    GridPath search(Grid& grid, const Cell& start, const Cell& goal,
                   VisualizationCallback callback) override;

    std::string name() const override { return "BFS"; }

private:
    bool use8Connected_;
};

}  // namespace motion_planning
