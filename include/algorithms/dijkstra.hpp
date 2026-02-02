#pragma once

#include "search_algorithm.hpp"
#include <queue>
#include <unordered_map>

namespace motion_planning {

class Dijkstra : public GridSearchAlgorithm {
public:
    explicit Dijkstra(bool use8Connected = true) : use8Connected_(use8Connected) {}

    GridPath search(Grid& grid, const Cell& start, const Cell& goal) override;

    GridPath search(Grid& grid, const Cell& start, const Cell& goal,
                   VisualizationCallback callback) override;

    std::string name() const override { return "Dijkstra"; }

private:
    bool use8Connected_;
};

}  // namespace motion_planning
