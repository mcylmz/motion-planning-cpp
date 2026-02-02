#pragma once

#include "core/types.hpp"
#include "core/grid.hpp"
#include "visualization/visualizer.hpp"
#include <functional>

namespace motion_planning {

// Callback for visualization during search
using VisualizationCallback = std::function<void(const Cell& cell, CellState state)>;

class GridSearchAlgorithm {
public:
    virtual ~GridSearchAlgorithm() = default;

    // Main search function
    virtual GridPath search(Grid& grid, const Cell& start, const Cell& goal) = 0;

    // Search with visualization callback
    virtual GridPath search(Grid& grid, const Cell& start, const Cell& goal,
                           VisualizationCallback callback) = 0;

    // Get search statistics
    const SearchStats& getStats() const { return stats_; }

    // Algorithm name for display
    virtual std::string name() const = 0;

protected:
    SearchStats stats_;

    // Reconstruct path from parent map
    GridPath reconstructPath(
        const std::unordered_map<Cell, Cell, CellHash>& cameFrom,
        const Cell& start,
        const Cell& goal
    );
};

}  // namespace motion_planning
