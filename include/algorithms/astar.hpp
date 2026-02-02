#pragma once

#include "search_algorithm.hpp"
#include <queue>
#include <unordered_map>
#include <functional>

namespace motion_planning {

// Heuristic function type
using Heuristic = std::function<double(const Cell&, const Cell&)>;

// Common heuristics
namespace heuristics {

// Manhattan distance (for 4-connected grids)
inline double manhattan(const Cell& a, const Cell& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

// Euclidean distance
inline double euclidean(const Cell& a, const Cell& b) {
    int dr = a.row - b.row;
    int dc = a.col - b.col;
    return std::sqrt(dr * dr + dc * dc);
}

// Chebyshev distance (for 8-connected grids)
inline double chebyshev(const Cell& a, const Cell& b) {
    return std::max(std::abs(a.row - b.row), std::abs(a.col - b.col));
}

// Octile distance (for 8-connected grids, more accurate)
inline double octile(const Cell& a, const Cell& b) {
    int dr = std::abs(a.row - b.row);
    int dc = std::abs(a.col - b.col);
    return std::max(dr, dc) + (std::sqrt(2.0) - 1.0) * std::min(dr, dc);
}

}  // namespace heuristics


class AStar : public GridSearchAlgorithm {
public:
    explicit AStar(bool use8Connected = true, Heuristic heuristic = heuristics::octile)
        : use8Connected_(use8Connected), heuristic_(heuristic) {}

    GridPath search(Grid& grid, const Cell& start, const Cell& goal) override;

    GridPath search(Grid& grid, const Cell& start, const Cell& goal,
                   VisualizationCallback callback) override;

    std::string name() const override { return "A*"; }

    void setHeuristic(Heuristic h) { heuristic_ = h; }

private:
    bool use8Connected_;
    Heuristic heuristic_;
};

}  // namespace motion_planning
