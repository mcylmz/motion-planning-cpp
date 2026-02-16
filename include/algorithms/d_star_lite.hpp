#pragma once

#include "search_algorithm.hpp"
#include "astar.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <limits>
#include <vector>

namespace motion_planning {

struct DStarKey {
    double k1 = 0.0;
    double k2 = 0.0;

    bool operator>(const DStarKey& other) const {
        if (k1 != other.k1) return k1 > other.k1;
        return k2 > other.k2;
    }

    bool operator<(const DStarKey& other) const {
        if (k1 != other.k1) return k1 < other.k1;
        return k2 < other.k2;
    }

    bool operator>=(const DStarKey& other) const {
        return !(*this < other);
    }

    bool operator<=(const DStarKey& other) const {
        return !(*this > other);
    }
};

class DStarLite : public GridSearchAlgorithm {
public:
    explicit DStarLite(bool use8Connected = true,
                       Heuristic heuristic = heuristics::octile);

    // Standard interface (one-shot, for comparison benchmarks)
    GridPath search(Grid& grid, const Cell& start, const Cell& goal) override;
    GridPath search(Grid& grid, const Cell& start, const Cell& goal,
                   VisualizationCallback callback) override;
    std::string name() const override { return "D* Lite"; }

    // === Incremental replanning API ===
    void initialize(Grid& grid, const Cell& start, const Cell& goal);
    bool computeShortestPath(VisualizationCallback callback = nullptr);
    void updateObstacles(const std::vector<Cell>& changedCells);
    Cell moveRobot();
    GridPath getCurrentPath() const;
    Cell getRobotPosition() const;
    bool isAtGoal() const;
    bool hasValidPath() const;

private:
    // Core D* Lite operations
    DStarKey calculateKey(const Cell& s) const;
    void updateVertex(const Cell& u);
    double cost(const Cell& a, const Cell& b) const;

    // Neighbor access
    std::vector<Cell> getNeighbors(const Cell& cell) const;

    // State
    Grid* grid_ = nullptr;
    Cell start_, goal_, robotPos_, lastRobotPos_;
    double km_;
    bool use8Connected_;
    Heuristic heuristic_;
    bool initialized_ = false;

    // Per-node values
    std::unordered_map<Cell, double, CellHash> g_, rhs_;

    // Priority queue: min-heap of (DStarKey, Cell) with lazy deletion
    using PQEntry = std::pair<DStarKey, Cell>;
    struct PQCompare {
        bool operator()(const PQEntry& a, const PQEntry& b) const {
            return a.first > b.first;  // Min-heap
        }
    };
    std::priority_queue<PQEntry, std::vector<PQEntry>, PQCompare> openList_;
    std::unordered_map<Cell, DStarKey, CellHash> inQueue_;

    static constexpr double INF = std::numeric_limits<double>::infinity();
    double getG(const Cell& s) const;
    double getRHS(const Cell& s) const;
};

}  // namespace motion_planning
