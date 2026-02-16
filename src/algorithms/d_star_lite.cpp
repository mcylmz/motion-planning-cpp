#include "algorithms/d_star_lite.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

namespace motion_planning {

DStarLite::DStarLite(bool use8Connected, Heuristic heuristic)
    : use8Connected_(use8Connected), heuristic_(heuristic), km_(0.0) {}

// ============================================================
// Standard GridSearchAlgorithm interface (one-shot)
// ============================================================

GridPath DStarLite::search(Grid& grid, const Cell& start, const Cell& goal) {
    return search(grid, start, goal, nullptr);
}

GridPath DStarLite::search(Grid& grid, const Cell& start, const Cell& goal,
                           VisualizationCallback callback) {
    auto startTime = std::chrono::high_resolution_clock::now();
    stats_ = SearchStats();

    initialize(grid, start, goal);
    bool found = computeShortestPath(callback);

    GridPath result;
    if (found) {
        result = getCurrentPath();
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    stats_.searchTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    stats_.pathLength = result.cost;

    return result;
}

// ============================================================
// Incremental replanning API
// ============================================================

void DStarLite::initialize(Grid& grid, const Cell& start, const Cell& goal) {
    grid_ = &grid;
    start_ = start;
    goal_ = goal;
    robotPos_ = start;
    lastRobotPos_ = start;
    km_ = 0.0;
    stats_ = SearchStats();
    initialized_ = true;

    // Clear all data structures
    g_.clear();
    rhs_.clear();
    inQueue_.clear();
    // Clear priority queue by swapping with empty
    decltype(openList_) empty;
    std::swap(openList_, empty);

    // Initialize goal: rhs(goal) = 0, g(goal) = INF (default)
    rhs_[goal_] = 0.0;

    // Insert goal into priority queue
    DStarKey key = calculateKey(goal_);
    openList_.push({key, goal_});
    inQueue_[goal_] = key;
}

bool DStarLite::computeShortestPath(VisualizationCallback callback) {
    while (!openList_.empty()) {
        // Peek at top
        auto [topKey, topCell] = openList_.top();

        DStarKey robotKey = calculateKey(robotPos_);
        double robotG = getG(robotPos_);
        double robotRHS = getRHS(robotPos_);

        // Termination: queue top >= robot key AND robot is consistent
        if (topKey >= robotKey && robotRHS == robotG) {
            break;
        }

        openList_.pop();

        // Lazy deletion: skip if cell is no longer in queue with this key
        auto it = inQueue_.find(topCell);
        if (it == inQueue_.end()) {
            continue;
        }
        // Check if key matches (cell may have been reinserted with different key)
        if (it->second.k1 != topKey.k1 || it->second.k2 != topKey.k2) {
            continue;
        }

        Cell u = topCell;
        DStarKey oldKey = topKey;
        DStarKey newKey = calculateKey(u);

        stats_.nodesExpanded++;

        if (callback && !(u == start_) && !(u == goal_)) {
            callback(u, CellState::Visited);
        }

        if (oldKey < newKey) {
            // Lazy key update: reinsert with updated key
            inQueue_[u] = newKey;
            openList_.push({newKey, u});
        } else if (getG(u) > getRHS(u)) {
            // Overconsistent: make consistent
            g_[u] = getRHS(u);
            inQueue_.erase(u);

            // Update all predecessors
            for (const Cell& s : getNeighbors(u)) {
                updateVertex(s);
                if (callback && !(s == start_) && !(s == goal_) &&
                    getG(s) != getRHS(s)) {
                    callback(s, CellState::Frontier);
                }
            }
        } else {
            // Underconsistent: set g to infinity
            g_[u] = INF;
            inQueue_.erase(u);

            // Update u itself and all predecessors
            updateVertex(u);
            for (const Cell& s : getNeighbors(u)) {
                updateVertex(s);
                if (callback && !(s == start_) && !(s == goal_) &&
                    getG(s) != getRHS(s)) {
                    callback(s, CellState::Frontier);
                }
            }
        }
    }

    return getG(robotPos_) != INF;
}

void DStarLite::updateObstacles(const std::vector<Cell>& changedCells) {
    if (!initialized_ || !grid_) return;

    // Accumulate heuristic correction for robot movement
    km_ += heuristic_(lastRobotPos_, robotPos_);
    lastRobotPos_ = robotPos_;

    for (const Cell& c : changedCells) {
        // Update the changed cell itself
        updateVertex(c);

        // Update all neighbors of the changed cell
        for (const Cell& n : getNeighbors(c)) {
            updateVertex(n);
        }
    }
}

Cell DStarLite::moveRobot() {
    if (!initialized_ || !grid_) return robotPos_;
    if (isAtGoal()) return robotPos_;

    // Move to the neighbor that minimizes cost(robotPos, n) + g(n)
    double bestCost = INF;
    Cell bestNext = robotPos_;

    for (const Cell& n : getNeighbors(robotPos_)) {
        double c = cost(robotPos_, n);
        if (c == INF) continue;

        double total = c + getG(n);
        if (total < bestCost) {
            bestCost = total;
            bestNext = n;
        }
    }

    robotPos_ = bestNext;
    return robotPos_;
}

GridPath DStarLite::getCurrentPath() const {
    GridPath path;
    path.found = false;

    if (!initialized_ || !grid_) return path;
    if (getG(robotPos_) == INF) return path;

    path.found = true;
    path.cost = 0.0;

    Cell current = robotPos_;
    std::unordered_set<Cell, CellHash> visited;

    while (!(current == goal_)) {
        path.cells.push_back(current);
        visited.insert(current);

        double bestCost = INF;
        Cell bestNext = current;

        for (const Cell& n : getNeighbors(current)) {
            if (visited.count(n)) continue;
            double c = cost(current, n);
            if (c == INF) continue;

            double total = c + getG(n);
            if (total < bestCost) {
                bestCost = total;
                bestNext = n;
            }
        }

        if (bestNext == current || bestCost == INF) {
            path.found = false;
            path.cells.clear();
            path.cost = 0.0;
            return path;
        }

        path.cost += cost(current, bestNext);
        current = bestNext;
    }

    path.cells.push_back(goal_);
    return path;
}

Cell DStarLite::getRobotPosition() const {
    return robotPos_;
}

bool DStarLite::isAtGoal() const {
    return robotPos_ == goal_;
}

bool DStarLite::hasValidPath() const {
    return initialized_ && getG(robotPos_) != INF;
}

// ============================================================
// Private helpers
// ============================================================

DStarKey DStarLite::calculateKey(const Cell& s) const {
    double gVal = getG(s);
    double rhsVal = getRHS(s);
    double minGRhs = std::min(gVal, rhsVal);

    return {
        minGRhs + heuristic_(s, robotPos_) + km_,
        minGRhs
    };
}

void DStarLite::updateVertex(const Cell& u) {
    if (!grid_) return;

    // If u is not the goal, update rhs as min over successors
    if (!(u == goal_)) {
        double minRhs = INF;
        for (const Cell& s : getNeighbors(u)) {
            double c = cost(u, s);
            if (c == INF) continue;
            double val = c + getG(s);
            if (val < minRhs) {
                minRhs = val;
            }
        }
        rhs_[u] = minRhs;
    }

    // Remove from queue if present
    inQueue_.erase(u);

    // If inconsistent, insert with new key
    if (getG(u) != getRHS(u)) {
        DStarKey key = calculateKey(u);
        openList_.push({key, u});
        inQueue_[u] = key;
    }
}

double DStarLite::cost(const Cell& a, const Cell& b) const {
    if (!grid_) return INF;
    if (!grid_->isValid(a) || !grid_->isValid(b)) return INF;
    if (grid_->isObstacle(a) || grid_->isObstacle(b)) return INF;

    return Grid::movementCost(a, b);
}

std::vector<Cell> DStarLite::getNeighbors(const Cell& cell) const {
    if (!grid_) return {};
    if (use8Connected_) {
        return grid_->getNeighbors8(cell);
    }
    return grid_->getNeighbors4(cell);
}

double DStarLite::getG(const Cell& s) const {
    auto it = g_.find(s);
    return (it != g_.end()) ? it->second : INF;
}

double DStarLite::getRHS(const Cell& s) const {
    auto it = rhs_.find(s);
    return (it != rhs_.end()) ? it->second : INF;
}

}  // namespace motion_planning
