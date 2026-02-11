#pragma once

#include "sampling_algorithm.hpp"
#include <vector>
#include <random>

namespace motion_planning {

class RRT : public SamplingAlgorithm {
public:
    struct Config {
        int maxIterations = 5000;
        double stepSize = 10.0;
        double goalThreshold = 10.0;   // Connect to goal if within this distance
        double goalBias = 0.05;        // Probability of sampling the goal directly
        unsigned int seed = 42;
    };

    explicit RRT(const Config& config = Config()) : config_(config) {}

    Path plan(const Environment& env, const Vec2& start, const Vec2& goal) override;
    Path plan(const Environment& env, const Vec2& start, const Vec2& goal,
             TreeVisualizationCallback callback) override;

    std::string name() const override { return "RRT"; }

    // Access tree nodes/edges for visualization after planning
    const std::vector<Vec2>& getNodes() const { return nodes_; }
    const std::vector<std::pair<int, int>>& getEdges() const { return edges_; }

    void setConfig(const Config& config) { config_ = config; }

private:
    // Find index of nearest node to the given point
    int nearestNode(const Vec2& point) const;

    // Steer from 'from' toward 'to' by at most stepSize
    Vec2 steer(const Vec2& from, const Vec2& to) const;

    // Reconstruct path from goal node back to root
    Path reconstructPath(int goalIndex) const;

    Config config_;

    // Tree storage
    std::vector<Vec2> nodes_;
    std::vector<int> parent_;                   // parent_[i] = parent index of node i
    std::vector<std::pair<int, int>> edges_;     // (parent, child) pairs
};

}  // namespace motion_planning
