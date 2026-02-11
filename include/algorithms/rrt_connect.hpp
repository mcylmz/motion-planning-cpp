#pragma once

#include "sampling_algorithm.hpp"
#include <vector>
#include <random>

namespace motion_planning {

class RRTConnect : public SamplingAlgorithm {
public:
    struct Config {
        int maxIterations = 5000;
        double stepSize = 15.0;
        double connectThreshold = 1.0;  // Trees connected when distance < this
        unsigned int seed = 42;
    };

    explicit RRTConnect(const Config& config = Config()) : config_(config) {}

    Path plan(const Environment& env, const Vec2& start, const Vec2& goal) override;
    Path plan(const Environment& env, const Vec2& start, const Vec2& goal,
             TreeVisualizationCallback callback) override;

    std::string name() const override { return "RRT-Connect"; }

    // Access start tree (A) nodes/edges for visualization
    const std::vector<Vec2>& getNodesStart() const { return nodesA_; }
    const std::vector<std::pair<int, int>>& getEdgesStart() const { return edgesA_; }

    // Access goal tree (B) nodes/edges for visualization
    const std::vector<Vec2>& getNodesGoal() const { return nodesB_; }
    const std::vector<std::pair<int, int>>& getEdgesGoal() const { return edgesB_; }

    void setConfig(const Config& config) { config_ = config; }

private:
    enum class ExtendResult { Reached, Advanced, Trapped };

    // Find index of nearest node in the given tree
    int nearestNode(const std::vector<Vec2>& nodes, const Vec2& point) const;

    // Steer from 'from' toward 'to' by at most stepSize
    Vec2 steer(const Vec2& from, const Vec2& to) const;

    // One step toward target; returns result and sets resultIndex
    ExtendResult extend(const Environment& env,
                        std::vector<Vec2>& nodes,
                        std::vector<int>& parent,
                        std::vector<std::pair<int, int>>& edges,
                        const Vec2& target,
                        int& resultIndex,
                        TreeVisualizationCallback callback);

    // Repeatedly extend toward target until Reached or Trapped
    ExtendResult connect(const Environment& env,
                         std::vector<Vec2>& nodes,
                         std::vector<int>& parent,
                         std::vector<std::pair<int, int>>& edges,
                         const Vec2& target,
                         int& resultIndex,
                         TreeVisualizationCallback callback);

    // Reconstruct path from both trees meeting at given indices
    Path reconstructPath(int startTreeIdx, int goalTreeIdx) const;

    Config config_;

    // Tree A (start tree)
    std::vector<Vec2> nodesA_;
    std::vector<int> parentA_;
    std::vector<std::pair<int, int>> edgesA_;

    // Tree B (goal tree)
    std::vector<Vec2> nodesB_;
    std::vector<int> parentB_;
    std::vector<std::pair<int, int>> edgesB_;
};

}  // namespace motion_planning
