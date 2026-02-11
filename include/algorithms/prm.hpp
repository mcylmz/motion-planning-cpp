#pragma once

#include "sampling_algorithm.hpp"
#include <vector>
#include <random>

namespace motion_planning {

class PRM : public SamplingAlgorithm {
public:
    struct Config {
        int numSamples = 500;
        int k = 10;  // Connect each node to k nearest neighbors
        unsigned int seed = 42;
    };

    explicit PRM(const Config& config = Config()) : config_(config) {}

    Path plan(const Environment& env, const Vec2& start, const Vec2& goal) override;
    Path plan(const Environment& env, const Vec2& start, const Vec2& goal,
             TreeVisualizationCallback callback) override;

    std::string name() const override { return "PRM"; }

    // Access roadmap for visualization
    const std::vector<Vec2>& getNodes() const { return nodes_; }
    const std::vector<std::pair<int, int>>& getEdges() const { return edges_; }

    void setConfig(const Config& config) { config_ = config; }

private:
    // Build the roadmap: sample points and connect k-nearest neighbors
    void buildRoadmap(const Environment& env, TreeVisualizationCallback callback);

    // Connect a point to the roadmap, returns its node index
    int connectToRoadmap(const Environment& env, const Vec2& point,
                         TreeVisualizationCallback callback);

    // Find k nearest neighbors of a given node
    std::vector<int> kNearestNeighbors(int nodeIdx, int k) const;

    // A* search on the adjacency list graph
    Path queryPath(int startIdx, int goalIdx);

    // Trace back from goal to start through cameFrom map
    Path reconstructPath(const std::vector<int>& cameFrom, int startIdx, int goalIdx);

    Config config_;

    // Roadmap graph
    std::vector<Vec2> nodes_;
    std::vector<std::vector<int>> adjacency_;  // Adjacency list
    std::vector<std::pair<int, int>> edges_;    // All edges for visualization
};

}  // namespace motion_planning
