#pragma once

#include "sampling_algorithm.hpp"
#include <vector>
#include <random>
#include <limits>

namespace motion_planning {

struct EllipseParams {
    Vec2 center;          // midpoint of start and goal
    double majorAxis;     // cbest / 2
    double minorAxis;     // sqrt(cbest² - cmin²) / 2
    double rotation;      // atan2 of start→goal (radians)
    bool active;          // true once first solution found
};

class InformedRRTStar : public SamplingAlgorithm {
public:
    struct Config {
        int maxIterations = 5000;
        double stepSize = 15.0;
        double goalThreshold = 20.0;
        double goalBias = 0.05;
        double gamma = 50.0;              // Near-neighbor radius tuning constant
        bool continueAfterGoal = true;    // Keep improving after first goal connection
        unsigned int seed = 42;
    };

    explicit InformedRRTStar(const Config& config = Config()) : config_(config) {}

    Path plan(const Environment& env, const Vec2& start, const Vec2& goal) override;
    Path plan(const Environment& env, const Vec2& start, const Vec2& goal,
             TreeVisualizationCallback callback) override;

    std::string name() const override { return "Informed RRT*"; }

    const std::vector<Vec2>& getNodes() const { return nodes_; }
    const std::vector<std::pair<int, int>>& getEdges() const { return edges_; }
    const EllipseParams& getEllipseParams() const { return ellipse_; }
    double getBestCost() const { return bestCost_; }

    void setConfig(const Config& config) { config_ = config; }

private:
    // Ellipse sampling
    Vec2 sampleEllipse(std::mt19937& rng) const;
    void updateEllipse();

    // Find index of nearest node to the given point
    int nearestNode(const Vec2& point) const;

    // Steer from 'from' toward 'to' by at most stepSize
    Vec2 steer(const Vec2& from, const Vec2& to) const;

    // Find all nodes within radius of point
    std::vector<int> nearNodes(const Vec2& point, double radius) const;

    // Compute the near-neighbor search radius
    double nearRadius() const;

    // Euclidean distance between two nodes
    double edgeCost(const Vec2& a, const Vec2& b) const;

    // Propagate cost updates to descendants after a rewire
    void propagateCostUpdate(int nodeIndex);

    // Reconstruct path from a node back to root
    Path reconstructPath(int goalIndex) const;

    // Rebuild edges_ from parent_ array
    void rebuildEdges();

    Config config_;

    // Tree storage
    std::vector<Vec2> nodes_;
    std::vector<int> parent_;
    std::vector<double> cost_;                     // Cost from start to each node
    std::vector<std::vector<int>> children_;       // Children of each node
    std::vector<std::pair<int, int>> edges_;       // (parent, child) pairs for visualization

    // Best goal tracking
    int bestGoalIndex_ = -1;
    double bestCost_ = std::numeric_limits<double>::infinity();

    // Ellipse state
    Vec2 start_, goal_;
    double cmin_ = 0.0;
    EllipseParams ellipse_;
};

}  // namespace motion_planning
