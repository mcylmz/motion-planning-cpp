#include "algorithms/rrt.hpp"
#include <chrono>

namespace motion_planning {

Path RRT::plan(const Environment& env, const Vec2& start, const Vec2& goal) {
    return plan(env, start, goal, nullptr);
}

Path RRT::plan(const Environment& env, const Vec2& start, const Vec2& goal,
               TreeVisualizationCallback callback) {
    auto startTime = std::chrono::high_resolution_clock::now();

    stats_ = SearchStats();
    Path result;
    result.found = false;

    // Initialize tree with start node
    nodes_.clear();
    parent_.clear();
    edges_.clear();

    nodes_.push_back(start);
    parent_.push_back(-1);  // Root has no parent

    std::mt19937 rng(config_.seed);
    std::uniform_real_distribution<double> goalDist(0.0, 1.0);

    for (int i = 0; i < config_.maxIterations; ++i) {
        stats_.nodesVisited++;

        // 1. Sample a random point (with goal bias)
        Vec2 sample;
        if (goalDist(rng) < config_.goalBias) {
            sample = goal;  // Bias toward goal
        } else {
            sample = env.sampleRandom(rng);
        }

        // 2. Find nearest node in the tree
        int nearestIdx = nearestNode(sample);
        const Vec2& nearest = nodes_[nearestIdx];

        // 3. Steer toward sample
        Vec2 newPoint = steer(nearest, sample);

        // 4. Check collision for the new edge
        if (!env.isFree(newPoint)) continue;
        if (env.isSegmentColliding(nearest, newPoint)) continue;

        // 5. Add new node to tree
        int newIdx = static_cast<int>(nodes_.size());
        nodes_.push_back(newPoint);
        parent_.push_back(nearestIdx);
        edges_.push_back({nearestIdx, newIdx});
        stats_.nodesExpanded++;

        // Visualization callback
        if (callback) {
            callback({nearest, newPoint});
        }

        // 6. Check if we can connect to the goal
        if (newPoint.distanceTo(goal) <= config_.goalThreshold) {
            // Verify the connection to goal is collision-free
            if (!env.isSegmentColliding(newPoint, goal)) {
                int goalIdx = static_cast<int>(nodes_.size());
                nodes_.push_back(goal);
                parent_.push_back(newIdx);
                edges_.push_back({newIdx, goalIdx});

                if (callback) {
                    callback({newPoint, goal});
                }

                result = reconstructPath(goalIdx);
                break;
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    stats_.searchTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    stats_.pathLength = result.totalLength;

    return result;
}

int RRT::nearestNode(const Vec2& point) const {
    int nearest = 0;
    double minDist = nodes_[0].distanceTo(point);

    for (int i = 1; i < static_cast<int>(nodes_.size()); ++i) {
        double dist = nodes_[i].distanceTo(point);
        if (dist < minDist) {
            minDist = dist;
            nearest = i;
        }
    }

    return nearest;
}

Vec2 RRT::steer(const Vec2& from, const Vec2& to) const {
    Vec2 direction = to - from;
    double dist = direction.length();

    if (dist <= config_.stepSize) {
        return to;  // Close enough, go directly
    }

    // Move stepSize distance toward target
    return from + direction.normalized() * config_.stepSize;
}

Path RRT::reconstructPath(int goalIndex) const {
    Path path;
    path.found = true;

    int current = goalIndex;
    while (current != -1) {
        path.points.push_back(nodes_[current]);
        current = parent_[current];
    }

    // Reverse to get start-to-goal order
    std::reverse(path.points.begin(), path.points.end());
    path.computeLength();

    return path;
}

}  // namespace motion_planning
