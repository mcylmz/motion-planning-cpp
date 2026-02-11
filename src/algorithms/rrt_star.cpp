#include "algorithms/rrt_star.hpp"
#include <chrono>
#include <algorithm>
#include <queue>

namespace motion_planning {

Path RRTStar::plan(const Environment& env, const Vec2& start, const Vec2& goal) {
    return plan(env, start, goal, nullptr);
}

Path RRTStar::plan(const Environment& env, const Vec2& start, const Vec2& goal,
                   TreeVisualizationCallback callback) {
    auto startTime = std::chrono::high_resolution_clock::now();

    stats_ = SearchStats();
    Path result;
    result.found = false;

    // Initialize tree with start node
    nodes_.clear();
    parent_.clear();
    cost_.clear();
    children_.clear();
    edges_.clear();
    bestGoalIndex_ = -1;
    bestCost_ = std::numeric_limits<double>::infinity();

    nodes_.push_back(start);
    parent_.push_back(-1);
    cost_.push_back(0.0);
    children_.push_back({});

    std::mt19937 rng(config_.seed);
    std::uniform_real_distribution<double> goalDist(0.0, 1.0);

    for (int i = 0; i < config_.maxIterations; ++i) {
        stats_.nodesVisited++;

        // 1. Sample a random point (with goal bias)
        Vec2 sample;
        if (goalDist(rng) < config_.goalBias) {
            sample = goal;
        } else {
            sample = env.sampleRandom(rng);
        }

        // 2. Find nearest node in the tree
        int nearestIdx = nearestNode(sample);
        const Vec2& nearest = nodes_[nearestIdx];

        // 3. Steer toward sample
        Vec2 newPoint = steer(nearest, sample);

        // 4. Collision check
        if (!env.isFree(newPoint)) continue;
        if (env.isSegmentColliding(nearest, newPoint)) continue;

        // 5. Find near neighbors within radius
        double radius = nearRadius();
        std::vector<int> nearIndices = nearNodes(newPoint, radius);

        // 6. Choose best parent among near neighbors
        int bestParent = nearestIdx;
        double bestParentCost = cost_[nearestIdx] + edgeCost(nearest, newPoint);

        for (int ni : nearIndices) {
            if (ni == nearestIdx) continue;
            double candidateCost = cost_[ni] + edgeCost(nodes_[ni], newPoint);
            if (candidateCost < bestParentCost) {
                // Verify collision-free connection
                if (!env.isSegmentColliding(nodes_[ni], newPoint)) {
                    bestParent = ni;
                    bestParentCost = candidateCost;
                }
            }
        }

        // 7. Add new node to tree
        int newIdx = static_cast<int>(nodes_.size());
        nodes_.push_back(newPoint);
        parent_.push_back(bestParent);
        cost_.push_back(bestParentCost);
        children_.push_back({});
        children_[bestParent].push_back(newIdx);
        stats_.nodesExpanded++;

        // Visualization callback for the new edge
        if (callback) {
            rebuildEdges();
            callback({nodes_[bestParent], newPoint});
        }

        // 8. Rewire: check if routing through new node improves cost for neighbors
        for (int ni : nearIndices) {
            if (ni == bestParent) continue;

            double newCostViaNewNode = cost_[newIdx] + edgeCost(newPoint, nodes_[ni]);
            if (newCostViaNewNode < cost_[ni]) {
                // Verify collision-free connection
                if (!env.isSegmentColliding(newPoint, nodes_[ni])) {
                    // Remove ni from old parent's children
                    int oldParent = parent_[ni];
                    if (oldParent != -1) {
                        auto& oldChildren = children_[oldParent];
                        oldChildren.erase(
                            std::remove(oldChildren.begin(), oldChildren.end(), ni),
                            oldChildren.end());
                    }

                    // Rewire
                    parent_[ni] = newIdx;
                    cost_[ni] = newCostViaNewNode;
                    children_[newIdx].push_back(ni);

                    // Propagate cost update to descendants
                    propagateCostUpdate(ni);

                    // Visualization callback for rewired edge
                    if (callback) {
                        rebuildEdges();
                        callback({newPoint, nodes_[ni]});
                    }
                }
            }
        }

        // 9. Check goal connection, update best path
        if (newPoint.distanceTo(goal) <= config_.goalThreshold) {
            if (!env.isSegmentColliding(newPoint, goal)) {
                double goalCostViaNew = cost_[newIdx] + edgeCost(newPoint, goal);

                if (goalCostViaNew < bestCost_) {
                    // Add goal node if first time, otherwise update connection
                    if (bestGoalIndex_ == -1) {
                        bestGoalIndex_ = static_cast<int>(nodes_.size());
                        nodes_.push_back(goal);
                        parent_.push_back(newIdx);
                        cost_.push_back(goalCostViaNew);
                        children_.push_back({});
                        children_[newIdx].push_back(bestGoalIndex_);
                    } else {
                        // Remove goal from old parent's children
                        int oldParent = parent_[bestGoalIndex_];
                        if (oldParent != -1) {
                            auto& oldChildren = children_[oldParent];
                            oldChildren.erase(
                                std::remove(oldChildren.begin(), oldChildren.end(), bestGoalIndex_),
                                oldChildren.end());
                        }
                        parent_[bestGoalIndex_] = newIdx;
                        cost_[bestGoalIndex_] = goalCostViaNew;
                        children_[newIdx].push_back(bestGoalIndex_);
                    }

                    bestCost_ = goalCostViaNew;

                    if (callback) {
                        rebuildEdges();
                        callback({newPoint, goal});
                    }

                    if (!config_.continueAfterGoal) {
                        break;
                    }
                }
            }
        }
    }

    // Reconstruct best path if goal was reached
    if (bestGoalIndex_ != -1) {
        result = reconstructPath(bestGoalIndex_);
    }

    // Rebuild edges for post-run visualization
    rebuildEdges();

    auto endTime = std::chrono::high_resolution_clock::now();
    stats_.searchTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    stats_.pathLength = result.totalLength;

    return result;
}

int RRTStar::nearestNode(const Vec2& point) const {
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

Vec2 RRTStar::steer(const Vec2& from, const Vec2& to) const {
    Vec2 direction = to - from;
    double dist = direction.length();

    if (dist <= config_.stepSize) {
        return to;
    }

    return from + direction.normalized() * config_.stepSize;
}

std::vector<int> RRTStar::nearNodes(const Vec2& point, double radius) const {
    std::vector<int> result;
    double radiusSq = radius * radius;

    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        if ((nodes_[i] - point).lengthSquared() <= radiusSq) {
            result.push_back(i);
        }
    }

    return result;
}

double RRTStar::nearRadius() const {
    int n = static_cast<int>(nodes_.size());
    if (n <= 1) return config_.stepSize;

    double r = config_.gamma * std::sqrt(std::log(static_cast<double>(n)) / n);
    return std::min(r, config_.stepSize);
}

double RRTStar::edgeCost(const Vec2& a, const Vec2& b) const {
    return a.distanceTo(b);
}

void RRTStar::propagateCostUpdate(int nodeIndex) {
    // BFS through descendants to update costs
    std::queue<int> queue;
    queue.push(nodeIndex);

    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();

        for (int child : children_[current]) {
            double newCost = cost_[current] + edgeCost(nodes_[current], nodes_[child]);
            if (newCost < cost_[child]) {
                cost_[child] = newCost;
                queue.push(child);
            }
        }
    }
}

Path RRTStar::reconstructPath(int goalIndex) const {
    Path path;
    path.found = true;

    int current = goalIndex;
    while (current != -1) {
        path.points.push_back(nodes_[current]);
        current = parent_[current];
    }

    std::reverse(path.points.begin(), path.points.end());
    path.computeLength();

    return path;
}

void RRTStar::rebuildEdges() {
    edges_.clear();
    for (int i = 1; i < static_cast<int>(nodes_.size()); ++i) {
        if (parent_[i] != -1) {
            edges_.push_back({parent_[i], i});
        }
    }
}

}  // namespace motion_planning
