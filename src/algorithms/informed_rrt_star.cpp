#include "algorithms/informed_rrt_star.hpp"
#include <chrono>
#include <algorithm>
#include <queue>
#include <cmath>

namespace motion_planning {

Path InformedRRTStar::plan(const Environment& env, const Vec2& start, const Vec2& goal) {
    return plan(env, start, goal, nullptr);
}

Path InformedRRTStar::plan(const Environment& env, const Vec2& start, const Vec2& goal,
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

    // Store start/goal and compute ellipse constants
    start_ = start;
    goal_ = goal;
    cmin_ = start.distanceTo(goal);
    ellipse_.center = Vec2((start.x + goal.x) / 2.0, (start.y + goal.y) / 2.0);
    ellipse_.rotation = std::atan2(goal.y - start.y, goal.x - start.x);
    ellipse_.active = false;
    ellipse_.majorAxis = 0.0;
    ellipse_.minorAxis = 0.0;

    nodes_.push_back(start);
    parent_.push_back(-1);
    cost_.push_back(0.0);
    children_.push_back({});

    std::mt19937 rng(config_.seed);
    std::uniform_real_distribution<double> goalDist(0.0, 1.0);

    for (int i = 0; i < config_.maxIterations; ++i) {
        stats_.nodesVisited++;

        // 1. Sample: goal bias, then ellipse (if active) or uniform
        Vec2 sample;
        if (goalDist(rng) < config_.goalBias) {
            sample = goal;
        } else if (ellipse_.active) {
            sample = sampleEllipse(rng);
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
                    if (bestGoalIndex_ == -1) {
                        bestGoalIndex_ = static_cast<int>(nodes_.size());
                        nodes_.push_back(goal);
                        parent_.push_back(newIdx);
                        cost_.push_back(goalCostViaNew);
                        children_.push_back({});
                        children_[newIdx].push_back(bestGoalIndex_);
                    } else {
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

                    // Update ellipse parameters now that we have a (better) solution
                    updateEllipse();

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

Vec2 InformedRRTStar::sampleEllipse(std::mt19937& rng) const {
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    // Rejection sampling from unit disk
    double ux, uy;
    do {
        ux = dist(rng);
        uy = dist(rng);
    } while (ux * ux + uy * uy > 1.0);

    // Scale to ellipse dimensions
    double sx = ellipse_.majorAxis * ux;
    double sy = ellipse_.minorAxis * uy;

    // Rotate by the angle of start→goal axis
    double cosR = std::cos(ellipse_.rotation);
    double sinR = std::sin(ellipse_.rotation);
    double rx = cosR * sx - sinR * sy;
    double ry = sinR * sx + cosR * sy;

    // Translate to ellipse center
    return Vec2(ellipse_.center.x + rx, ellipse_.center.y + ry);
}

void InformedRRTStar::updateEllipse() {
    ellipse_.active = true;
    ellipse_.majorAxis = bestCost_ / 2.0;
    double cminHalf = cmin_ / 2.0;
    double majorSq = ellipse_.majorAxis * ellipse_.majorAxis;
    double cminHalfSq = cminHalf * cminHalf;
    ellipse_.minorAxis = std::sqrt(std::max(0.0, majorSq - cminHalfSq));
}

int InformedRRTStar::nearestNode(const Vec2& point) const {
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

Vec2 InformedRRTStar::steer(const Vec2& from, const Vec2& to) const {
    Vec2 direction = to - from;
    double dist = direction.length();

    if (dist <= config_.stepSize) {
        return to;
    }

    return from + direction.normalized() * config_.stepSize;
}

std::vector<int> InformedRRTStar::nearNodes(const Vec2& point, double radius) const {
    std::vector<int> result;
    double radiusSq = radius * radius;

    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        if ((nodes_[i] - point).lengthSquared() <= radiusSq) {
            result.push_back(i);
        }
    }

    return result;
}

double InformedRRTStar::nearRadius() const {
    int n = static_cast<int>(nodes_.size());
    if (n <= 1) return config_.stepSize;

    double r = config_.gamma * std::sqrt(std::log(static_cast<double>(n)) / n);
    return std::min(r, config_.stepSize);
}

double InformedRRTStar::edgeCost(const Vec2& a, const Vec2& b) const {
    return a.distanceTo(b);
}

void InformedRRTStar::propagateCostUpdate(int nodeIndex) {
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

Path InformedRRTStar::reconstructPath(int goalIndex) const {
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

void InformedRRTStar::rebuildEdges() {
    edges_.clear();
    for (int i = 1; i < static_cast<int>(nodes_.size()); ++i) {
        if (parent_[i] != -1) {
            edges_.push_back({parent_[i], i});
        }
    }
}

}  // namespace motion_planning
