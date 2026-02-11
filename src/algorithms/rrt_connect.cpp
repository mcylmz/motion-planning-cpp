#include "algorithms/rrt_connect.hpp"
#include <chrono>
#include <algorithm>

namespace motion_planning {

Path RRTConnect::plan(const Environment& env, const Vec2& start, const Vec2& goal) {
    return plan(env, start, goal, nullptr);
}

Path RRTConnect::plan(const Environment& env, const Vec2& start, const Vec2& goal,
                      TreeVisualizationCallback callback) {
    auto startTime = std::chrono::high_resolution_clock::now();

    stats_ = SearchStats();
    Path result;
    result.found = false;

    // Initialize tree A (start tree)
    nodesA_.clear();
    parentA_.clear();
    edgesA_.clear();
    nodesA_.push_back(start);
    parentA_.push_back(-1);

    // Initialize tree B (goal tree)
    nodesB_.clear();
    parentB_.clear();
    edgesB_.clear();
    nodesB_.push_back(goal);
    parentB_.push_back(-1);

    std::mt19937 rng(config_.seed);

    // Pointers to alternate which tree extends vs connects
    auto* extNodes = &nodesA_;
    auto* extParent = &parentA_;
    auto* extEdges = &edgesA_;
    auto* conNodes = &nodesB_;
    auto* conParent = &parentB_;
    auto* conEdges = &edgesB_;
    bool swapped = false;  // false = A extends, B connects

    for (int i = 0; i < config_.maxIterations; ++i) {
        stats_.nodesVisited++;

        // 1. Sample random point (no goal bias needed)
        Vec2 sample = env.sampleRandom(rng);

        // 2. Extend the extending tree toward sample
        int extIdx = -1;
        ExtendResult extResult = extend(env, *extNodes, *extParent, *extEdges,
                                        sample, extIdx, callback);

        if (extResult != ExtendResult::Trapped) {
            stats_.nodesExpanded++;

            // 3. Connect the other tree toward the new node
            const Vec2& newNode = (*extNodes)[extIdx];
            int conIdx = -1;
            ExtendResult conResult = connect(env, *conNodes, *conParent, *conEdges,
                                             newNode, conIdx, callback);

            if (conResult == ExtendResult::Reached) {
                // 4. Trees connected! Reconstruct path
                int startTreeIdx, goalTreeIdx;
                if (!swapped) {
                    // A extended, B connected
                    startTreeIdx = extIdx;
                    goalTreeIdx = conIdx;
                } else {
                    // B extended, A connected
                    startTreeIdx = conIdx;
                    goalTreeIdx = extIdx;
                }

                result = reconstructPath(startTreeIdx, goalTreeIdx);
                break;
            }

            if (conResult == ExtendResult::Advanced) {
                stats_.nodesExpanded++;
            }
        }

        // 5. Swap which tree extends vs connects
        std::swap(extNodes, conNodes);
        std::swap(extParent, conParent);
        std::swap(extEdges, conEdges);
        swapped = !swapped;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    stats_.searchTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    stats_.pathLength = result.totalLength;

    return result;
}

RRTConnect::ExtendResult RRTConnect::extend(
    const Environment& env,
    std::vector<Vec2>& nodes,
    std::vector<int>& parent,
    std::vector<std::pair<int, int>>& edges,
    const Vec2& target,
    int& resultIndex,
    TreeVisualizationCallback callback) {

    int nearestIdx = nearestNode(nodes, target);
    const Vec2& nearest = nodes[nearestIdx];

    Vec2 newPoint = steer(nearest, target);

    if (!env.isFree(newPoint)) return ExtendResult::Trapped;
    if (env.isSegmentColliding(nearest, newPoint)) return ExtendResult::Trapped;

    int newIdx = static_cast<int>(nodes.size());
    nodes.push_back(newPoint);
    parent.push_back(nearestIdx);
    edges.push_back({nearestIdx, newIdx});
    resultIndex = newIdx;

    if (callback) {
        callback({nearest, newPoint});
    }

    if (newPoint.distanceTo(target) < config_.connectThreshold) {
        return ExtendResult::Reached;
    }

    return ExtendResult::Advanced;
}

RRTConnect::ExtendResult RRTConnect::connect(
    const Environment& env,
    std::vector<Vec2>& nodes,
    std::vector<int>& parent,
    std::vector<std::pair<int, int>>& edges,
    const Vec2& target,
    int& resultIndex,
    TreeVisualizationCallback callback) {

    ExtendResult result = ExtendResult::Advanced;

    while (result == ExtendResult::Advanced) {
        result = extend(env, nodes, parent, edges, target, resultIndex, callback);
    }

    return result;
}

int RRTConnect::nearestNode(const std::vector<Vec2>& nodes, const Vec2& point) const {
    int nearest = 0;
    double minDist = nodes[0].distanceTo(point);

    for (int i = 1; i < static_cast<int>(nodes.size()); ++i) {
        double dist = nodes[i].distanceTo(point);
        if (dist < minDist) {
            minDist = dist;
            nearest = i;
        }
    }

    return nearest;
}

Vec2 RRTConnect::steer(const Vec2& from, const Vec2& to) const {
    Vec2 direction = to - from;
    double dist = direction.length();

    if (dist <= config_.stepSize) {
        return to;
    }

    return from + direction.normalized() * config_.stepSize;
}

Path RRTConnect::reconstructPath(int startTreeIdx, int goalTreeIdx) const {
    Path path;
    path.found = true;

    // Start chain: trace parentA_ from startTreeIdx back to root, then reverse
    std::vector<Vec2> startChain;
    int current = startTreeIdx;
    while (current != -1) {
        startChain.push_back(nodesA_[current]);
        current = parentA_[current];
    }
    std::reverse(startChain.begin(), startChain.end());

    // Goal chain: trace parentB_ from goalTreeIdx back to root (no reverse)
    std::vector<Vec2> goalChain;
    current = goalTreeIdx;
    while (current != -1) {
        goalChain.push_back(nodesB_[current]);
        current = parentB_[current];
    }

    // Concatenate: start→...→meetA + meetB→...→goal
    path.points = std::move(startChain);
    path.points.insert(path.points.end(), goalChain.begin(), goalChain.end());
    path.computeLength();

    return path;
}

}  // namespace motion_planning
