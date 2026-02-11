#include "algorithms/prm.hpp"
#include <chrono>
#include <algorithm>
#include <queue>
#include <unordered_map>

namespace motion_planning {

Path PRM::plan(const Environment& env, const Vec2& start, const Vec2& goal) {
    return plan(env, start, goal, nullptr);
}

Path PRM::plan(const Environment& env, const Vec2& start, const Vec2& goal,
               TreeVisualizationCallback callback) {
    auto startTime = std::chrono::high_resolution_clock::now();

    stats_ = SearchStats();
    Path result;
    result.found = false;

    // Phase 1: Build roadmap
    buildRoadmap(env, callback);

    // Phase 2: Connect start and goal to roadmap, then query
    int startIdx = connectToRoadmap(env, start, callback);
    int goalIdx = connectToRoadmap(env, goal, callback);

    if (startIdx < 0 || goalIdx < 0) {
        auto endTime = std::chrono::high_resolution_clock::now();
        stats_.searchTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        return result;
    }

    // Phase 3: A* query
    result = queryPath(startIdx, goalIdx);

    auto endTime = std::chrono::high_resolution_clock::now();
    stats_.searchTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    stats_.pathLength = result.totalLength;

    return result;
}

void PRM::buildRoadmap(const Environment& env, TreeVisualizationCallback callback) {
    nodes_.clear();
    adjacency_.clear();
    edges_.clear();

    std::mt19937 rng(config_.seed);

    // Sample collision-free points
    for (int i = 0; i < config_.numSamples; ++i) {
        Vec2 sample = env.sampleRandom(rng);
        if (!env.isFree(sample)) continue;

        int idx = static_cast<int>(nodes_.size());
        nodes_.push_back(sample);
        adjacency_.emplace_back();
        stats_.nodesVisited++;

        if (callback) {
            callback({sample, sample});  // Point event (from == to)
        }
    }

    // Connect k-nearest neighbors
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        auto neighbors = kNearestNeighbors(i, config_.k);

        for (int neighborIdx : neighbors) {
            // Check if already connected
            bool alreadyConnected = false;
            for (int adj : adjacency_[i]) {
                if (adj == neighborIdx) {
                    alreadyConnected = true;
                    break;
                }
            }
            if (alreadyConnected) continue;

            // Check collision-free segment
            if (!env.isSegmentColliding(nodes_[i], nodes_[neighborIdx])) {
                adjacency_[i].push_back(neighborIdx);
                adjacency_[neighborIdx].push_back(i);
                edges_.push_back({i, neighborIdx});
                stats_.nodesExpanded++;

                if (callback) {
                    callback({nodes_[i], nodes_[neighborIdx]});
                }
            }
        }
    }
}

int PRM::connectToRoadmap(const Environment& env, const Vec2& point,
                           TreeVisualizationCallback callback) {
    // Add the point as a new node
    int pointIdx = static_cast<int>(nodes_.size());
    nodes_.push_back(point);
    adjacency_.emplace_back();

    // Find k nearest neighbors and connect
    auto neighbors = kNearestNeighbors(pointIdx, config_.k);

    for (int neighborIdx : neighbors) {
        if (!env.isSegmentColliding(point, nodes_[neighborIdx])) {
            adjacency_[pointIdx].push_back(neighborIdx);
            adjacency_[neighborIdx].push_back(pointIdx);
            edges_.push_back({pointIdx, neighborIdx});

            if (callback) {
                callback({point, nodes_[neighborIdx]});
            }
        }
    }

    if (adjacency_[pointIdx].empty()) {
        return -1;  // Could not connect to roadmap
    }

    return pointIdx;
}

std::vector<int> PRM::kNearestNeighbors(int nodeIdx, int k) const {
    // Collect distances to all other nodes
    std::vector<std::pair<double, int>> distances;
    distances.reserve(nodes_.size());

    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        if (i == nodeIdx) continue;
        double dist = nodes_[nodeIdx].distanceTo(nodes_[i]);
        distances.push_back({dist, i});
    }

    // Partial sort to get k nearest
    int count = std::min(k, static_cast<int>(distances.size()));
    std::partial_sort(distances.begin(), distances.begin() + count, distances.end());

    std::vector<int> result;
    result.reserve(count);
    for (int i = 0; i < count; ++i) {
        result.push_back(distances[i].second);
    }

    return result;
}

Path PRM::queryPath(int startIdx, int goalIdx) {
    // A* search on adjacency list
    using PQEntry = std::pair<double, int>;  // (f_cost, node_index)
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> openSet;

    std::vector<double> gCost(nodes_.size(), std::numeric_limits<double>::infinity());
    std::vector<int> cameFrom(nodes_.size(), -1);
    std::vector<bool> closed(nodes_.size(), false);

    gCost[startIdx] = 0.0;
    double heuristic = nodes_[startIdx].distanceTo(nodes_[goalIdx]);
    openSet.push({heuristic, startIdx});

    while (!openSet.empty()) {
        auto [fCost, current] = openSet.top();
        openSet.pop();

        if (current == goalIdx) {
            return reconstructPath(cameFrom, startIdx, goalIdx);
        }

        if (closed[current]) continue;
        closed[current] = true;

        for (int neighbor : adjacency_[current]) {
            if (closed[neighbor]) continue;

            double edgeCost = nodes_[current].distanceTo(nodes_[neighbor]);
            double tentativeG = gCost[current] + edgeCost;

            if (tentativeG < gCost[neighbor]) {
                gCost[neighbor] = tentativeG;
                cameFrom[neighbor] = current;
                double h = nodes_[neighbor].distanceTo(nodes_[goalIdx]);
                openSet.push({tentativeG + h, neighbor});
            }
        }
    }

    // No path found
    Path result;
    result.found = false;
    return result;
}

Path PRM::reconstructPath(const std::vector<int>& cameFrom, int startIdx, int goalIdx) {
    Path path;
    path.found = true;

    std::vector<Vec2> points;
    int current = goalIdx;
    while (current != -1) {
        points.push_back(nodes_[current]);
        if (current == startIdx) break;
        current = cameFrom[current];
    }

    std::reverse(points.begin(), points.end());
    path.points = std::move(points);
    path.computeLength();

    return path;
}

}  // namespace motion_planning
