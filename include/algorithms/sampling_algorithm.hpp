#pragma once

#include "core/types.hpp"
#include "core/environment.hpp"
#include "visualization/visualizer.hpp"
#include <functional>

namespace motion_planning {

// Callback for visualizing tree growth
struct TreeEdge {
    Vec2 from;
    Vec2 to;
};

using TreeVisualizationCallback = std::function<void(const TreeEdge& edge)>;

class SamplingAlgorithm {
public:
    virtual ~SamplingAlgorithm() = default;

    // Main planning function
    virtual Path plan(const Environment& env, const Vec2& start, const Vec2& goal) = 0;

    // Planning with visualization callback
    virtual Path plan(const Environment& env, const Vec2& start, const Vec2& goal,
                     TreeVisualizationCallback callback) = 0;

    // Get search statistics
    const SearchStats& getStats() const { return stats_; }

    // Algorithm name
    virtual std::string name() const = 0;

protected:
    SearchStats stats_;
};

}  // namespace motion_planning
