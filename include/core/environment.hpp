#pragma once

#include "types.hpp"
#include <vector>
#include <random>

namespace motion_planning {

class Environment {
public:
    Environment(double width, double height);

    // Workspace bounds
    double width() const { return width_; }
    double height() const { return height_; }
    AABB bounds() const { return AABB({0, 0}, {width_, height_}); }

    // Obstacle management
    void addRectangle(const Vec2& position, double width, double height);
    void addCircle(const Vec2& center, double radius);
    void clearObstacles();
    bool removeObstacleAt(const Vec2& point);

    const std::vector<Rectangle>& getRectangles() const { return rectangles_; }
    const std::vector<Circle>& getCircles() const { return circles_; }

    // Collision detection
    bool isColliding(const Vec2& point) const;
    bool isSegmentColliding(const Vec2& from, const Vec2& to) const;
    bool isInBounds(const Vec2& point) const;
    bool isFree(const Vec2& point) const;

    // Random sampling
    Vec2 sampleRandom(std::mt19937& rng) const;

    // Start/Goal
    void setStart(const Vec2& start) { start_ = start; hasStart_ = true; }
    void setGoal(const Vec2& goal) { goal_ = goal; hasGoal_ = true; }
    Vec2 getStart() const { return start_; }
    Vec2 getGoal() const { return goal_; }
    bool hasStart() const { return hasStart_; }
    bool hasGoal() const { return hasGoal_; }

private:
    // Line-segment vs rectangle intersection
    bool segmentIntersectsRect(const Vec2& p1, const Vec2& p2, const Rectangle& rect) const;
    // Line-segment vs circle intersection
    bool segmentIntersectsCircle(const Vec2& p1, const Vec2& p2, const Circle& circle) const;

    double width_;
    double height_;

    std::vector<Rectangle> rectangles_;
    std::vector<Circle> circles_;

    Vec2 start_;
    Vec2 goal_;
    bool hasStart_ = false;
    bool hasGoal_ = false;
};

}  // namespace motion_planning
