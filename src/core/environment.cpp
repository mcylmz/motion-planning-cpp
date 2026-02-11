#include "core/environment.hpp"
#include <algorithm>

namespace motion_planning {

Environment::Environment(double width, double height)
    : width_(width), height_(height) {}

void Environment::addRectangle(const Vec2& position, double width, double height) {
    rectangles_.emplace_back(position, width, height);
}

void Environment::addCircle(const Vec2& center, double radius) {
    circles_.emplace_back(center, radius);
}

void Environment::clearObstacles() {
    rectangles_.clear();
    circles_.clear();
}

bool Environment::removeObstacleAt(const Vec2& point) {
    // Check circles first (smaller, easier to click)
    for (auto it = circles_.begin(); it != circles_.end(); ++it) {
        if (it->contains(point)) {
            circles_.erase(it);
            return true;
        }
    }
    // Then rectangles
    for (auto it = rectangles_.begin(); it != rectangles_.end(); ++it) {
        if (it->contains(point)) {
            rectangles_.erase(it);
            return true;
        }
    }
    return false;
}

bool Environment::isColliding(const Vec2& point) const {
    for (const auto& rect : rectangles_) {
        if (rect.contains(point)) return true;
    }
    for (const auto& circle : circles_) {
        if (circle.contains(point)) return true;
    }
    return false;
}

bool Environment::isInBounds(const Vec2& point) const {
    return point.x >= 0 && point.x <= width_ &&
           point.y >= 0 && point.y <= height_;
}

bool Environment::isFree(const Vec2& point) const {
    return isInBounds(point) && !isColliding(point);
}

bool Environment::isSegmentColliding(const Vec2& from, const Vec2& to) const {
    for (const auto& rect : rectangles_) {
        if (segmentIntersectsRect(from, to, rect)) return true;
    }
    for (const auto& circle : circles_) {
        if (segmentIntersectsCircle(from, to, circle)) return true;
    }
    return false;
}

Vec2 Environment::sampleRandom(std::mt19937& rng) const {
    std::uniform_real_distribution<double> distX(0, width_);
    std::uniform_real_distribution<double> distY(0, height_);
    return Vec2(distX(rng), distY(rng));
}

bool Environment::segmentIntersectsRect(const Vec2& p1, const Vec2& p2,
                                         const Rectangle& rect) const {
    // Check if either endpoint is inside the rectangle
    if (rect.contains(p1) || rect.contains(p2)) return true;

    // Rectangle corners
    Vec2 topLeft = rect.position;
    Vec2 topRight = {rect.position.x + rect.width, rect.position.y};
    Vec2 bottomLeft = {rect.position.x, rect.position.y + rect.height};
    Vec2 bottomRight = {rect.position.x + rect.width, rect.position.y + rect.height};

    // Check segment against all 4 edges of the rectangle
    auto segmentsIntersect = [](const Vec2& a, const Vec2& b,
                                const Vec2& c, const Vec2& d) -> bool {
        Vec2 ab = b - a;
        Vec2 ac = c - a;
        Vec2 ad = d - a;

        double crossC = ab.cross(ac);
        double crossD = ab.cross(ad);
        if (crossC * crossD > 0) return false;  // c and d on same side

        Vec2 cd = d - c;
        Vec2 ca = a - c;
        Vec2 cb = b - c;

        double crossA = cd.cross(ca);
        double crossB = cd.cross(cb);
        if (crossA * crossB > 0) return false;  // a and b on same side

        return true;
    };

    if (segmentsIntersect(p1, p2, topLeft, topRight)) return true;
    if (segmentsIntersect(p1, p2, topRight, bottomRight)) return true;
    if (segmentsIntersect(p1, p2, bottomRight, bottomLeft)) return true;
    if (segmentsIntersect(p1, p2, bottomLeft, topLeft)) return true;

    return false;
}

bool Environment::segmentIntersectsCircle(const Vec2& p1, const Vec2& p2,
                                           const Circle& circle) const {
    // Check if either endpoint is inside the circle
    if (circle.contains(p1) || circle.contains(p2)) return true;

    // Find closest point on segment to circle center
    Vec2 d = p2 - p1;
    Vec2 f = p1 - circle.center;

    double a = d.dot(d);
    double b = 2.0 * f.dot(d);
    double c = f.dot(f) - circle.radius * circle.radius;

    double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0) return false;

    discriminant = std::sqrt(discriminant);
    double t1 = (-b - discriminant) / (2.0 * a);
    double t2 = (-b + discriminant) / (2.0 * a);

    // Check if intersection is within the segment [0, 1]
    if (t1 >= 0.0 && t1 <= 1.0) return true;
    if (t2 >= 0.0 && t2 <= 1.0) return true;

    return false;
}

}  // namespace motion_planning
