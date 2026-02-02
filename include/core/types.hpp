#pragma once

#include <cmath>
#include <vector>
#include <limits>
#include <functional>

namespace motion_planning {

// 2D Point/Vector
struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    Vec2() = default;
    Vec2(double x, double y) : x(x), y(y) {}

    Vec2 operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }
    Vec2 operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }
    Vec2 operator*(double scalar) const { return {x * scalar, y * scalar}; }
    Vec2 operator/(double scalar) const { return {x / scalar, y / scalar}; }

    Vec2& operator+=(const Vec2& other) { x += other.x; y += other.y; return *this; }
    Vec2& operator-=(const Vec2& other) { x -= other.x; y -= other.y; return *this; }

    bool operator==(const Vec2& other) const {
        return std::abs(x - other.x) < 1e-9 && std::abs(y - other.y) < 1e-9;
    }

    double dot(const Vec2& other) const { return x * other.x + y * other.y; }
    double cross(const Vec2& other) const { return x * other.y - y * other.x; }
    double length() const { return std::sqrt(x * x + y * y); }
    double lengthSquared() const { return x * x + y * y; }

    Vec2 normalized() const {
        double len = length();
        if (len < 1e-9) return {0, 0};
        return {x / len, y / len};
    }

    double distanceTo(const Vec2& other) const {
        return (*this - other).length();
    }
};

// Grid cell position
struct Cell {
    int row = 0;
    int col = 0;

    Cell() = default;
    Cell(int row, int col) : row(row), col(col) {}

    bool operator==(const Cell& other) const {
        return row == other.row && col == other.col;
    }

    bool operator!=(const Cell& other) const {
        return !(*this == other);
    }
};

// Hash function for Cell (for use in unordered containers)
struct CellHash {
    std::size_t operator()(const Cell& cell) const {
        return std::hash<int>()(cell.row) ^ (std::hash<int>()(cell.col) << 16);
    }
};

// Axis-Aligned Bounding Box
struct AABB {
    Vec2 min;
    Vec2 max;

    AABB() = default;
    AABB(const Vec2& min, const Vec2& max) : min(min), max(max) {}

    bool contains(const Vec2& point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y;
    }

    bool intersects(const AABB& other) const {
        return !(other.max.x < min.x || other.min.x > max.x ||
                 other.max.y < min.y || other.min.y > max.y);
    }

    Vec2 center() const {
        return (min + max) / 2.0;
    }

    double width() const { return max.x - min.x; }
    double height() const { return max.y - min.y; }
};

// Rectangle obstacle
struct Rectangle {
    Vec2 position;  // top-left corner
    double width;
    double height;

    Rectangle() : width(0), height(0) {}
    Rectangle(const Vec2& pos, double w, double h)
        : position(pos), width(w), height(h) {}

    AABB toAABB() const {
        return AABB(position, {position.x + width, position.y + height});
    }

    bool contains(const Vec2& point) const {
        return point.x >= position.x && point.x <= position.x + width &&
               point.y >= position.y && point.y <= position.y + height;
    }
};

// Circle obstacle
struct Circle {
    Vec2 center;
    double radius;

    Circle() : radius(0) {}
    Circle(const Vec2& c, double r) : center(c), radius(r) {}

    bool contains(const Vec2& point) const {
        return center.distanceTo(point) <= radius;
    }
};

// Path result
struct Path {
    std::vector<Vec2> points;
    double totalLength = 0.0;
    bool found = false;

    void computeLength() {
        totalLength = 0.0;
        for (size_t i = 1; i < points.size(); ++i) {
            totalLength += points[i - 1].distanceTo(points[i]);
        }
    }
};

// Grid-based path
struct GridPath {
    std::vector<Cell> cells;
    double cost = 0.0;
    bool found = false;
};

// Search statistics
struct SearchStats {
    int nodesExpanded = 0;
    int nodesVisited = 0;
    double searchTime = 0.0;  // milliseconds
    double pathLength = 0.0;
};

}  // namespace motion_planning
