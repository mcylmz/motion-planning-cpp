#pragma once

#include "types.hpp"
#include <vector>
#include <stdexcept>

namespace motion_planning {

enum class CellState {
    Free = 0,
    Obstacle = 1,
    Start = 2,
    Goal = 3,
    Visited = 4,
    Path = 5,
    Frontier = 6
};

class Grid {
public:
    Grid(int rows, int cols, double cellSize = 1.0);

    // Accessors
    int rows() const { return rows_; }
    int cols() const { return cols_; }
    double cellSize() const { return cellSize_; }

    // Cell state management
    CellState getState(int row, int col) const;
    CellState getState(const Cell& cell) const;
    void setState(int row, int col, CellState state);
    void setState(const Cell& cell, CellState state);

    // Obstacle management
    bool isObstacle(int row, int col) const;
    bool isObstacle(const Cell& cell) const;
    bool isFree(int row, int col) const;
    bool isFree(const Cell& cell) const;
    void setObstacle(int row, int col, bool obstacle = true);
    void addRectangleObstacle(int startRow, int startCol, int height, int width);
    void addCircleObstacle(int centerRow, int centerCol, int radius);

    // Bounds checking
    bool isValid(int row, int col) const;
    bool isValid(const Cell& cell) const;

    // Coordinate conversions
    Vec2 cellToWorld(int row, int col) const;
    Vec2 cellToWorld(const Cell& cell) const;
    Cell worldToCell(const Vec2& point) const;

    // Neighbors (4-connected and 8-connected)
    std::vector<Cell> getNeighbors4(const Cell& cell) const;
    std::vector<Cell> getNeighbors8(const Cell& cell) const;

    // Movement costs
    static double movementCost(const Cell& from, const Cell& to);

    // Reset visualization states (keeps obstacles)
    void resetVisualization();

    // Clear everything
    void clear();

    // Start and goal
    void setStart(const Cell& cell);
    void setGoal(const Cell& cell);
    Cell getStart() const { return start_; }
    Cell getGoal() const { return goal_; }
    bool hasStart() const { return hasStart_; }
    bool hasGoal() const { return hasGoal_; }

private:
    int rows_;
    int cols_;
    double cellSize_;
    std::vector<std::vector<CellState>> grid_;
    std::vector<std::vector<bool>> obstacles_;

    Cell start_;
    Cell goal_;
    bool hasStart_ = false;
    bool hasGoal_ = false;
};

}  // namespace motion_planning
