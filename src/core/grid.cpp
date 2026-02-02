#include "core/grid.hpp"
#include <cmath>

namespace motion_planning {

Grid::Grid(int rows, int cols, double cellSize)
    : rows_(rows), cols_(cols), cellSize_(cellSize) {
    grid_.resize(rows, std::vector<CellState>(cols, CellState::Free));
    obstacles_.resize(rows, std::vector<bool>(cols, false));
}

CellState Grid::getState(int row, int col) const {
    if (!isValid(row, col)) {
        throw std::out_of_range("Cell out of bounds");
    }
    return grid_[row][col];
}

CellState Grid::getState(const Cell& cell) const {
    return getState(cell.row, cell.col);
}

void Grid::setState(int row, int col, CellState state) {
    if (!isValid(row, col)) {
        throw std::out_of_range("Cell out of bounds");
    }
    grid_[row][col] = state;
}

void Grid::setState(const Cell& cell, CellState state) {
    setState(cell.row, cell.col, state);
}

bool Grid::isObstacle(int row, int col) const {
    if (!isValid(row, col)) return true;  // Out of bounds treated as obstacle
    return obstacles_[row][col];
}

bool Grid::isObstacle(const Cell& cell) const {
    return isObstacle(cell.row, cell.col);
}

bool Grid::isFree(int row, int col) const {
    return isValid(row, col) && !obstacles_[row][col];
}

bool Grid::isFree(const Cell& cell) const {
    return isFree(cell.row, cell.col);
}

void Grid::setObstacle(int row, int col, bool obstacle) {
    if (isValid(row, col)) {
        obstacles_[row][col] = obstacle;
        if (obstacle) {
            grid_[row][col] = CellState::Obstacle;
        } else {
            grid_[row][col] = CellState::Free;
        }
    }
}

void Grid::addRectangleObstacle(int startRow, int startCol, int height, int width) {
    for (int r = startRow; r < startRow + height && r < rows_; ++r) {
        for (int c = startCol; c < startCol + width && c < cols_; ++c) {
            if (isValid(r, c)) {
                setObstacle(r, c, true);
            }
        }
    }
}

void Grid::addCircleObstacle(int centerRow, int centerCol, int radius) {
    for (int r = centerRow - radius; r <= centerRow + radius; ++r) {
        for (int c = centerCol - radius; c <= centerCol + radius; ++c) {
            if (isValid(r, c)) {
                int dr = r - centerRow;
                int dc = c - centerCol;
                if (dr * dr + dc * dc <= radius * radius) {
                    setObstacle(r, c, true);
                }
            }
        }
    }
}

bool Grid::isValid(int row, int col) const {
    return row >= 0 && row < rows_ && col >= 0 && col < cols_;
}

bool Grid::isValid(const Cell& cell) const {
    return isValid(cell.row, cell.col);
}

Vec2 Grid::cellToWorld(int row, int col) const {
    return Vec2(col * cellSize_ + cellSize_ / 2.0,
                row * cellSize_ + cellSize_ / 2.0);
}

Vec2 Grid::cellToWorld(const Cell& cell) const {
    return cellToWorld(cell.row, cell.col);
}

Cell Grid::worldToCell(const Vec2& point) const {
    int col = static_cast<int>(point.x / cellSize_);
    int row = static_cast<int>(point.y / cellSize_);
    return Cell(row, col);
}

std::vector<Cell> Grid::getNeighbors4(const Cell& cell) const {
    std::vector<Cell> neighbors;
    neighbors.reserve(4);

    // Up, Down, Left, Right
    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = {0, 0, -1, 1};

    for (int i = 0; i < 4; ++i) {
        int nr = cell.row + dr[i];
        int nc = cell.col + dc[i];
        if (isFree(nr, nc)) {
            neighbors.emplace_back(nr, nc);
        }
    }

    return neighbors;
}

std::vector<Cell> Grid::getNeighbors8(const Cell& cell) const {
    std::vector<Cell> neighbors;
    neighbors.reserve(8);

    // All 8 directions
    const int dr[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dc[] = {-1, 0, 1, -1, 1, -1, 0, 1};

    for (int i = 0; i < 8; ++i) {
        int nr = cell.row + dr[i];
        int nc = cell.col + dc[i];
        if (isFree(nr, nc)) {
            // For diagonal moves, check that we're not cutting corners
            if (dr[i] != 0 && dc[i] != 0) {
                if (!isFree(cell.row + dr[i], cell.col) ||
                    !isFree(cell.row, cell.col + dc[i])) {
                    continue;  // Can't cut corners
                }
            }
            neighbors.emplace_back(nr, nc);
        }
    }

    return neighbors;
}

double Grid::movementCost(const Cell& from, const Cell& to) {
    int dr = std::abs(from.row - to.row);
    int dc = std::abs(from.col - to.col);

    // Diagonal movement costs sqrt(2), orthogonal costs 1
    if (dr == 1 && dc == 1) {
        return std::sqrt(2.0);
    }
    return 1.0;
}

void Grid::resetVisualization() {
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            if (obstacles_[r][c]) {
                grid_[r][c] = CellState::Obstacle;
            } else {
                grid_[r][c] = CellState::Free;
            }
        }
    }

    if (hasStart_) {
        grid_[start_.row][start_.col] = CellState::Start;
    }
    if (hasGoal_) {
        grid_[goal_.row][goal_.col] = CellState::Goal;
    }
}

void Grid::clear() {
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            grid_[r][c] = CellState::Free;
            obstacles_[r][c] = false;
        }
    }
    hasStart_ = false;
    hasGoal_ = false;
}

void Grid::setStart(const Cell& cell) {
    if (isValid(cell) && !isObstacle(cell)) {
        if (hasStart_) {
            grid_[start_.row][start_.col] = CellState::Free;
        }
        start_ = cell;
        hasStart_ = true;
        grid_[cell.row][cell.col] = CellState::Start;
    }
}

void Grid::setGoal(const Cell& cell) {
    if (isValid(cell) && !isObstacle(cell)) {
        if (hasGoal_) {
            grid_[goal_.row][goal_.col] = CellState::Free;
        }
        goal_ = cell;
        hasGoal_ = true;
        grid_[cell.row][cell.col] = CellState::Goal;
    }
}

}  // namespace motion_planning
