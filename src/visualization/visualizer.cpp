#include "visualization/visualizer.hpp"
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>

namespace motion_planning {

Visualizer::Visualizer(const Config& config) : config_(config) {
    window_.create(
        sf::VideoMode({config.windowWidth, config.windowHeight}),
        config.title,
        sf::Style::Titlebar | sf::Style::Close
    );
    window_.setFramerateLimit(config.frameRateLimit);

    // Try to load a system font
    std::vector<std::string> fontPaths = {
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/consola.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/System/Library/Fonts/Helvetica.ttc"
    };

    for (const auto& path : fontPaths) {
        if (font_.openFromFile(path)) {
            fontLoaded_ = true;
            break;
        }
    }
}

bool Visualizer::isOpen() const {
    return window_.isOpen();
}

void Visualizer::close() {
    window_.close();
}

void Visualizer::clear() {
    window_.clear(colors::BACKGROUND);
}

void Visualizer::display() {
    window_.display();
}

std::optional<sf::Event> Visualizer::pollEvent() {
    return window_.pollEvent();
}

void Visualizer::setGrid(Grid* grid) {
    grid_ = grid;
    if (grid_) {
        double scaleX = static_cast<double>(config_.windowWidth) / (grid_->cols() * grid_->cellSize());
        double scaleY = static_cast<double>(config_.windowHeight) / (grid_->rows() * grid_->cellSize());
        scale_ = std::min(scaleX, scaleY) * 0.95;

        double gridWidth = grid_->cols() * grid_->cellSize() * scale_;
        double gridHeight = grid_->rows() * grid_->cellSize() * scale_;
        offset_.x = (config_.windowWidth - gridWidth) / 2.0;
        offset_.y = (config_.windowHeight - gridHeight) / 2.0;
    }
}

sf::Color Visualizer::getCellColor(CellState state) const {
    switch (state) {
        case CellState::Free:     return colors::FREE;
        case CellState::Obstacle: return colors::OBSTACLE;
        case CellState::Start:    return colors::START;
        case CellState::Goal:     return colors::GOAL;
        case CellState::Visited:  return colors::VISITED;
        case CellState::Frontier: return colors::FRONTIER;
        case CellState::Path:     return colors::PATH;
        default:                  return colors::FREE;
    }
}

void Visualizer::drawGrid() {
    if (!grid_) return;

    float cellScreenSize = static_cast<float>(grid_->cellSize() * scale_);

    sf::RectangleShape cell({cellScreenSize - 1, cellScreenSize - 1});

    for (int r = 0; r < grid_->rows(); ++r) {
        for (int c = 0; c < grid_->cols(); ++c) {
            cell.setPosition({
                static_cast<float>(offset_.x + c * cellScreenSize),
                static_cast<float>(offset_.y + r * cellScreenSize)
            });
            cell.setFillColor(getCellColor(grid_->getState(r, c)));
            window_.draw(cell);
        }
    }

    if (config_.showGridLines) {
        drawGridLines();
    }
}

void Visualizer::drawGridLines() {
    if (!grid_) return;

    float cellScreenSize = static_cast<float>(grid_->cellSize() * scale_);

    sf::VertexArray lines(sf::PrimitiveType::Lines);

    for (int c = 0; c <= grid_->cols(); ++c) {
        float x = static_cast<float>(offset_.x + c * cellScreenSize);
        lines.append(sf::Vertex{{x, static_cast<float>(offset_.y)}, colors::GRID_LINE});
        lines.append(sf::Vertex{{x, static_cast<float>(offset_.y + grid_->rows() * cellScreenSize)}, colors::GRID_LINE});
    }

    for (int r = 0; r <= grid_->rows(); ++r) {
        float y = static_cast<float>(offset_.y + r * cellScreenSize);
        lines.append(sf::Vertex{{static_cast<float>(offset_.x), y}, colors::GRID_LINE});
        lines.append(sf::Vertex{{static_cast<float>(offset_.x + grid_->cols() * cellScreenSize), y}, colors::GRID_LINE});
    }

    window_.draw(lines);
}

void Visualizer::updateCell(const Cell& cell, CellState state) {
    if (grid_ && grid_->isValid(cell)) {
        grid_->setState(cell, state);
    }
}

void Visualizer::drawPath(const std::vector<Cell>& path, sf::Color color) {
    if (!grid_ || path.empty()) return;

    for (const auto& cell : path) {
        if (grid_->isValid(cell)) {
            grid_->setState(cell, CellState::Path);
        }
    }

    if (path.size() >= 2) {
        std::vector<Vec2> worldPath;
        for (const auto& cell : path) {
            worldPath.push_back(grid_->cellToWorld(cell));
        }
        drawPath(worldPath, color);
    }
}

void Visualizer::drawPath(const std::vector<Vec2>& path, sf::Color color) {
    if (path.size() < 2) return;

    for (size_t i = 1; i < path.size(); ++i) {
        drawLine(path[i - 1], path[i], color, 3.0f);
    }
}

void Visualizer::drawLine(const Vec2& from, const Vec2& to, sf::Color color, float thickness) {
    sf::Vector2f screenFrom = worldToScreen(from);
    sf::Vector2f screenTo = worldToScreen(to);

    sf::Vector2f direction = screenTo - screenFrom;
    float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);

    if (length < 0.001f) return;

    sf::RectangleShape line({length, thickness});
    line.setPosition(screenFrom);
    line.setFillColor(color);

    float angle = std::atan2(direction.y, direction.x) * 180.0f / 3.14159f;
    line.setRotation(sf::degrees(angle));

    window_.draw(line);
}

void Visualizer::drawPoint(const Vec2& point, sf::Color color, float radius) {
    sf::CircleShape circle(radius);
    sf::Vector2f screenPos = worldToScreen(point);
    circle.setPosition({screenPos.x - radius, screenPos.y - radius});
    circle.setFillColor(color);
    window_.draw(circle);
}

void Visualizer::drawCircle(const Vec2& center, double radius, sf::Color color, bool filled) {
    sf::CircleShape circle(static_cast<float>(radius * scale_));
    sf::Vector2f screenPos = worldToScreen(center);
    circle.setPosition({screenPos.x - static_cast<float>(radius * scale_),
                        screenPos.y - static_cast<float>(radius * scale_)});

    if (filled) {
        circle.setFillColor(color);
    } else {
        circle.setFillColor(sf::Color::Transparent);
        circle.setOutlineColor(color);
        circle.setOutlineThickness(2.0f);
    }

    window_.draw(circle);
}

void Visualizer::drawRectangle(const Vec2& position, double width, double height, sf::Color color, bool filled) {
    sf::RectangleShape rect({
        static_cast<float>(width * scale_),
        static_cast<float>(height * scale_)
    });

    sf::Vector2f screenPos = worldToScreen(position);
    rect.setPosition(screenPos);

    if (filled) {
        rect.setFillColor(color);
    } else {
        rect.setFillColor(sf::Color::Transparent);
        rect.setOutlineColor(color);
        rect.setOutlineThickness(2.0f);
    }

    window_.draw(rect);
}

void Visualizer::drawText(const std::string& text, const Vec2& position, unsigned int size, sf::Color color) {
    if (!fontLoaded_) return;

    sf::Text sfText(font_, text, size);
    sfText.setFillColor(color);

    sf::Vector2f screenPos = worldToScreen(position);
    sfText.setPosition(screenPos);

    window_.draw(sfText);
}

void Visualizer::drawStats(const SearchStats& stats) {
    if (!fontLoaded_) return;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Nodes Expanded: " << stats.nodesExpanded << "\n";
    ss << "Nodes Visited: " << stats.nodesVisited << "\n";
    ss << "Search Time: " << stats.searchTime << " ms\n";
    ss << "Path Length: " << stats.pathLength;

    sf::Text text(font_, ss.str(), 14);
    text.setFillColor(colors::TEXT);
    text.setPosition({10.0f, 10.0f});

    sf::RectangleShape bg({200.0f, 80.0f});
    bg.setPosition({5.0f, 5.0f});
    bg.setFillColor(sf::Color(255, 255, 255, 200));
    window_.draw(bg);

    window_.draw(text);
}

Vec2 Visualizer::screenToWorld(int screenX, int screenY) const {
    double worldX = (screenX - offset_.x) / scale_;
    double worldY = (screenY - offset_.y) / scale_;
    return Vec2(worldX, worldY);
}

sf::Vector2f Visualizer::worldToScreen(const Vec2& world) const {
    return sf::Vector2f(
        static_cast<float>(world.x * scale_ + offset_.x),
        static_cast<float>(world.y * scale_ + offset_.y)
    );
}

void Visualizer::handleEvents() {
    while (auto event = pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            close();
        }
        else if (const auto* mouseEvent = event->getIf<sf::Event::MouseButtonPressed>()) {
            if (clickCallback_) {
                Vec2 worldPos = screenToWorld(mouseEvent->position.x, mouseEvent->position.y);
                clickCallback_(worldPos, mouseEvent->button);
            }
        }
        else if (const auto* keyEvent = event->getIf<sf::Event::KeyPressed>()) {
            if (keyCallback_) {
                keyCallback_(keyEvent->code);
            }
        }
    }
}

void Visualizer::delay() const {
    std::this_thread::sleep_for(std::chrono::milliseconds(animationDelay_));
}

}  // namespace motion_planning
