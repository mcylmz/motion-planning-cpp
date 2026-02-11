#pragma once

#include "core/types.hpp"
#include "core/grid.hpp"
#include "colors.hpp"
#include <SFML/Graphics.hpp>
#include <string>
#include <functional>
#include <memory>
#include <optional>

namespace motion_planning {

class Visualizer {
public:
    struct Config {
        unsigned int windowWidth = 800;
        unsigned int windowHeight = 800;
        std::string title = "Motion Planning Visualizer";
        bool showGridLines = true;
        bool showCoordinates = false;
        unsigned int frameRateLimit = 60;
    };

    Visualizer(const Config& config = Config());
    ~Visualizer() = default;

    // Window management
    bool isOpen() const;
    void close();
    void clear();
    void display();
    std::optional<sf::Event> pollEvent();

    // World bounds (for continuous space algorithms like RRT)
    void setWorldBounds(double worldWidth, double worldHeight);

    // Grid visualization
    void setGrid(Grid* grid);
    void drawGrid();
    void updateCell(const Cell& cell, CellState state);

    // Path visualization
    void drawPath(const std::vector<Cell>& path, sf::Color color = colors::PATH);
    void drawPath(const std::vector<Vec2>& path, sf::Color color = colors::PATH_LINE);

    // Tree/Graph visualization (for RRT, PRM, etc.)
    void drawLine(const Vec2& from, const Vec2& to, sf::Color color = colors::TREE_EDGE, float thickness = 1.0f);
    void drawPoint(const Vec2& point, sf::Color color = colors::TREE_NODE, float radius = 3.0f);
    void drawCircle(const Vec2& center, double radius, sf::Color color, bool filled = false);
    void drawRectangle(const Vec2& position, double width, double height, sf::Color color, bool filled = true);

    // Text rendering
    void drawText(const std::string& text, const Vec2& position, unsigned int size = 14, sf::Color color = colors::TEXT);
    void drawStats(const SearchStats& stats);

    // Coordinate conversion
    Vec2 screenToWorld(int screenX, int screenY) const;
    sf::Vector2f worldToScreen(const Vec2& world) const;

    // Interactive callbacks
    using ClickCallback = std::function<void(const Vec2& worldPos, sf::Mouse::Button button)>;
    using KeyCallback = std::function<void(sf::Keyboard::Key key)>;

    void setClickCallback(ClickCallback callback) { clickCallback_ = callback; }
    void setKeyCallback(KeyCallback callback) { keyCallback_ = callback; }

    // Process input events
    void handleEvents();

    // Animation control
    void setAnimationDelay(int milliseconds) { animationDelay_ = milliseconds; }
    void delay() const;

    // Get window reference for advanced usage
    sf::RenderWindow& getWindow() { return window_; }

private:
    sf::Color getCellColor(CellState state) const;
    void drawGridLines();

    sf::RenderWindow window_;
    Config config_;
    Grid* grid_ = nullptr;

    // For coordinate transformation
    double scale_ = 1.0;
    Vec2 offset_{0, 0};

    // Callbacks
    ClickCallback clickCallback_;
    KeyCallback keyCallback_;

    // Animation
    int animationDelay_ = 10;  // milliseconds

    // Font for text rendering
    sf::Font font_;
    bool fontLoaded_ = false;
};

}  // namespace motion_planning
