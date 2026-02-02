#include "core/grid.hpp"
#include "visualization/visualizer.hpp"
#include "algorithms/bfs.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/astar.hpp"
#include <iostream>
#include <memory>

using namespace motion_planning;

enum class Mode {
    SetStart,
    SetGoal,
    DrawObstacle,
    EraseObstacle
};

enum class Algorithm {
    BFS,
    Dijkstra,
    AStar
};

int main() {
    // Create grid
    const int GRID_ROWS = 40;
    const int GRID_COLS = 40;
    const double CELL_SIZE = 1.0;

    Grid grid(GRID_ROWS, GRID_COLS, CELL_SIZE);

    // Add some default obstacles
    grid.addRectangleObstacle(10, 10, 15, 3);
    grid.addRectangleObstacle(5, 20, 3, 15);
    grid.addCircleObstacle(30, 30, 5);
    grid.addRectangleObstacle(20, 5, 3, 10);

    // Set default start and goal
    grid.setStart(Cell(5, 5));
    grid.setGoal(Cell(35, 35));

    // Create visualizer
    Visualizer::Config vizConfig;
    vizConfig.windowWidth = 800;
    vizConfig.windowHeight = 850;
    vizConfig.title = "Motion Planning - Grid Search";
    vizConfig.showGridLines = true;

    Visualizer viz(vizConfig);
    viz.setGrid(&grid);
    viz.setAnimationDelay(5);

    // State
    Mode mode = Mode::DrawObstacle;
    Algorithm currentAlgo = Algorithm::AStar;
    bool isSearching = false;
    bool pathFound = false;
    SearchStats lastStats;

    // Algorithms
    BFS bfs(true);
    Dijkstra dijkstra(true);
    AStar astar(true);

    std::cout << "=== Motion Planning Visualizer ===" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Left Click  - Draw obstacle / Set start/goal" << std::endl;
    std::cout << "  Right Click - Erase obstacle" << std::endl;
    std::cout << "  S - Set Start mode" << std::endl;
    std::cout << "  G - Set Goal mode" << std::endl;
    std::cout << "  D - Draw Obstacle mode" << std::endl;
    std::cout << "  1 - BFS algorithm" << std::endl;
    std::cout << "  2 - Dijkstra algorithm" << std::endl;
    std::cout << "  3 - A* algorithm" << std::endl;
    std::cout << "  Space - Run search" << std::endl;
    std::cout << "  R - Reset visualization" << std::endl;
    std::cout << "  C - Clear all" << std::endl;
    std::cout << "  Esc - Quit" << std::endl;

    // Click callback
    viz.setClickCallback([&](const Vec2& worldPos, sf::Mouse::Button button) {
        if (isSearching) return;

        Cell cell = grid.worldToCell(worldPos);
        if (!grid.isValid(cell)) return;

        if (button == sf::Mouse::Button::Left) {
            switch (mode) {
                case Mode::SetStart:
                    grid.resetVisualization();
                    grid.setStart(cell);
                    pathFound = false;
                    break;
                case Mode::SetGoal:
                    grid.resetVisualization();
                    grid.setGoal(cell);
                    pathFound = false;
                    break;
                case Mode::DrawObstacle:
                    if (!(cell == grid.getStart()) && !(cell == grid.getGoal())) {
                        grid.setObstacle(cell.row, cell.col, true);
                        pathFound = false;
                    }
                    break;
                case Mode::EraseObstacle:
                    grid.setObstacle(cell.row, cell.col, false);
                    pathFound = false;
                    break;
            }
        } else if (button == sf::Mouse::Button::Right) {
            grid.setObstacle(cell.row, cell.col, false);
            pathFound = false;
        }
    });

    // Key callback
    viz.setKeyCallback([&](sf::Keyboard::Key key) {
        if (isSearching && key != sf::Keyboard::Key::Escape) return;

        switch (key) {
            case sf::Keyboard::Key::Escape:
                viz.close();
                break;
            case sf::Keyboard::Key::S:
                mode = Mode::SetStart;
                std::cout << "Mode: Set Start" << std::endl;
                break;
            case sf::Keyboard::Key::G:
                mode = Mode::SetGoal;
                std::cout << "Mode: Set Goal" << std::endl;
                break;
            case sf::Keyboard::Key::D:
                mode = Mode::DrawObstacle;
                std::cout << "Mode: Draw Obstacle" << std::endl;
                break;
            case sf::Keyboard::Key::Num1:
                currentAlgo = Algorithm::BFS;
                std::cout << "Algorithm: BFS" << std::endl;
                break;
            case sf::Keyboard::Key::Num2:
                currentAlgo = Algorithm::Dijkstra;
                std::cout << "Algorithm: Dijkstra" << std::endl;
                break;
            case sf::Keyboard::Key::Num3:
                currentAlgo = Algorithm::AStar;
                std::cout << "Algorithm: A*" << std::endl;
                break;
            case sf::Keyboard::Key::R:
                grid.resetVisualization();
                pathFound = false;
                std::cout << "Visualization reset" << std::endl;
                break;
            case sf::Keyboard::Key::C:
                grid.clear();
                pathFound = false;
                std::cout << "Grid cleared" << std::endl;
                break;
            case sf::Keyboard::Key::Space:
                if (grid.hasStart() && grid.hasGoal()) {
                    grid.resetVisualization();
                    isSearching = true;
                    pathFound = false;

                    auto vizCallback = [&](const Cell& cell, CellState state) {
                        grid.setState(cell, state);

                        static int counter = 0;
                        if (++counter % 5 == 0) {
                            viz.clear();
                            viz.drawGrid();
                            viz.display();
                            viz.delay();
                        }

                        while (auto event = viz.pollEvent()) {
                            if (event->is<sf::Event::Closed>()) {
                                viz.close();
                            }
                        }
                    };

                    GridPath path;
                    GridSearchAlgorithm* algo = nullptr;

                    switch (currentAlgo) {
                        case Algorithm::BFS:
                            algo = &bfs;
                            break;
                        case Algorithm::Dijkstra:
                            algo = &dijkstra;
                            break;
                        case Algorithm::AStar:
                            algo = &astar;
                            break;
                    }

                    path = algo->search(grid, grid.getStart(), grid.getGoal(), vizCallback);
                    lastStats = algo->getStats();

                    if (path.found) {
                        viz.drawPath(path.cells);
                        pathFound = true;
                        std::cout << "Path found! Length: " << path.cost
                                  << ", Nodes expanded: " << lastStats.nodesExpanded
                                  << ", Time: " << lastStats.searchTime << "ms" << std::endl;
                    } else {
                        std::cout << "No path found!" << std::endl;
                    }

                    isSearching = false;
                }
                break;
            default:
                break;
        }
    });

    // Main loop
    while (viz.isOpen()) {
        viz.handleEvents();

        viz.clear();
        viz.drawGrid();

        if (pathFound) {
            viz.drawStats(lastStats);
        }

        viz.display();
    }

    return 0;
}
