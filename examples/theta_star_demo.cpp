/**
 * Theta* (Any-Angle Planning) Demo
 *
 * Demonstrates Theta* algorithm which produces shorter, smoother paths
 * compared to A* by allowing any-angle parent connections via line-of-sight checks.
 */

#include "core/grid.hpp"
#include "visualization/visualizer.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/theta_star.hpp"
#include <iostream>
#include <iomanip>

using namespace motion_planning;

void printStats(const std::string& name, const SearchStats& stats, bool found) {
    std::cout << std::setw(12) << name << " | "
              << std::setw(8) << (found ? "Yes" : "No") << " | "
              << std::setw(8) << stats.nodesExpanded << " | "
              << std::setw(8) << std::fixed << std::setprecision(2) << stats.pathLength << " | "
              << std::setw(8) << std::setprecision(3) << stats.searchTime << " ms"
              << std::endl;
}

int main() {
    const int ROWS = 50;
    const int COLS = 50;
    Grid grid(ROWS, COLS, 1.0);

    // Create maze-like environment (same as grid_search.cpp)
    grid.addRectangleObstacle(5, 15, 30, 2);
    grid.addRectangleObstacle(15, 30, 25, 2);
    grid.addRectangleObstacle(25, 5, 2, 20);
    grid.addRectangleObstacle(35, 20, 2, 25);
    grid.addCircleObstacle(10, 40, 4);
    grid.addCircleObstacle(40, 10, 5);
    grid.addRectangleObstacle(8, 8, 3, 3);
    grid.addRectangleObstacle(42, 42, 4, 4);

    Cell start(2, 2);
    Cell goal(47, 47);
    grid.setStart(start);
    grid.setGoal(goal);

    std::cout << "=== Theta* vs A* Comparison ===" << std::endl;
    std::cout << "Grid size: " << ROWS << "x" << COLS << std::endl;
    std::cout << "Start: (" << start.row << ", " << start.col << ")" << std::endl;
    std::cout << "Goal: (" << goal.row << ", " << goal.col << ")" << std::endl;
    std::cout << std::endl;

    std::cout << std::setw(12) << "Algorithm" << " | "
              << std::setw(8) << "Found" << " | "
              << std::setw(8) << "Expanded" << " | "
              << std::setw(8) << "Length" << " | "
              << std::setw(12) << "Time"
              << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    // A* (8-connected with octile heuristic)
    AStar astar(true, heuristics::octile);
    auto astarPath = astar.search(grid, start, goal);
    printStats("A* Octile", astar.getStats(), astarPath.found);
    grid.resetVisualization();

    // Theta*
    ThetaStar thetaStar;
    auto thetaPath = thetaStar.search(grid, start, goal);
    printStats("Theta*", thetaStar.getStats(), thetaPath.found);
    grid.resetVisualization();

    if (astarPath.found && thetaPath.found) {
        double improvement = (1.0 - thetaPath.cost / astarPath.cost) * 100.0;
        std::cout << std::endl;
        std::cout << "Theta* path is " << std::fixed << std::setprecision(2)
                  << improvement << "% shorter than A*" << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Launching visualization with Theta*..." << std::endl;
    std::cout << "Press SPACE to start, R to reset, ESC to quit." << std::endl;

    Visualizer::Config config;
    config.windowWidth = 800;
    config.windowHeight = 850;
    config.title = "Theta* (Any-Angle Planning) Visualization";

    Visualizer viz(config);
    viz.setGrid(&grid);
    viz.setAnimationDelay(2);

    bool searchStarted = false;
    bool searchComplete = false;
    SearchStats stats;
    std::vector<Vec2> anyAnglePath;

    viz.setKeyCallback([&](sf::Keyboard::Key key) {
        if (key == sf::Keyboard::Key::Escape) {
            viz.close();
        } else if (key == sf::Keyboard::Key::Space && !searchStarted) {
            searchStarted = true;

            auto callback = [&](const Cell& cell, CellState state) {
                grid.setState(cell, state);

                static int counter = 0;
                if (++counter % 3 == 0) {
                    viz.clear();
                    viz.drawGrid();
                    viz.display();
                    viz.delay();

                    while (auto event = viz.pollEvent()) {
                        if (event->is<sf::Event::Closed>()) {
                            viz.close();
                        }
                    }
                }
            };

            ThetaStar theta;
            auto path = theta.search(grid, start, goal, callback);
            stats = theta.getStats();

            if (path.found) {
                // Convert path cells to world coordinates for any-angle line drawing
                anyAnglePath.clear();
                for (const Cell& cell : path.cells) {
                    anyAnglePath.push_back(grid.cellToWorld(cell));
                }
            }

            searchComplete = true;
        } else if (key == sf::Keyboard::Key::R) {
            grid.resetVisualization();
            searchStarted = false;
            searchComplete = false;
            anyAnglePath.clear();
        }
    });

    while (viz.isOpen()) {
        viz.handleEvents();
        viz.clear();
        viz.drawGrid();

        if (searchComplete && !anyAnglePath.empty()) {
            viz.drawPath(anyAnglePath, colors::PATH_LINE);
            viz.drawStats(stats);
        }

        viz.display();
    }

    return 0;
}
