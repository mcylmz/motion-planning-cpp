/**
 * D* Lite (Dynamic Replanning) Demo
 *
 * Demonstrates the D* Lite incremental search algorithm. The robot moves along
 * its planned path while the user can dynamically add/remove obstacles. The
 * algorithm efficiently replans by updating only affected nodes rather than
 * searching from scratch.
 *
 * Controls:
 *   Space       - Start planning / Toggle auto-move
 *   N           - Step robot one cell (manual mode)
 *   Left Click  - Add obstacle (triggers replan)
 *   Right Click - Remove obstacle (triggers replan)
 *   R           - Reset everything
 *   Esc         - Quit
 */

#include "core/grid.hpp"
#include "visualization/visualizer.hpp"
#include "algorithms/d_star_lite.hpp"
#include <iostream>
#include <iomanip>

using namespace motion_planning;

enum class DemoState {
    Idle,
    Planning,
    Moving,
    GoalReached
};

int main() {
    const int ROWS = 50;
    const int COLS = 50;
    Grid grid(ROWS, COLS, 1.0);

    // Create maze-like environment
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

    std::cout << "=== D* Lite (Dynamic Replanning) Demo ===" << std::endl;
    std::cout << "Grid size: " << ROWS << "x" << COLS << std::endl;
    std::cout << "Start: (" << start.row << ", " << start.col << ")" << std::endl;
    std::cout << "Goal: (" << goal.row << ", " << goal.col << ")" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Space       - Start planning / Toggle auto-move" << std::endl;
    std::cout << "  N           - Step robot one cell (manual mode)" << std::endl;
    std::cout << "  Left Click  - Add obstacle (triggers replan)" << std::endl;
    std::cout << "  Right Click - Remove obstacle (triggers replan)" << std::endl;
    std::cout << "  R           - Reset everything" << std::endl;
    std::cout << "  Esc         - Quit" << std::endl;
    std::cout << std::endl;

    Visualizer::Config config;
    config.windowWidth = 800;
    config.windowHeight = 850;
    config.title = "D* Lite (Dynamic Replanning) Visualization";

    Visualizer viz(config);
    viz.setGrid(&grid);
    viz.setAnimationDelay(2);

    DStarLite dstar;
    DemoState state = DemoState::Idle;
    bool autoMove = false;
    GridPath currentPath;
    int totalReplans = 0;
    int totalSteps = 0;

    sf::Clock moveClock;
    const float moveInterval = 0.2f;  // seconds between auto-moves

    auto replan = [&](const std::vector<Cell>& changedCells) {
        if (state != DemoState::Moving) return;

        grid.resetVisualization();
        grid.setStart(start);
        grid.setGoal(goal);

        dstar.updateObstacles(changedCells);

        auto replanStart = std::chrono::high_resolution_clock::now();
        bool found = dstar.computeShortestPath();
        auto replanEnd = std::chrono::high_resolution_clock::now();
        double replanTime = std::chrono::duration<double, std::milli>(replanEnd - replanStart).count();

        totalReplans++;

        if (found) {
            currentPath = dstar.getCurrentPath();
            std::cout << "Replan #" << totalReplans
                      << ": " << changedCells.size() << " cells changed, "
                      << "new path length: " << std::fixed << std::setprecision(2)
                      << currentPath.cost
                      << ", time: " << std::setprecision(3) << replanTime << "ms"
                      << std::endl;
        } else {
            currentPath.found = false;
            currentPath.cells.clear();
            std::cout << "Replan #" << totalReplans
                      << ": No valid path! (" << changedCells.size() << " cells changed)"
                      << std::endl;
        }
    };

    // Click callback: add/remove obstacles dynamically
    viz.setClickCallback([&](const Vec2& worldPos, sf::Mouse::Button button) {
        Cell cell = grid.worldToCell(worldPos);
        if (!grid.isValid(cell)) return;
        if (cell == start || cell == goal) return;
        if (cell == dstar.getRobotPosition()) return;

        if (button == sf::Mouse::Button::Left) {
            if (!grid.isObstacle(cell)) {
                grid.setObstacle(cell.row, cell.col, true);
                replan({cell});
            }
        } else if (button == sf::Mouse::Button::Right) {
            if (grid.isObstacle(cell)) {
                grid.setObstacle(cell.row, cell.col, false);
                replan({cell});
            }
        }
    });

    // Key callback
    viz.setKeyCallback([&](sf::Keyboard::Key key) {
        if (key == sf::Keyboard::Key::Escape) {
            viz.close();
            return;
        }

        if (key == sf::Keyboard::Key::R) {
            // Full reset
            grid.resetVisualization();
            grid.setStart(start);
            grid.setGoal(goal);
            state = DemoState::Idle;
            autoMove = false;
            currentPath = GridPath();
            totalReplans = 0;
            totalSteps = 0;
            std::cout << "Reset." << std::endl;
            return;
        }

        if (key == sf::Keyboard::Key::Space) {
            if (state == DemoState::Idle) {
                // Start initial planning
                state = DemoState::Planning;
                grid.resetVisualization();
                grid.setStart(start);
                grid.setGoal(goal);

                std::cout << "Planning initial path..." << std::endl;

                dstar.initialize(grid, start, goal);

                auto callback = [&](const Cell& cell, CellState cellState) {
                    grid.setState(cell, cellState);

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

                auto planStart = std::chrono::high_resolution_clock::now();
                bool found = dstar.computeShortestPath(callback);
                auto planEnd = std::chrono::high_resolution_clock::now();
                double planTime = std::chrono::duration<double, std::milli>(planEnd - planStart).count();

                if (found) {
                    currentPath = dstar.getCurrentPath();
                    std::cout << "Initial path found! Length: " << std::fixed
                              << std::setprecision(2) << currentPath.cost
                              << ", Nodes expanded: " << dstar.getStats().nodesExpanded
                              << ", Time: " << std::setprecision(3) << planTime << "ms"
                              << std::endl;
                    std::cout << "Press SPACE for auto-move, N to step manually." << std::endl;

                    state = DemoState::Moving;
                    autoMove = false;
                    grid.resetVisualization();
                    grid.setStart(start);
                    grid.setGoal(goal);
                } else {
                    std::cout << "No path found!" << std::endl;
                    state = DemoState::Idle;
                }
            } else if (state == DemoState::Moving) {
                // Toggle auto-move
                autoMove = !autoMove;
                moveClock.restart();
                std::cout << (autoMove ? "Auto-move ON" : "Auto-move OFF") << std::endl;
            } else if (state == DemoState::GoalReached) {
                // Return to idle
                grid.resetVisualization();
                grid.setStart(start);
                grid.setGoal(goal);
                state = DemoState::Idle;
                autoMove = false;
                currentPath = GridPath();
                totalReplans = 0;
                totalSteps = 0;
            }
            return;
        }

        if (key == sf::Keyboard::Key::N && state == DemoState::Moving) {
            // Manual step
            if (!dstar.isAtGoal() && dstar.hasValidPath()) {
                dstar.moveRobot();
                totalSteps++;
                currentPath = dstar.getCurrentPath();

                if (dstar.isAtGoal()) {
                    state = DemoState::GoalReached;
                    std::cout << "Goal reached! Total steps: " << totalSteps
                              << ", Total replans: " << totalReplans << std::endl;
                    std::cout << "Press SPACE to restart." << std::endl;
                }
            }
            return;
        }
    });

    // Main loop
    while (viz.isOpen()) {
        viz.handleEvents();

        // Auto-move logic
        if (state == DemoState::Moving && autoMove &&
            !dstar.isAtGoal() && dstar.hasValidPath() &&
            moveClock.getElapsedTime().asSeconds() >= moveInterval) {

            dstar.moveRobot();
            totalSteps++;
            currentPath = dstar.getCurrentPath();
            moveClock.restart();

            if (dstar.isAtGoal()) {
                state = DemoState::GoalReached;
                autoMove = false;
                std::cout << "Goal reached! Total steps: " << totalSteps
                          << ", Total replans: " << totalReplans << std::endl;
                std::cout << "Press SPACE to restart." << std::endl;
            }
        }

        // Render
        viz.clear();
        viz.drawGrid();

        if (state == DemoState::Moving || state == DemoState::GoalReached) {
            // Draw current path
            if (currentPath.found && !currentPath.cells.empty()) {
                viz.drawPath(currentPath.cells, colors::PATH);
            }

            // Draw robot position
            Cell robotCell = dstar.getRobotPosition();
            Vec2 robotWorld = grid.cellToWorld(robotCell);
            viz.drawPoint(robotWorld, colors::RRT_NEW, 8.0f);

            // Draw stats
            SearchStats displayStats;
            displayStats.nodesExpanded = dstar.getStats().nodesExpanded;
            displayStats.pathLength = currentPath.cost;
            displayStats.searchTime = dstar.getStats().searchTime;
            viz.drawStats(displayStats);
        }

        viz.display();
    }

    return 0;
}
