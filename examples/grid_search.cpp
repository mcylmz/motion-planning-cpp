/**
 * Grid Search Example
 *
 * Demonstrates BFS, Dijkstra, and A* algorithms on a grid world.
 */

#include "core/grid.hpp"
#include "visualization/visualizer.hpp"
#include "algorithms/bfs.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/astar.hpp"
#include <iostream>
#include <iomanip>

using namespace motion_planning;

void printStats(const std::string& name, const SearchStats& stats, bool found) {
    std::cout << std::setw(10) << name << " | "
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

    std::cout << "=== Grid Search Algorithm Comparison ===" << std::endl;
    std::cout << "Grid size: " << ROWS << "x" << COLS << std::endl;
    std::cout << "Start: (" << start.row << ", " << start.col << ")" << std::endl;
    std::cout << "Goal: (" << goal.row << ", " << goal.col << ")" << std::endl;
    std::cout << std::endl;

    std::cout << std::setw(10) << "Algorithm" << " | "
              << std::setw(8) << "Found" << " | "
              << std::setw(8) << "Expanded" << " | "
              << std::setw(8) << "Length" << " | "
              << std::setw(12) << "Time"
              << std::endl;
    std::cout << std::string(55, '-') << std::endl;

    // BFS
    BFS bfs(true);
    auto bfsPath = bfs.search(grid, start, goal);
    printStats("BFS", bfs.getStats(), bfsPath.found);
    grid.resetVisualization();

    // Dijkstra
    Dijkstra dijkstra(true);
    auto dijkstraPath = dijkstra.search(grid, start, goal);
    printStats("Dijkstra", dijkstra.getStats(), dijkstraPath.found);
    grid.resetVisualization();

    // A* with different heuristics
    AStar astarOctile(true, heuristics::octile);
    auto astarPath = astarOctile.search(grid, start, goal);
    printStats("A* Octile", astarOctile.getStats(), astarPath.found);
    grid.resetVisualization();

    AStar astarEuclid(true, heuristics::euclidean);
    auto astarEuclidPath = astarEuclid.search(grid, start, goal);
    printStats("A* Euclid", astarEuclid.getStats(), astarEuclidPath.found);
    grid.resetVisualization();

    std::cout << std::endl;
    std::cout << "Launching visualization with A*..." << std::endl;
    std::cout << "Press SPACE to start, R to reset, ESC to quit." << std::endl;

    Visualizer::Config config;
    config.windowWidth = 800;
    config.windowHeight = 850;
    config.title = "Grid Search - A* Visualization";

    Visualizer viz(config);
    viz.setGrid(&grid);
    viz.setAnimationDelay(2);

    bool searchStarted = false;
    bool searchComplete = false;
    SearchStats stats;

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

            AStar astar(true, heuristics::octile);
            auto path = astar.search(grid, start, goal, callback);
            stats = astar.getStats();

            if (path.found) {
                viz.drawPath(path.cells);
            }

            searchComplete = true;
        } else if (key == sf::Keyboard::Key::R) {
            grid.resetVisualization();
            searchStarted = false;
            searchComplete = false;
        }
    });

    while (viz.isOpen()) {
        viz.handleEvents();
        viz.clear();
        viz.drawGrid();

        if (searchComplete) {
            viz.drawStats(stats);
        }

        viz.display();
    }

    return 0;
}
