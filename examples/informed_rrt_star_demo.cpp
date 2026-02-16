/**
 * Informed RRT* Demo
 *
 * Demonstrates the Informed RRT* algorithm (Gammell et al., 2014) in a
 * continuous 2D environment with geometric obstacles. Informed RRT* extends
 * RRT* by restricting sampling to an ellipsoidal subset once the first
 * solution is found. The ellipse has start and goal as foci, with transverse
 * diameter equal to the current best cost. This dramatically accelerates
 * convergence to the optimal path.
 *
 * The demo visualizes the sampling ellipse shrinking in real-time as the
 * path cost improves.
 *
 * Controls:
 *   Space - Run/Re-run Informed RRT*
 *   R     - Reset tree (keep obstacles)
 *   C     - Clear all obstacles
 *   S     - Set start mode (click to place)
 *   G     - Set goal mode (click to place)
 *   D     - Draw rectangle mode (click to place)
 *   O     - Draw circle mode (click to place)
 *   T     - Toggle continue-after-goal mode
 *   N     - Normal mode (no drawing)
 *   Right Click - Remove obstacle under cursor
 *   +/-   - Adjust obstacle size / step size (depends on mode)
 *   Esc   - Quit
 */

#include "core/environment.hpp"
#include "visualization/visualizer.hpp"
#include "algorithms/informed_rrt_star.hpp"
#include <iostream>
#include <iomanip>

using namespace motion_planning;

int main() {
    // Create continuous environment (500 x 500 units)
    const double ENV_WIDTH = 500.0;
    const double ENV_HEIGHT = 500.0;
    Environment env(ENV_WIDTH, ENV_HEIGHT);

    // Add default obstacles
    env.addRectangle({100, 50}, 40, 200);
    env.addRectangle({250, 200}, 40, 250);
    env.addRectangle({100, 350}, 200, 40);
    env.addCircle({350, 100}, 50);
    env.addCircle({150, 150}, 30);
    env.addRectangle({350, 300}, 100, 40);

    // Set start and goal
    Vec2 start(30, 30);
    Vec2 goal(470, 470);
    env.setStart(start);
    env.setGoal(goal);

    // Create visualizer
    Visualizer::Config vizConfig;
    vizConfig.windowWidth = 800;
    vizConfig.windowHeight = 800;
    vizConfig.title = "Informed RRT* - Ellipsoidal Sampling";
    vizConfig.showGridLines = false;

    Visualizer viz(vizConfig);
    viz.setWorldBounds(ENV_WIDTH, ENV_HEIGHT);
    viz.setAnimationDelay(2);

    // Informed RRT* config
    InformedRRTStar::Config config;
    config.maxIterations = 5000;
    config.stepSize = 15.0;
    config.goalThreshold = 20.0;
    config.goalBias = 0.05;
    config.gamma = 50.0;
    config.continueAfterGoal = true;
    config.seed = 42;

    InformedRRTStar informedRRTStar(config);

    // State
    bool hasResult = false;
    bool isRunning = false;
    Path resultPath;
    SearchStats stats;
    unsigned int currentSeed = 42;

    enum class Mode {
        Normal,
        SetStart,
        SetGoal,
        DrawRect,
        DrawCircle
    };
    Mode mode = Mode::Normal;

    // Obstacle placement sizes
    double rectWidth = 40.0;
    double rectHeight = 40.0;
    double circleRadius = 25.0;

    // Ghost preview position (tracks mouse)
    Vec2 mousePos(0, 0);
    bool showPreview = false;

    auto printMode = [](Mode m) {
        switch (m) {
            case Mode::Normal:     std::cout << "Mode: Normal" << std::endl; break;
            case Mode::SetStart:   std::cout << "Mode: Set Start (click)" << std::endl; break;
            case Mode::SetGoal:    std::cout << "Mode: Set Goal (click)" << std::endl; break;
            case Mode::DrawRect:   std::cout << "Mode: Draw Rectangle (click to place, +/- to resize)" << std::endl; break;
            case Mode::DrawCircle: std::cout << "Mode: Draw Circle (click to place, +/- to resize)" << std::endl; break;
        }
    };

    std::cout << "=== Informed RRT* Demo ===" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Space       - Run Informed RRT*" << std::endl;
    std::cout << "  R           - Reset tree" << std::endl;
    std::cout << "  C           - Clear all obstacles" << std::endl;
    std::cout << "  S           - Set start mode" << std::endl;
    std::cout << "  G           - Set goal mode" << std::endl;
    std::cout << "  D           - Draw rectangle mode" << std::endl;
    std::cout << "  O           - Draw circle mode" << std::endl;
    std::cout << "  T           - Toggle continue-after-goal" << std::endl;
    std::cout << "  N           - Normal mode (no drawing)" << std::endl;
    std::cout << "  Right Click - Remove obstacle" << std::endl;
    std::cout << "  +/-         - Adjust size (obstacle or step)" << std::endl;
    std::cout << "  Esc         - Quit" << std::endl;
    std::cout << std::endl;
    std::cout << "Continue after goal: " << (config.continueAfterGoal ? "ON" : "OFF") << std::endl;
    std::cout << std::endl;

    // Click callback
    viz.setClickCallback([&](const Vec2& worldPos, sf::Mouse::Button button) {
        if (isRunning) return;

        Vec2 envPos(worldPos.x, worldPos.y);
        if (!env.isInBounds(envPos)) return;

        if (button == sf::Mouse::Button::Right) {
            if (env.removeObstacleAt(envPos)) {
                hasResult = false;
                std::cout << "Obstacle removed" << std::endl;
            }
            return;
        }

        if (button != sf::Mouse::Button::Left) return;

        switch (mode) {
            case Mode::SetStart:
                if (env.isFree(envPos)) {
                    start = envPos;
                    env.setStart(start);
                    hasResult = false;
                    std::cout << "Start: (" << (int)start.x << ", " << (int)start.y << ")" << std::endl;
                }
                break;

            case Mode::SetGoal:
                if (env.isFree(envPos)) {
                    goal = envPos;
                    env.setGoal(goal);
                    hasResult = false;
                    std::cout << "Goal: (" << (int)goal.x << ", " << (int)goal.y << ")" << std::endl;
                }
                break;

            case Mode::DrawRect: {
                Vec2 pos(envPos.x - rectWidth / 2.0, envPos.y - rectHeight / 2.0);
                env.addRectangle(pos, rectWidth, rectHeight);
                hasResult = false;
                std::cout << "Rectangle added at (" << (int)envPos.x << ", " << (int)envPos.y
                          << ") size " << (int)rectWidth << "x" << (int)rectHeight << std::endl;
                break;
            }

            case Mode::DrawCircle:
                env.addCircle(envPos, circleRadius);
                hasResult = false;
                std::cout << "Circle added at (" << (int)envPos.x << ", " << (int)envPos.y
                          << ") radius " << (int)circleRadius << std::endl;
                break;

            case Mode::Normal:
                break;
        }
    });

    // Key callback
    viz.setKeyCallback([&](sf::Keyboard::Key key) {
        if (isRunning && key != sf::Keyboard::Key::Escape) return;

        switch (key) {
            case sf::Keyboard::Key::Escape:
                viz.close();
                break;

            case sf::Keyboard::Key::N:
                mode = Mode::Normal;
                printMode(mode);
                break;

            case sf::Keyboard::Key::S:
                mode = Mode::SetStart;
                printMode(mode);
                break;

            case sf::Keyboard::Key::G:
                mode = Mode::SetGoal;
                printMode(mode);
                break;

            case sf::Keyboard::Key::D:
                mode = Mode::DrawRect;
                printMode(mode);
                break;

            case sf::Keyboard::Key::O:
                mode = Mode::DrawCircle;
                printMode(mode);
                break;

            case sf::Keyboard::Key::T:
                config.continueAfterGoal = !config.continueAfterGoal;
                informedRRTStar.setConfig(config);
                std::cout << "Continue after goal: "
                          << (config.continueAfterGoal ? "ON" : "OFF") << std::endl;
                break;

            case sf::Keyboard::Key::R:
                hasResult = false;
                std::cout << "Tree reset" << std::endl;
                break;

            case sf::Keyboard::Key::C:
                env.clearObstacles();
                hasResult = false;
                std::cout << "All obstacles cleared" << std::endl;
                break;

            case sf::Keyboard::Key::Equal: // + key
                if (mode == Mode::DrawRect) {
                    rectWidth += 5.0;
                    rectHeight += 5.0;
                    std::cout << "Rect size: " << (int)rectWidth << "x" << (int)rectHeight << std::endl;
                } else if (mode == Mode::DrawCircle) {
                    circleRadius += 5.0;
                    std::cout << "Circle radius: " << (int)circleRadius << std::endl;
                } else {
                    config.stepSize += 2.0;
                    informedRRTStar.setConfig(config);
                    std::cout << "Step size: " << config.stepSize << std::endl;
                }
                break;

            case sf::Keyboard::Key::Hyphen: // - key
                if (mode == Mode::DrawRect) {
                    rectWidth = std::max(10.0, rectWidth - 5.0);
                    rectHeight = std::max(10.0, rectHeight - 5.0);
                    std::cout << "Rect size: " << (int)rectWidth << "x" << (int)rectHeight << std::endl;
                } else if (mode == Mode::DrawCircle) {
                    circleRadius = std::max(5.0, circleRadius - 5.0);
                    std::cout << "Circle radius: " << (int)circleRadius << std::endl;
                } else {
                    config.stepSize = std::max(2.0, config.stepSize - 2.0);
                    informedRRTStar.setConfig(config);
                    std::cout << "Step size: " << config.stepSize << std::endl;
                }
                break;

            case sf::Keyboard::Key::Space: {
                hasResult = false;
                isRunning = true;

                config.seed = currentSeed++;
                informedRRTStar.setConfig(config);

                auto vizCallback = [&](const TreeEdge& edge) {
                    viz.clear();

                    // Draw obstacles
                    for (const auto& rect : env.getRectangles()) {
                        viz.drawRectangle(rect.position, rect.width, rect.height,
                                         colors::OBSTACLE, true);
                    }
                    for (const auto& circle : env.getCircles()) {
                        viz.drawCircle(circle.center, circle.radius,
                                      colors::OBSTACLE, true);
                    }

                    // Draw all tree edges
                    for (const auto& [parentIdx, childIdx] : informedRRTStar.getEdges()) {
                        viz.drawLine(informedRRTStar.getNodes()[parentIdx],
                                    informedRRTStar.getNodes()[childIdx],
                                    colors::TREE_EDGE, 1.0f);
                    }

                    // Highlight newest/rewired edge in amber
                    viz.drawLine(edge.from, edge.to, colors::REWIRE, 2.0f);

                    // Draw sampling ellipse ON TOP of tree edges
                    const auto& ep = informedRRTStar.getEllipseParams();
                    if (ep.active) {
                        viz.drawEllipse(ep.center, ep.majorAxis, ep.minorAxis,
                                       ep.rotation, colors::ELLIPSE_FILL, true);
                        viz.drawEllipse(ep.center, ep.majorAxis, ep.minorAxis,
                                       ep.rotation, colors::ELLIPSE_OUTLINE, false);
                    }

                    // Draw start and goal
                    viz.drawPoint(start, colors::START, 8.0f);
                    viz.drawPoint(goal, colors::GOAL, 8.0f);

                    viz.display();
                    viz.delay();

                    while (auto event = viz.pollEvent()) {
                        if (event->is<sf::Event::Closed>()) {
                            viz.close();
                        }
                    }
                };

                resultPath = informedRRTStar.plan(env, start, goal, vizCallback);
                stats = informedRRTStar.getStats();
                hasResult = true;
                isRunning = false;

                if (resultPath.found) {
                    const auto& ep = informedRRTStar.getEllipseParams();
                    std::cout << "Path found! Length: " << std::fixed
                              << std::setprecision(1) << resultPath.totalLength
                              << ", Nodes: " << informedRRTStar.getNodes().size()
                              << ", Time: " << std::setprecision(2)
                              << stats.searchTime << "ms"
                              << ", Best cost: " << std::setprecision(1)
                              << informedRRTStar.getBestCost() << std::endl;
                    std::cout << "Ellipse: active=" << ep.active
                              << ", center=(" << (int)ep.center.x << "," << (int)ep.center.y << ")"
                              << ", major=" << std::setprecision(1) << ep.majorAxis
                              << ", minor=" << ep.minorAxis
                              << ", rotation=" << std::setprecision(3) << ep.rotation
                              << " rad" << std::endl;
                } else {
                    std::cout << "No path found after " << config.maxIterations
                              << " iterations (" << informedRRTStar.getNodes().size()
                              << " nodes)" << std::endl;
                }
                break;
            }

            default:
                break;
        }
    });

    // Main loop
    while (viz.isOpen()) {
        viz.handleEvents();

        // Track mouse for ghost preview
        auto mouseScreenPos = sf::Mouse::getPosition(viz.getWindow());
        mousePos = viz.screenToWorld(mouseScreenPos.x, mouseScreenPos.y);
        showPreview = env.isInBounds(mousePos) &&
                      (mode == Mode::DrawRect || mode == Mode::DrawCircle);

        viz.clear();

        // Draw obstacles
        for (const auto& rect : env.getRectangles()) {
            viz.drawRectangle(rect.position, rect.width, rect.height,
                             colors::OBSTACLE, true);
        }
        for (const auto& circle : env.getCircles()) {
            viz.drawCircle(circle.center, circle.radius,
                          colors::OBSTACLE, true);
        }

        // Draw ghost preview of obstacle to place
        if (showPreview) {
            sf::Color previewColor(100, 100, 100, 120);
            if (mode == Mode::DrawRect) {
                Vec2 previewPos(mousePos.x - rectWidth / 2.0, mousePos.y - rectHeight / 2.0);
                viz.drawRectangle(previewPos, rectWidth, rectHeight, previewColor, true);
            } else if (mode == Mode::DrawCircle) {
                viz.drawCircle(mousePos, circleRadius, previewColor, true);
            }
        }

        // Draw tree and ellipse if we have results
        if (hasResult) {
            for (const auto& [parentIdx, childIdx] : informedRRTStar.getEdges()) {
                viz.drawLine(informedRRTStar.getNodes()[parentIdx],
                            informedRRTStar.getNodes()[childIdx],
                            colors::TREE_EDGE, 1.0f);
            }

            // Draw sampling ellipse ON TOP of tree edges
            const auto& ep = informedRRTStar.getEllipseParams();
            if (ep.active) {
                viz.drawEllipse(ep.center, ep.majorAxis, ep.minorAxis,
                               ep.rotation, colors::ELLIPSE_FILL, true);
                viz.drawEllipse(ep.center, ep.majorAxis, ep.minorAxis,
                               ep.rotation, colors::ELLIPSE_OUTLINE, false);
            }

            if (resultPath.found) {
                viz.drawPath(resultPath.points, colors::PATH_LINE);
            }

            viz.drawStats(stats);
        }

        // Draw start and goal
        viz.drawPoint(start, colors::START, 8.0f);
        viz.drawPoint(goal, colors::GOAL, 8.0f);

        viz.display();
    }

    return 0;
}
