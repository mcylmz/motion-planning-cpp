#pragma once

#include <SFML/Graphics/Color.hpp>

namespace motion_planning {
namespace colors {

// Grid cell colors
const sf::Color FREE        = sf::Color(240, 240, 240);     // Light gray
const sf::Color OBSTACLE    = sf::Color(40, 40, 40);        // Dark gray
const sf::Color START       = sf::Color(76, 175, 80);       // Green
const sf::Color GOAL        = sf::Color(244, 67, 54);       // Red
const sf::Color VISITED     = sf::Color(144, 202, 249);     // Light blue
const sf::Color FRONTIER    = sf::Color(255, 235, 59);      // Yellow
const sf::Color PATH        = sf::Color(156, 39, 176);      // Purple

// UI colors
const sf::Color GRID_LINE   = sf::Color(200, 200, 200);     // Grid lines
const sf::Color BACKGROUND  = sf::Color(255, 255, 255);     // White
const sf::Color TEXT        = sf::Color(33, 33, 33);        // Dark text

// Tree/Graph visualization
const sf::Color TREE_NODE   = sf::Color(33, 150, 243);      // Blue
const sf::Color TREE_EDGE   = sf::Color(100, 181, 246);     // Light blue
const sf::Color PATH_LINE   = sf::Color(156, 39, 176);      // Purple

// Sampling-based algorithm colors
const sf::Color SAMPLE      = sf::Color(255, 152, 0);       // Orange
const sf::Color RRT_NEW     = sf::Color(76, 175, 80);       // Green

}  // namespace colors
}  // namespace motion_planning
