#pragma once
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

#include "robotlib.h"

using Eigen::Vector3d;

sf::VertexArray world_axis(sf::RenderWindow& window, sf::Color axes_color);
sf::Vector2f _transform_point(sf::Vector2f point_m, sf::RenderWindow& window);
sf::VertexArray gfx_frame(Vector3d frame, sf::RenderWindow& window,
                          sf::Color axes_color);
sf::RectangleShape gfx_link(RotaryLink lnk, sf::RenderWindow& window);
