#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

#include "SFML/Graphics/CircleShape.hpp"
#include "SFML/Graphics/RenderWindow.hpp"
#include "robotlib.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using namespace std;

sf::VertexArray xyAxis(sf::RenderWindow& window) {
    // Create the X and Y axes
    sf::VertexArray xyAxis(sf::Lines, 4);

    // Set the position of the axes
    float xAxisLength = window.getSize().x;
    float yAxisLength = window.getSize().y;
    float xOffset = window.getSize().x / 2.f;
    float yOffset = window.getSize().y / 2.f;

    // Define the X axis
    xyAxis[0].position = sf::Vector2f(-xAxisLength, yOffset);
    xyAxis[1].position = sf::Vector2f(xAxisLength, yOffset);

    // Define the Y axis
    xyAxis[2].position = sf::Vector2f(xOffset, -yAxisLength);
    xyAxis[3].position = sf::Vector2f(xOffset, yAxisLength);

    // Set the color of the axes
    xyAxis[0].color = sf::Color::Blue;
    xyAxis[1].color = sf::Color::Blue;
    xyAxis[2].color = sf::Color::Blue;
    xyAxis[3].color = sf::Color::Blue;

    return xyAxis;
}

int main() {
    Vector3d joint_angles_current(0, 0, 0);
    Vector3d link_lengths_m(0.5, 0.5, 0);
    /* Matrix3d goal_frame = utoi(Vector3d(0.6, -0.3, 45)); */
    Matrix3d goal_frame = utoi(Vector3d(0, 0, -90));
    Matrix3d wrelb = kin(joint_angles_current, link_lengths_m);
    Vector3d wrist_to_tool(0.1, 0.2, 30);
    Vector3d base_to_station(-0.1, 0.3, 0);
    cout << "Forward Kinematics:" << endl << itou(wrelb) << endl;
    auto [sol_near, sol_far, found_sol] =
        solve(goal_frame, link_lengths_m, joint_angles_current,
              utoi(wrist_to_tool), utoi(base_to_station));
    if (found_sol) {
        cout << "Near solution: " << endl << sol_near << endl;
        cout << "Far solution: " << endl << sol_far << endl;
    } else {
        cout << "No solution!" << endl;
    }
    auto window = sf::RenderWindow{{800, 600u}, "Robot Intro!"};
    window.setFramerateLimit(144);

    while (window.isOpen()) {
        for (auto event = sf::Event{}; window.pollEvent(event);) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        window.clear();

        sf::CircleShape shape(50.f);
        shape.setFillColor(sf::Color(100, 250, 50));
        // draw things here:
        sf::VertexArray axs = xyAxis(window);
        window.draw(axs);
        window.draw(shape);

        window.display();
    }
    return 0;
}
