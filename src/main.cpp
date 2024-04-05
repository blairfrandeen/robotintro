#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

#include "SFML/Graphics/CircleShape.hpp"
#include "SFML/Graphics/RectangleShape.hpp"
#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Graphics/Vertex.hpp"
#include "SFML/System/Vector2.hpp"
#include "robotlib.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using namespace std;

// multiply meters by this number to get into graphics.
const double SFML_SCALE = 400.f;

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

sf::Vector2f _transform_point(sf::Vector2f point_m, sf::RenderWindow& window) {
    sf::Vector2u win_size = window.getSize();
    double x_offset = win_size.x / 2.f + point_m.x * SFML_SCALE;
    double y_offset = win_size.y / 2.f - point_m.y * SFML_SCALE;

    return sf::Vector2f(x_offset, y_offset);
}

/* Given a link origin and length, draw a rectangle representing the link
   starting at the origin vertex, with a given length and angle measured from
   the X axis.
*/
sf::RectangleShape link(sf::Vector2f origin, double len_m, double angle_deg) {
    // Create a rectangle shape
    sf::RectangleShape linkRect(
        sf::Vector2f(len_m * SFML_SCALE, 10.0f));  // Width = len, Height = 10

    // Set the origin of the rectangle to the center of the left edge
    linkRect.setOrigin(0.0f, linkRect.getSize().y / 2.0f);

    // Set the position of the rectangle
    linkRect.setPosition(origin);

    // Rotate the rectangle around its origin
    linkRect.setRotation(-angle_deg);

    // Return the rotated and positioned rectangle
    return linkRect;
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

        // draw things here:
        sf::VertexArray axs = xyAxis(window);
        sf::RectangleShape lnk_1 =
            link(_transform_point(sf::Vector2f(0, 0), window), 0.5, 30);
        sf::RectangleShape lnk_2 =
            link(_transform_point(sf::Vector2f(0.433, 0.25), window), 0.5, 60);
        window.draw(axs);
        window.draw(lnk_1);
        window.draw(lnk_2);

        window.display();
    }
    return 0;
}
