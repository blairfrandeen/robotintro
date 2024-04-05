#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

#include "SFML/Graphics/Color.hpp"
#include "SFML/System/Vector2.hpp"
#include "SFML/Window/Window.hpp"
#include "robotlib.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using namespace std;

// multiply meters by this number to get into graphics.
const double SFML_SCALE = 300.f;

sf::VertexArray world_axis(sf::RenderWindow& window, sf::Color axes_color) {
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
    for (int i = 0; i < 4; i++) {
        xyAxis[i].color = axes_color;
    }

    return xyAxis;
}

sf::Vector2f _transform_point(sf::Vector2f point_m, sf::RenderWindow& window) {
    sf::Vector2u win_size = window.getSize();
    double x_offset = win_size.x / 2.f + point_m.x * SFML_SCALE;
    double y_offset = win_size.y / 2.f - point_m.y * SFML_SCALE;

    return sf::Vector2f(x_offset, y_offset);
}

sf::VertexArray gfx_frame(Vector3d frame, sf::RenderWindow& window,
                          sf::Color axes_color) {
    const float AXIS_LENGTH = 100.0f;  // Length of the axes in pixels
    const float ARROW_SIZE = 10.0f;    // Size of the arrowhead in pixels

    sf::VertexArray frame_vtx(sf::Lines);

    sf::Vector2f origin_m(frame[0], frame[1]);
    // Transform the origin from world coordinates to SFML window coordinates
    sf::Vector2f origin_px = _transform_point(origin_m, window);

    // Calculate the rotation angle in radians
    float angle_rad = -radians(frame[2]);

    // Calculate the X-axis endpoints
    sf::Vector2f x_start(origin_px.x, origin_px.y);
    sf::Vector2f x_end(origin_px.x + AXIS_LENGTH * cos(angle_rad),
                       origin_px.y + AXIS_LENGTH * sin(angle_rad));

    // Calculate the Y-axis endpoints
    sf::Vector2f y_start(origin_px.x, origin_px.y);
    sf::Vector2f y_end(origin_px.x + AXIS_LENGTH * sin(angle_rad),
                       origin_px.y - AXIS_LENGTH * cos(angle_rad));

    // Draw the X-axis
    frame_vtx.append(sf::Vertex(x_start, axes_color));
    frame_vtx.append(sf::Vertex(x_end, axes_color));

    // Draw the arrowhead for the X-axis
    sf::Vector2f x_arrow_start1(
        x_end.x - ARROW_SIZE * cos(angle_rad) - ARROW_SIZE * sin(angle_rad),
        x_end.y - ARROW_SIZE * sin(angle_rad) + ARROW_SIZE * cos(angle_rad));
    sf::Vector2f x_arrow_end1(x_end.x, x_end.y);
    frame_vtx.append(sf::Vertex(x_arrow_start1, axes_color));
    frame_vtx.append(sf::Vertex(x_arrow_end1, axes_color));

    sf::Vector2f x_arrow_start2(
        x_end.x - ARROW_SIZE * cos(angle_rad) + ARROW_SIZE * sin(angle_rad),
        x_end.y - ARROW_SIZE * sin(angle_rad) - ARROW_SIZE * cos(angle_rad));
    sf::Vector2f x_arrow_end2(x_end.x, x_end.y);
    frame_vtx.append(sf::Vertex(x_arrow_start2, axes_color));
    frame_vtx.append(sf::Vertex(x_arrow_end2, axes_color));

    // Draw the Y-axis
    frame_vtx.append(sf::Vertex(y_start, axes_color));
    frame_vtx.append(sf::Vertex(y_end, axes_color));

    // Draw the arrowhead for the Y-axis
    sf::Vector2f y_arrow_start1(
        y_end.x + ARROW_SIZE * cos(angle_rad + M_PI / 2) -
            ARROW_SIZE * sin(angle_rad + M_PI / 2),
        y_end.y + ARROW_SIZE * sin(angle_rad + M_PI / 2) +
            ARROW_SIZE * cos(angle_rad + M_PI / 2));
    sf::Vector2f y_arrow_end1(y_end.x, y_end.y);
    frame_vtx.append(sf::Vertex(y_arrow_start1, axes_color));
    frame_vtx.append(sf::Vertex(y_arrow_end1, axes_color));

    sf::Vector2f y_arrow_start2(
        y_end.x + ARROW_SIZE * cos(angle_rad + M_PI / 2) +
            ARROW_SIZE * sin(angle_rad + M_PI / 2),
        y_end.y + ARROW_SIZE * sin(angle_rad + M_PI / 2) -
            ARROW_SIZE * cos(angle_rad + M_PI / 2));
    sf::Vector2f y_arrow_end2(y_end.x, y_end.y);
    frame_vtx.append(sf::Vertex(y_arrow_start2, axes_color));
    frame_vtx.append(sf::Vertex(y_arrow_end2, axes_color));

    return frame_vtx;
}
/* Given a link origin and length, draw a rectangle representing the link
   starting at the origin vertex, with a given length and angle measured from
   the X axis.
*/
sf::RectangleShape link(sf::Vector2f origin, double len_m, double angle_deg,
                        sf::RenderWindow& window) {
    // Create a rectangle shape
    sf::RectangleShape linkRect(
        sf::Vector2f(len_m * SFML_SCALE, 10.0f));  // Width = len, Height = 10

    // Set the origin of the rectangle to the center of the left edge
    linkRect.setOrigin(0.0f, linkRect.getSize().y / 2.0f);

    // Set the position of the rectangle
    linkRect.setPosition(_transform_point(origin, window));

    // Rotate the rectangle around its origin
    linkRect.setRotation(-angle_deg);

    // Return the rotated and positioned rectangle
    return linkRect;
}

int main() {
    Vector3d joint_angles_current(0, 0, 0);
    Vector3d link_lengths_m(0.5, 0.5, 0);
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
        sf::VertexArray axs = world_axis(window, sf::Color::Cyan);
        sf::RectangleShape lnk_1 = link(sf::Vector2f(0, 0), 0.5, 30, window);
        sf::VertexArray frm =
            gfx_frame(Vector3d(0.433, 0.25, 30), window, sf::Color::Red);
        sf::RectangleShape lnk_2 =
            link(sf::Vector2f(0.433, 0.25), 0.5, 60, window);
        sf::VertexArray frm2 =
            gfx_frame(Vector3d(0.683, 0.683, 60), window, sf::Color::Red);
        window.draw(axs);
        window.draw(lnk_1);
        window.draw(frm);
        window.draw(lnk_2);
        window.draw(frm2);

        window.display();
    }
    return 0;
}
