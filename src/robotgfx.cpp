#include "robotgfx.h"

#include "robotlib.h"

using Eigen::Vector3d;

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
sf::RectangleShape gfx_link(RotaryLink lnk, sf::RenderWindow& window) {
    // Create a rectangle shape
    sf::RectangleShape linkRect(sf::Vector2f(
        lnk.length_m * SFML_SCALE, 10.0f));  // Width = len, Height = 10

    // Set the origin of the rectangle to the center of the left edge
    linkRect.setOrigin(0.0f, linkRect.getSize().y / 2.0f);

    // Set the position of the rectangle
    sf::Vector2f oframe(lnk.origin_frame[0], lnk.origin_frame[1]);
    linkRect.setPosition(_transform_point(oframe, window));

    // Rotate the rectangle around its origin
    linkRect.setRotation(-lnk.end_frame[2]);

    // Return the rotated and positioned rectangle
    return linkRect;
}
