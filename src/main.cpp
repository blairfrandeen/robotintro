#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <iterator>
#include <vector>

#include "robotgfx.h"
#include "robotlib.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using namespace std;

int main() {
    Vector3d origin_frame;

    Vector3d link_lengths_m(0.5, 0.5, 0);
    Planar3DOFManipulator m(link_lengths_m, origin_frame);
    Vector3d joint_angles_deg(45, -30, 0);
    /* m.move_joints(joint_angles_deg); */
    cout << m.base_to_end_effector() << endl;
    m.set_tool(Vector3d(0.1, 0.2, 30));
    cout << m.base_to_tool() << endl;

    sf::RenderWindow window(sf::VideoMode(800, 600), "Robot Intro!");
    window.setFramerateLimit(144);

    while (window.isOpen()) {
        for (auto event = sf::Event{}; window.pollEvent(event);) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        window.clear();
        m.move_to_angles(Vector3d(100, -160, 30.1));

        // draw things here:
        sf::VertexArray axs = world_axis(window, sf::Color::Cyan);
        sf::RectangleShape lnk_1 = gfx_link(m.L1, window);
        sf::RectangleShape lnk_2 = gfx_link(m.L2, window);
        sf::VertexArray frm = gfx_frame(m.L1.end_frame, window, sf::Color::Red);
        sf::VertexArray frm2 =
            gfx_frame(m.L2.end_frame, window, sf::Color::Red);
        sf::VertexArray frm3 =
            gfx_frame(m.L3.end_frame, window, sf::Color::Green);
        window.draw(axs);
        window.draw(lnk_1);
        window.draw(frm);
        window.draw(lnk_2);
        window.draw(frm2);
        window.draw(frm3);

        window.display();
    }
    return 0;
}
