#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

#include "Eigen/src/Core/Matrix.h"
#include "robotlib.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using namespace std;

Vector3d _get_solution(double sin_theta_2, double cos_theta_2,
                       Vector3d link_lengths_m, Vector3d goal_frame_vec) {
    double x = goal_frame_vec[0];
    double y = goal_frame_vec[1];
    double phi = radians(goal_frame_vec[2]);
    double l1 = link_lengths_m[0];
    double l2 = link_lengths_m[1];
    // book equation 4.19
    double k1 = l1 + l2 * cos_theta_2;
    double k2 = l2 * sin_theta_2;

    // book equation 4.27
    double theta_1 = atan2(y, x) - atan2(k2, k1);
    double theta_2 = asin(sin_theta_2);
    double theta_3 = phi - theta_1 - theta_2;

    return Vector3d(degrees(theta_1), degrees(theta_2), degrees(theta_3));
}

tuple<Vector3d, Vector3d, bool> invkin(Matrix3d goal_frame,
                                       Vector3d joint_angles_current,
                                       Vector3d link_lengths_m) {
    // initialize variables
    Vector3d sol_near, sol_far;
    bool found_sol = false;
    Vector3d goal_frame_vec = itou(goal_frame);
    double x = goal_frame_vec[0];
    double y = goal_frame_vec[1];
    double l1 = link_lengths_m[0];
    double l2 = link_lengths_m[1];

    // book equation 4.14
    double cos_theta_2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if ((cos_theta_2 < -1) || (cos_theta_2 > 1)) {
        // no solution found
        cout << "No solution found!" << endl;
        return std::make_tuple(sol_near, sol_far, found_sol);
    }

    // book equation 4.15
    double sin_theta_2 = sqrt(1 - cos_theta_2 * cos_theta_2);
    Vector3d sol1 =
        _get_solution(sin_theta_2, cos_theta_2, link_lengths_m, goal_frame_vec);
    cout << sol1 << endl;
    Vector3d sol2 = _get_solution(-sin_theta_2, cos_theta_2, link_lengths_m,
                                  goal_frame_vec);
    cout << sol2 << endl;

    double sol1_dist = abs((sol1 - joint_angles_current).topRows(2).sum());
    double sol2_dist = abs((sol2 - joint_angles_current).topRows(2).sum());
    cout << "sol 1 dist: " << sol1_dist << endl;
    cout << "sol 2 dist: " << sol2_dist << endl;

    if (sol2_dist > sol1_dist) {
        return std::make_tuple(sol1, sol2, true);
    } else {
        return std::make_tuple(sol2, sol1, true);
    }
}

int main() {
    Vector3d joint_angles_current(35, 30, 60);
    Vector3d link_lengths_m(0.5, 0.5, 0);
    Matrix3d goal_frame = utoi(Vector3d(0.683, 0.683, 90));
    Matrix3d wrelb = kin(joint_angles_current, link_lengths_m);
    cout << "Forward Kinematics:" << endl << itou(wrelb) << endl;
    auto [sol_near, sol_far, found_sol] =
        invkin(goal_frame, joint_angles_current, link_lengths_m);
    if (found_sol) {
        cout << "Near solution: " << endl << sol_near << endl;
        cout << "Far solution: " << endl << sol_far << endl;
    } else {
        cout << "No solution!" << endl;
    }
    return 0;
}
