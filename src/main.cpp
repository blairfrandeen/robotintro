#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

#include "robotlib.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using namespace std;

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
