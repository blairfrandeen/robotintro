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
    Vector3d joint_angles_current(0, 0, 0);
    Vector3d link_lengths_m(0.5, 0.5, 0);
    Matrix3d goal_frame = utoi(Vector3d(0, 0, -90));
    Matrix3d wrelb = kin(joint_angles_current, link_lengths_m);
    Vector3d wrist_to_tool(0.1, 0.2, 30);
    Vector3d base_to_station(-0.1, 0.3, 0);
    cout << "Forward Kinematics:" << endl << itou(wrelb) << endl;
    auto [sol_near, sol_far, found_sol] =
        /* invkin(goal_frame, joint_angles_current, link_lengths_m); */
        solve(goal_frame, link_lengths_m, joint_angles_current,
              utoi(wrist_to_tool), utoi(base_to_station));
    if (found_sol) {
        cout << "Near solution: " << endl << sol_near << endl;
        cout << "Far solution: " << endl << sol_far << endl;
    } else {
        cout << "No solution!" << endl;
    }
    return 0;
}
