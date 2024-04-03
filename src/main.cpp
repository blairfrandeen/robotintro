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

int main() {
    /*
    Vector3d v(4, 3, 30);
    Vector3d u(5, 5, 45);
    cout << "v = " << endl << v << endl;
    cout << "u = " << endl << u << endl;
    cout << "T(v) =" << endl << utoi(v) << endl;
    cout << "T_inv(v) =" << endl << itransform(utoi(v)) << endl;
    cout << "T(u) =" << endl << utoi(u) << endl;
    cout << "T(u) * T(v) =" << endl << tmult(u, v) << endl;
    */

    Vector3d joint_angles_deg(45, -30, 30);
    Vector3d link_lengths_m(0.5, 0.5, 0);
    Matrix3d wrelb = kin(joint_angles_deg, link_lengths_m);
    cout << "Vector Form:" << endl << itou(wrelb) << endl;
    cout << "Matrix Form:" << endl << wrelb << endl;
    return 0;
}
