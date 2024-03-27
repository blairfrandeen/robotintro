#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "robotlib.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using namespace std;

int main() {
    Vector3d v(4, 3, 30);
    Vector3d u(5, 5, 45);
    cout << "v = " << endl << v << endl;
    cout << "u = " << endl << u << endl;
    cout << "T(v) =" << endl << utoi(v) << endl;
    cout << "T_inv(v) =" << endl << itransform(utoi(v)) << endl;
    cout << "T(u) =" << endl << utoi(u) << endl;
    cout << "T(u) * T(v) =" << endl << tmult(u, v) << endl;
    return 0;
}
