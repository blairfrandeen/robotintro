#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

#include "Eigen/src/Core/Matrix.h"
#include "robotlib.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Rotation2D;
using Eigen::Vector2d;
using Eigen::Vector3d;

using namespace std;

Matrix3d link_transform(const double link_len, double theta_rad) {
    // distrance transform for link length
    Matrix3d dist_transform = Matrix3d::Identity();
    dist_transform(0, 2) = link_len;
    /* cout << "dist transform" << endl << dist_transform << endl; */

    // rotation transform for link angle
    Matrix3d rot_transform = Matrix3d::Zero();
    rot_transform.topLeftCorner(2, 2) =
        Rotation2D<double>(theta_rad).toRotationMatrix();
    rot_transform(2, 2) = 1;
    /* cout << "rot transform" << endl << rot_transform << endl; */

    // transform describing end of link 1 relative to the base
    /* Matrix3d link_transform = tmult(dist_transform, rot_transform); */
    Matrix3d link_transform = dist_transform * rot_transform;
    /* cout << "link transform" << endl << link_transform << endl; */

    return link_transform;
}

/* Forward kinematics for problem 3.1
   Robot with planar joints. Two links with lengths 0.5.
   joint_angles in degrees

   Returns wrist frame relative to base frame
*/
Matrix3d kin(Vector3d joint_angles_deg) {
    // both links have length 0.5
    const double L1 = 0.5, L2 = 0.5;

    Matrix3d link1_rel_b = link_transform(L1, radians(joint_angles_deg[0]));
    Matrix3d link2_rel_1 = link_transform(L2, radians(joint_angles_deg[1]));
    Matrix3d link3_rel_2 = link_transform(0, radians(joint_angles_deg[2]));

    Matrix3d wrelb = tmult(link3_rel_2, tmult(link2_rel_1, link1_rel_b));

    return wrelb;
}

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

    Vector3d joint_angles(30, 30, 30);
    cout << "Vector Form:" << endl << itou(kin(joint_angles)) << endl;
    cout << "Matrix Form:" << endl << kin(joint_angles) << endl;
    return 0;
}
