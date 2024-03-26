#include <cmath>
#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using namespace std;

float radians(float degrees) {
    return M_PI * degrees / 180.0;
}

float degrees(float radians) {
    return 180 / M_PI * radians;
}

Matrix3d utoi(Vector3d v) {
    Matrix3d m;
    // position vector
    m.topRightCorner(2,1) << v(0),
                             v(1);

    // rotation matrix
    auto theta_rad = radians(v(2));
    m.topLeftCorner(2,2) << cos(theta_rad), -sin(theta_rad),
                            sin(theta_rad), cos(theta_rad);

    // make 3x3 for convenience
    m.bottomRows(1) << 0, 0, 1;

    return m;
}

Matrix2d _rot_submatrix(const Matrix3d& m) {
    return m.topLeftCorner(2,2);
}

Vector2d _pos_submatrix(const Matrix3d& m) {
    return m.topRightCorner(2,1);
}


Vector3d itou(const Matrix3d& m) {
    Vector3d v = m.rightCols(1);
    v(2) = degrees(acos(m(0,0))); // better way?

    return v;
}

Matrix3d tmult(Matrix3d b_rel_a, Matrix3d c_rel_b) {
    Matrix3d m;
    m.topLeftCorner(2,2) = _rot_submatrix(c_rel_b) * _rot_submatrix(b_rel_a);

    Vector2d b_org_rel_a = b_rel_a.topRightCorner(2,1);
    Vector2d c_org_rel_b = c_rel_b.topRightCorner(2,1);

    m.topRightCorner(2,1) = _rot_submatrix(b_rel_a) * c_org_rel_b + b_org_rel_a;
    m.bottomRows(1) << 0, 0, 1;


    return m;
    /* return c_rel_b * b_rel_a; */
}

Matrix3d tmult(Vector3d b_rel_a, Vector3d c_rel_b) {
    return tmult(utoi(b_rel_a), utoi(c_rel_b));
}

Matrix3d itransform(const Matrix3d& m) {
    Matrix3d m_inv;
    m_inv.topLeftCorner(2,2) = _rot_submatrix(m).transpose();
    m_inv.topRightCorner(2,1) = - _rot_submatrix(m).transpose() * _pos_submatrix(m);
    m_inv.bottomRows(1) << 0, 0, 1;

    return m_inv;
}

int main()
{

    Vector3d v(4,3,30);
    Vector3d u(5,5,45);
    cout << "v = " << endl << v << endl;
    cout << "u = " << endl << u << endl;
    cout << "T(v) =" << endl << utoi(v) << endl;
    cout << "T_inv(v) =" << endl << itransform(utoi(v)) << endl;
    cout << "T(u) =" << endl << utoi(u) << endl;
    cout << "T(u) * T(v) =" << endl << tmult(u,v) << endl;

    return 0;
}

