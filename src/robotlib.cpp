#include "robotlib.h"

#include <cmath>

#include "Eigen/src/Core/Matrix.h"

double radians(double degrees) { return M_PI * degrees / 180.0; }
double degrees(double radians) { return 180 / M_PI * radians; }

Matrix3d utoi(Vector3d v) {
    Matrix3d m;

    // position vector
    m.topRightCorner(2, 1) << v(0), v(1);

    // rotation matrix
    auto theta_rad = radians(v(2));
    m.topLeftCorner(2, 2) << cos(theta_rad), -sin(theta_rad), sin(theta_rad),
        cos(theta_rad);

    // make 3x3 for convenience
    m.bottomRows(1) << 0, 0, 1;

    return m;
}

Vector3d itou(const Matrix3d &m) {
    Vector3d v = m.rightCols(1);
    v(2) = degrees(std::atan2(m(1, 0), m(0, 0)));

    return v;
}

Matrix2d _rot_submatrix(const Matrix3d &m) { return m.topLeftCorner(2, 2); }

Vector2d _pos_submatrix(const Matrix3d &m) { return m.topRightCorner(2, 1); }

Matrix3d tmult(const Matrix3d &a_to_b, const Matrix3d &b_to_c) {
    Matrix3d m;
    m.topLeftCorner(2, 2) = _rot_submatrix(a_to_b) * _rot_submatrix(b_to_c);
    Vector2d a_to_b_orig = a_to_b.topRightCorner(2, 1);
    Vector2d b_to_c_orig = b_to_c.topRightCorner(2, 1);
    m.topRightCorner(2, 1) = _rot_submatrix(a_to_b) * b_to_c_orig + a_to_b_orig;
    m.bottomRows(1) << 0, 0, 1;

    return m;
}

Matrix3d tmult(const Vector3d &b_rel_a, const Vector3d &c_rel_b) {
    return tmult(utoi(b_rel_a), utoi(c_rel_b));
}

Matrix3d itransform(const Matrix3d &m) {
    Matrix3d m_inv;
    m_inv.topLeftCorner(2, 2) = _rot_submatrix(m).transpose();
    m_inv.topRightCorner(2, 1) =
        -_rot_submatrix(m).transpose() * _pos_submatrix(m);
    m_inv.bottomRows(1) << 0, 0, 1;

    return m_inv;
}

Matrix3d link_transform2d(const double link_len, double theta_rad) {
    // distrance transform for link length
    Matrix3d dist_transform = Matrix3d::Identity();
    dist_transform(0, 2) = link_len;

    // rotation transform for link angle
    Matrix3d rot_transform = Matrix3d::Identity();
    rot_transform.topLeftCorner(2, 2) =
        Eigen::Rotation2D<double>(theta_rad).toRotationMatrix();

    // transform describing end of link 1 relative to the base
    Matrix3d link_transform = tmult(rot_transform, dist_transform);

    return link_transform;
}

/* Forward kinematics for problem 3.1
   Robot with planar joints. Two links with lengths 0.5.
   joint_angles in degrees

   Returns wrist frame relative to base frame
*/
Matrix3d kin(Vector3d joint_angles_deg, Vector3d link_lengths_m) {
    Matrix3d base_to_link1 =
        link_transform2d(link_lengths_m[0], radians(joint_angles_deg[0]));
    Matrix3d link1_to_link2 =
        link_transform2d(link_lengths_m[1], radians(joint_angles_deg[1]));
    Matrix3d link2_to_wrist =
        link_transform2d(link_lengths_m[2], radians(joint_angles_deg[2]));

    Matrix3d base_to_link2 = tmult(base_to_link1, link1_to_link2);
    Matrix3d base_to_wrist = tmult(base_to_link2, link2_to_wrist);

    return base_to_wrist;
}

Matrix3d where(Vector3d joint_angles_deg, Vector3d link_lengths_m,
               Matrix3d wrist_to_tool, Matrix3d base_to_station) {
    Matrix3d base_to_wrist = kin(joint_angles_deg, link_lengths_m);
    Matrix3d station_to_base = itransform(base_to_station);
    Matrix3d station_to_tool = station_to_base * base_to_wrist * wrist_to_tool;
    return station_to_tool;
}

class Link {
   public:
    const double length, twist;
    double offset, angle;
};

class PrismaticLink : public Link {
   public:
    const double length, twist, angle;
    double offset;
    PrismaticLink(double l, double t, double a, double o);
};
class RotaryLink : public Link {
   public:
    const double length, twist, offset;
    double angle;
    RotaryLink(double l, double t, double a, double o);
};
