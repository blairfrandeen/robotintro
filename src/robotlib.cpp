#include "robotlib.h"

#include <cmath>
#include <iostream>
#include <tuple>

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

std::tuple<Vector3d, Vector3d, bool> solve(Matrix3d station_to_tool_goal,
                                           Vector3d link_lengths_m,
                                           Vector3d joint_angles_current,
                                           Matrix3d wrist_to_tool,
                                           Matrix3d base_to_station) {
    Matrix3d tool_to_wrist = itransform(wrist_to_tool);
    Matrix3d base_to_wrist_goal =
        base_to_station * station_to_tool_goal * tool_to_wrist;
    std::cout << itou(base_to_wrist_goal) << std::endl;
    return invkin(base_to_wrist_goal, joint_angles_current, link_lengths_m);
}

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

    // book equation 4.15
    double theta_2 = atan2(sin_theta_2, cos_theta_2);
    double theta_3 = phi - theta_1 - theta_2;

    return Vector3d(degrees(theta_1), degrees(theta_2), degrees(theta_3));
}

std::tuple<Vector3d, Vector3d, bool> invkin(Matrix3d goal_frame,
                                            Vector3d joint_angles_current,
                                            Vector3d link_lengths_m) {
    // initialize variables
    Vector3d sol_near, sol_far;
    Vector3d goal_frame_vec = itou(goal_frame);
    double x = goal_frame_vec[0];
    double y = goal_frame_vec[1];
    double l1 = link_lengths_m[0];
    double l2 = link_lengths_m[1];

    // book equation 4.14
    double cos_theta_2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if ((cos_theta_2 < -1) || (cos_theta_2 > 1)) {
        // no solution found
        return std::make_tuple(sol_near, sol_far, false);
    }

    // book equation 4.15
    double sin_theta_2 = sqrt(1 - cos_theta_2 * cos_theta_2);
    Vector3d sol1 =
        _get_solution(sin_theta_2, cos_theta_2, link_lengths_m, goal_frame_vec);
    Vector3d sol2 = _get_solution(-sin_theta_2, cos_theta_2, link_lengths_m,
                                  goal_frame_vec);

    double sol1_dist = abs((sol1 - joint_angles_current).topRows(2).sum());
    double sol2_dist = abs((sol2 - joint_angles_current).topRows(2).sum());

    if (sol2_dist > sol1_dist) {
        return std::make_tuple(sol1, sol2, true);
    } else {
        return std::make_tuple(sol2, sol1, true);
    }
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
