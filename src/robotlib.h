#pragma once
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

double radians(double degrees);
double degrees(double radians);
Matrix3d utoi(Vector3d v);
Vector3d itou(const Matrix3d &m);
Matrix2d _rot_submatrix(const Matrix3d &m);
Vector2d _pos_submatrix(const Matrix3d &m);
Matrix3d tmult(const Matrix3d &b_rel_a, const Matrix3d &c_rel_b);
Matrix3d tmult(const Vector3d &b_rel_a, const Vector3d &c_rel_b);
Matrix3d itransform(const Matrix3d &m);
Matrix3d link_transform2d(const double link_len, double theta_rad);
const Matrix3d kin(Vector3d joint_angles_deg, Vector3d link_lengths_m);
const std::tuple<Vector3d, Vector3d, bool> invkin(Matrix3d goal_frame,
                                                  Vector3d joint_angles_current,
                                                  Vector3d link_lengths_m);
const std::tuple<Vector3d, Vector3d, bool> solve(Matrix3d station_to_tool_goal,
                                                 Vector3d link_lengths_m,
                                                 Vector3d joint_angles_current,
                                                 Matrix3d wrist_to_tool,
                                                 Matrix3d base_to_station);

const double MAX_JOINT_SPEED_DEG_FRAME = 0.2;

class RotaryLink {
   public:
    const double length_m;
    double theta_rad;
    Vector3d &origin_frame;
    Vector3d end_frame;
    RotaryLink(Vector3d &origin_frame, double len_m)
        : length_m(len_m),
          origin_frame(origin_frame),
          end_frame(_get_end_frame(origin_frame)) {
        theta_rad = 0.f;
    }

    void rotate(double angle_deg) {
        theta_rad += radians(angle_deg);
        end_frame = _get_end_frame(origin_frame);
    }

   private:
    Vector3d _get_end_frame(Vector3d &base_to_origin) {
        Vector3d origin_to_end(length_m * cos(theta_rad),
                               length_m * sin(theta_rad), degrees(theta_rad));
        return itou(tmult(base_to_origin, origin_to_end));
    }
};

class Planar3DOFManipulator {
   public:
    Vector3d &base_frame;
    RotaryLink L1;
    RotaryLink L2;
    RotaryLink L3;
    Planar3DOFManipulator(Vector3d link_lengths_m, Vector3d &base_frame)
        : base_frame(base_frame),
          L1(base_frame, link_lengths_m[0]),
          L2(L1.end_frame, link_lengths_m[1]),
          L3(L2.end_frame, link_lengths_m[2]),
          _wrist_to_tool(Matrix3d()) {}

    void move_joints(Vector3d joint_angles_deg) {
        L1.rotate(joint_angles_deg[0]);
        L2.rotate(joint_angles_deg[1]);
        L3.rotate(joint_angles_deg[2]);
    }

    Vector3d base_to_end_effector() const { return L3.end_frame; }

    Vector3d base_to_tool() const {
        // L3.end_frame is base_to_wrist
        // want base_to_tool

        return itou(tmult(utoi(L3.end_frame), _wrist_to_tool));
    }

    Vector3d joint_angles_deg() const {
        return Vector3d(degrees(L1.theta_rad), degrees(L2.theta_rad),
                        degrees(L3.theta_rad));
    }

    Vector3d link_lengths_m() const {
        return Vector3d(L1.length_m, L2.length_m, L3.length_m);
    }

    Matrix3d wrist_to_tool() const { return _wrist_to_tool; }

    void set_tool(const Vector3d wrist_to_tool_u) {
        _wrist_to_tool = utoi(wrist_to_tool_u);
    }

    void move_to_angles(const Vector3d target_joint_angles_deg) {
        Vector3d dist_to_go = target_joint_angles_deg - joint_angles_deg();
        if (abs(dist_to_go.sum()) > 0) {
            double max_dist = dist_to_go.cwiseAbs().maxCoeff();
            move_joints(dist_to_go / max_dist * MAX_JOINT_SPEED_DEG_FRAME);
        }
    }

   private:
    Matrix3d _wrist_to_tool;
};
