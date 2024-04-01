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
