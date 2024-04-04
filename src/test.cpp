#include <catch2/catch_test_macros.hpp>

#include "Eigen/Geometry"
#include "Eigen/src/Core/Matrix.h"
#include "robotlib.cpp"
#include "robotlib.h"

TEST_CASE("Angle conversions are two way", "[angle]") {
    REQUIRE(radians(0.0) == degrees(0.0));
    REQUIRE(radians(360.0) == 2 * M_PI);
    REQUIRE(radians(180.0) == M_PI);
    REQUIRE(degrees(2 * M_PI) == 360.0);
    REQUIRE(radians(50.0) == radians(degrees(radians(50.0))));
}

TEST_CASE("user to internal form works", "[matrix]") {
    Eigen::Vector3d user_form(4, 3, 30);
    Eigen::Matrix3d expected;
    expected << 0.866025, -0.5, 4, 0.5, 0.866025, 3, 0, 0, 1;
    REQUIRE(utoi(user_form).isApprox(expected, 1e-6));
}

TEST_CASE("internal to user form works", "[matrix]") {
    Eigen::Vector3d expected(4, 3, 30.);
    Eigen::Matrix3d internal_form;
    internal_form << 0.866025, -0.5, 4, 0.5, 0.866025, 3, 0, 0, 1;
    REQUIRE(itou(internal_form).isApprox(expected, 1e-1));
}

TEST_CASE("inverse transform works", "[matrix]") {
    Eigen::Matrix3d transform_mat;
    transform_mat << 0.866025, -0.5, 4, 0.5, 0.866025, 3, 0, 0, 1;
    REQUIRE(itransform(transform_mat).isApprox(transform_mat.inverse(), 1e-6));

    for (int i = 0; i > 5; i++) {
        Eigen::Matrix3d r;
        Eigen::Rotation2D<double> rot(rand());
        r.topLeftCorner(2, 2) = rot.toRotationMatrix();
        r.topRightCorner(2, 1) << rand(), rand();
        r.bottomRows(1) << 0, 0, 1;

        REQUIRE(itransform(r).isApprox(r.inverse(), 1e-6));
    }
}

TEST_CASE("matrix multiplication", "[matrix]") {
    /* Eigen::Rotation2D<double> rot1(rand()); */
    Eigen::Matrix3d T1 = Eigen::Matrix3d::Identity();
    T1.topLeftCorner(2, 2) =
        Eigen::Rotation2D<double>(rand()).toRotationMatrix();
    T1.rightCols(1) << rand(), rand(), 1;
    Eigen::Matrix3d T2 = Eigen::Matrix3d::Identity();
    T2.topLeftCorner(2, 2) =
        Eigen::Rotation2D<double>(rand()).toRotationMatrix();
    T1.rightCols(1) << rand(), rand(), 1;
    REQUIRE(tmult(T1, T2) == T1 * T2);
}

TEST_CASE("Example 2.1", "[example]") {
    Eigen::Rotation2D<double> rot_b_rel_a(radians(30));
    Eigen::Matrix3d transform;
    transform.topLeftCorner(2, 2) = rot_b_rel_a.toRotationMatrix();
    transform.bottomRows(1) << 0, 0, 1;
    transform.rightCols(1) << 0, 0, 1;

    Eigen::Vector3d p_rel_b(0, 2, 0);
    Eigen::Vector3d a_rel_p_expected(-1, 1.732, 0);
    Eigen::Vector3d result = transform * p_rel_b;

    REQUIRE(result.isApprox(a_rel_p_expected, 1e-3));
}

TEST_CASE("Programming Exercise 2.5", "[exercise]") {
    Eigen::Vector3d a_rel_u(11, -1, 30);
    Eigen::Vector3d b_rel_a(0, 7, 45);
    Eigen::Vector3d u_rel_c(-3, -3, -30);

    Eigen::Matrix3d result =
        tmult(b_rel_a, itou(itransform(tmult(u_rel_c, a_rel_u))));
    Eigen::Vector3d expected(-10.884, 9.36156, 45);
    REQUIRE(itou(result).isApprox(expected, 1e-3));
}

TEST_CASE("forward kinematics", "[kinematics]") {
    Eigen::Vector3d link_len_m(0.5, 0.5, 0);
    // solved on paper
    REQUIRE(itou(kin(Eigen::Vector3d(45, -30, -30), link_len_m))
                .isApprox(Eigen::Vector3d(0.836, 0.482, -15), 1e-3));
    // solved on paper
    REQUIRE(itou(kin(Eigen::Vector3d(30, 30, 0), link_len_m))
                .isApprox(Eigen::Vector3d(0.683, 0.683, 60), 1e-3));
}

TEST_CASE("where kinematics", "[kinematics]") {
    Eigen::Vector3d link_len_m(0.5, 0.5, 0);
    Eigen::Vector3d wrist_to_tool(0.1, 0.2, 30);
    Eigen::Vector3d base_to_station(-0.1, 0.3, 0);
    // solved in onshape
    REQUIRE(itou(where(Eigen::Vector3d(45, -30, -30), link_len_m,
                       utoi(wrist_to_tool), utoi(base_to_station)))
                .isApprox(Eigen::Vector3d(1.085, 0.350, 15), 1e-3));
    REQUIRE(itou(where(Eigen::Vector3d(30, 30, 0), link_len_m,
                       utoi(wrist_to_tool), utoi(base_to_station)))
                .isApprox(Eigen::Vector3d(0.660, 0.570, 90), 1e-3));
}
