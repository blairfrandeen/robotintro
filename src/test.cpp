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
