#include "wave/geometry/src/util/math/CrossMatrix.hpp"
#include "../test.hpp"

TEST(ExpMapTest, quaternionMatchesEigenAA) {
    const auto v = Eigen::Vector3d{Eigen::Vector3d::Random()};
    const auto a = Eigen::AngleAxisd{v.norm(), v.normalized()};

    const auto res_q = wave::quaternionFromExpMap(v);
    const auto expected = Eigen::Quaterniond{a};
    EXPECT_APPROX(expected, res_q);
}

TEST(LogMapTest, quaternionMatchesEigenAA) {
    const auto q = wave::RotationQd::Random().value();
    const auto a = Eigen::AngleAxisd{q};

    const auto res_q = wave::rotationVectorFromQuaternion(q);
    const auto expected = Eigen::Vector3d{a.axis() * a.angle()};
    EXPECT_APPROX(expected, res_q);
}

TEST(LogMapTest, matrixMatchesEigenAA) {
    const auto q = wave::RotationQd::Random().value();
    const auto a = Eigen::AngleAxisd{q};

    const auto res_m = wave::rotationVectorFromMatrix(q.matrix());
    const auto expected = Eigen::Vector3d{a.axis() * a.angle()};
    EXPECT_APPROX(expected, res_m);
}
