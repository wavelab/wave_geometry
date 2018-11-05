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
    for (int reps = 100; reps--;) {
        const auto q = wave::RotationQd::Random().value();
        const auto a = Eigen::AngleAxisd{q};

        const auto res_q = wave::rotationVectorFromQuaternion(q);
        const auto expected = Eigen::Vector3d{a.axis() * a.angle()};
        ASSERT_APPROX(expected, res_q);

        // The inverse rotation produces an opposite vector
        const auto res_q_n = wave::rotationVectorFromQuaternion(q.conjugate());
        ASSERT_APPROX(-expected, res_q_n);
    }
}

TEST(LogMapTest, matrixMatchesEigenAA) {
    for (int reps = 100; reps--;) {
        const auto q = wave::RotationQd::Random().value();
        const auto a = Eigen::AngleAxisd{q};
        const auto m = q.matrix();

        const auto res_m = wave::rotationVectorFromMatrix(m);
        const auto expected = Eigen::Vector3d{a.axis() * a.angle()};
        ASSERT_APPROX(expected, res_m);

        // The inverse rotation produces an opposite vector
        const auto res_q_n = wave::rotationVectorFromMatrix(m.transpose());
        ASSERT_APPROX(-expected, res_q_n);
    }
}
