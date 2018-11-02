#include "wave/geometry/geometry.hpp"
#include "test.hpp"

/** Miscellanious tests which don't fit in the RotationTest fixture.
 *  So far the fixture doesn't test maps, so test them here.
 * (assume that the map will work the same as non-map after construction).
 */
TEST(RotationMiscTest, constructRotationQConstMap) {
    const auto q = wave::randomQuaternion<double>();
    // Currently a mapped rotation can be constructed by giving it a map temporary
    // @todo add constructor taking a pointer?
    const wave::QuaternionRotation<Eigen::Map<const Eigen::Quaterniond>> r{
      Eigen::Map<const Eigen::Quaterniond>{q.coeffs().data()}};

    EXPECT_APPROX(wave::RotationQd{q}, r);
}

TEST(RotationMiscTest, constructRotationQMap) {
    auto q = wave::randomQuaternion<double>();
    wave::QuaternionRotation<Eigen::Map<Eigen::Quaterniond>> r{
      Eigen::Map<Eigen::Quaterniond>{q.coeffs().data()}};

    EXPECT_APPROX(wave::RotationQd{q}, r);

    // Change mapped value: both should change
    r.value() = wave::randomQuaternion<double>();
    EXPECT_APPROX(wave::RotationQd{q}, r);
    EXPECT_EQ(q.coeffs().data(), r.value().coeffs().data());
}

TEST(RotationMiscTest, constructRotationMMap) {
    auto m = Eigen::Matrix3d{wave::randomQuaternion<double>()};
    const wave::MatrixRotation<Eigen::Map<Eigen::Matrix3d>> r{
      Eigen::Map<Eigen::Matrix3d>{m.data()}};

    EXPECT_APPROX(m, r.value());
    EXPECT_EQ(m.data(), r.value().data());
}
