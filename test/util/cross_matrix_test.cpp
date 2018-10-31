#include "wave/geometry/src/util/math/CrossMatrix.hpp"
#include "../test.hpp"

/** The list of implementation types to run each test case on */
using TranslationTypes = testing::Types<Eigen::Vector3d, Eigen::Vector3f>;

namespace {
/** Returns the skew-symmetric "cross-product" or "hat" matrix of a 3-vector
 * This was my original implementation before making the CrossMatrix expression */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> manualCrossMatrix(
  const Eigen::MatrixBase<Derived> &v) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    using S = typename Derived::Scalar;
    Eigen::Matrix<S, 3, 3> m;
    m << S{0}, -v.z(), v.y(),  //
      v.z(), S{0}, -v.x(),     //
      -v.y(), v.x(), S{0};
    return m;
};
}  // namespace

TEST(CrossMatrixTest, evaluate) {
    for (int reps = 100; reps--;) {
        Eigen::Vector3d vec = Eigen::Vector3d::Random();

        Eigen::Matrix3d expected = manualCrossMatrix(vec);
        const auto &crossExpr = wave::crossMatrix(vec);

        EXPECT_APPROX(expected, crossExpr);
    }
}

TEST(CrossMatrixTest, evaluateForProduct) {
    for (int reps = 100; reps--;) {
        Eigen::Matrix3d mat = Eigen::Matrix3d::Random();
        Eigen::Vector3d vec = Eigen::Vector3d::Random();

        Eigen::Matrix3d expected = manualCrossMatrix(mat * vec);
        const auto &crossExpr = wave::crossMatrix(mat * vec);

        EXPECT_APPROX(expected, crossExpr);
    }
}

TEST(CrossMatrixTest, multiplyVec) {
    Eigen::Vector3d a, b, expected, actual;
    for (int reps = 100; reps--;) {
        a.setRandom();
        b.setRandom();

        expected = manualCrossMatrix(a) * b;
        actual = wave::crossMatrix(a) * b;

        EXPECT_APPROX(expected, actual);
    }
}

TEST(CrossMatrixTest, multiplyNegativeVec) {
    Eigen::Vector3d a, b, expected, actual;
    for (int reps = 100; reps--;) {
        a.setRandom();
        b.setRandom();

        expected = -manualCrossMatrix(a) * b;
        actual = -wave::crossMatrix(a) * b;

        EXPECT_APPROX(expected, actual);
    }
}

TEST(CrossMatrixTest, multiplyMat) {
    Eigen::Vector3d a;
    Eigen::Matrix<double, 3, 20> b, expected, actual;
    for (int reps = 100; reps--;) {
        a.setRandom();
        b.setRandom();

        expected = manualCrossMatrix(a) * b;
        actual = wave::crossMatrix(a) * b;

        EXPECT_APPROX(expected, actual);
    }
}

TEST(CrossMatrixTest, crossNegTimesMat) {
    Eigen::Vector3d a;
    Eigen::Matrix<double, 3, 20> b, expected, actual;
    for (int reps = 100; reps--;) {
        a.setRandom();
        b.setRandom();

        expected = manualCrossMatrix(-a) * b;
        actual = wave::crossMatrix(-a) * b;

        EXPECT_APPROX(expected, actual);
    }
}


TEST(CrossMatrixTest, multiplyMatRight) {
    Eigen::Vector3d a;
    Eigen::Matrix<double, 20, 3> b, expected, actual;
    for (int reps = 100; reps--;) {
        a.setRandom();
        b.setRandom();

        expected = b * manualCrossMatrix(a);
        actual = b * wave::crossMatrix(a);

        EXPECT_APPROX(expected, actual);
    }
}

TEST(CrossMatrixTest, multiplyDynamicMat) {
    Eigen::Vector3d a;
    Eigen::MatrixXd b, expected, actual;
    for (int reps = 100; reps--;) {
        a.setRandom();
        b.setRandom(3, 20);

        expected = manualCrossMatrix(a) * b;
        actual = wave::crossMatrix(a) * b;

        EXPECT_APPROX(expected, actual);
    }
}

TEST(CrossMatrixTest, multiplyDynamicMatRight) {
    Eigen::Vector3d a;
    Eigen::MatrixXd b, expected, actual;
    for (int reps = 100; reps--;) {
        a.setRandom();
        b.setRandom(20, 3);

        expected = b * manualCrossMatrix(a);
        actual = b * wave::crossMatrix(a);

        EXPECT_APPROX(expected, actual);
    }
}

TEST(CrossMatrixTest, multiplyCrossMatrices) {
    Eigen::Vector3d a, b, c;
    Eigen::Matrix3d expected, actual;
    for (int reps = 100; reps--;) {
        a.setRandom();
        b.setRandom();
        c.setRandom();

        expected = manualCrossMatrix(a) * manualCrossMatrix(b) * manualCrossMatrix(c);
        actual = wave::crossMatrix(a) * wave::crossMatrix(b) * wave::crossMatrix(c);

        EXPECT_APPROX(expected, actual);
    }
}

TEST(CrossMatrixTest, inverse) {
    Eigen::Vector3d a;
    Eigen::Matrix3d expected, actual;
    for (int reps = 100; reps--;) {
        a.setRandom();

        expected = manualCrossMatrix(a).inverse();
        actual = wave::crossMatrix(a).inverse();

        EXPECT_EQ(expected, actual);
    }
}


TEST(CrossMatrixTest, attributes) {
    Eigen::Vector3d vec = Eigen::Vector3d::Random();
    const auto &crossExpr = wave::CrossMatrix<Eigen::Vector3d>{vec};
    EXPECT_EQ(3, crossExpr.rows());
    EXPECT_EQ(3, crossExpr.cols());
}
