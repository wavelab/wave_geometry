#include <benchmark/benchmark.h>
#include "../bechmark_helpers.hpp"

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/expressionTesting.h>


#include "wave/geometry/src/util/math/math.hpp"


template <typename T>
using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;


using gtsam::Rot3;
using gtsam::Point3;
using gtsam::Expression;

/** Return a vector of random gtsam rotations */
EigenVector<Rot3> randomRot3s(int N) {
    EigenVector<Rot3> v;
    v.reserve(N);
    for (auto i = N; i--;) {
        const auto &q = wave::randomQuaternion<double>();
        v.emplace_back(q);
    }
    return v;
}

inline Eigen::Matrix3d crossMatrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),  //
      v.z(), 0.0, -v.x(),     //
      -v.y(), v.x(), 0.0;
    return m;
};

class RotateChain : public benchmark::Fixture {
 protected:
    const int N = 1000;
    std::vector<gtsam::Values> vv;
    const EigenVector<Rot3> R1 = randomRot3s(N);
    const EigenVector<Rot3> R2 = randomRot3s(N);
    const EigenVector<Rot3> R3 = randomRot3s(N);
    const EigenVector<Rot3> R4 = randomRot3s(N);
    const EigenVector<Rot3> R5 = randomRot3s(N);
    const EigenVector<Rot3> R6 = randomRot3s(N);
    const EigenVector<Rot3> R7 = randomRot3s(N);
    const EigenVector<Rot3> R8 = randomRot3s(N);
    const EigenVector<Rot3> R9 = randomRot3s(N);
    const EigenVector<Rot3> R10 = randomRot3s(N);
    const EigenVector<Point3> v1 = randomMatrices<Point3>(N);

    RotateChain() {
        for (auto i = 0; i < N; ++i) {
            gtsam::Values values{};
            values.insert(gtsam::Symbol{'R', 1}, Rot3{wave::randomQuaternion<double>()});
            values.insert(gtsam::Symbol{'R', 2}, Rot3{wave::randomQuaternion<double>()});
            values.insert(gtsam::Symbol{'R', 3}, Rot3{wave::randomQuaternion<double>()});
            values.insert(gtsam::Symbol{'R', 4}, Rot3{wave::randomQuaternion<double>()});
            values.insert(gtsam::Symbol{'R', 5}, Rot3{wave::randomQuaternion<double>()});
            values.insert(gtsam::Symbol{'R', 6}, Rot3{wave::randomQuaternion<double>()});
            values.insert(gtsam::Symbol{'R', 7}, Rot3{wave::randomQuaternion<double>()});
            values.insert(gtsam::Symbol{'R', 8}, Rot3{wave::randomQuaternion<double>()});
            values.insert(gtsam::Symbol{'R', 9}, Rot3{wave::randomQuaternion<double>()});
            values.insert(gtsam::Symbol{'R', 10}, Rot3{wave::randomQuaternion<double>()});
            values.insert(gtsam::Symbol{'p', 1}, Point3{Eigen::Vector3d::Random()});
            vv.push_back(values);
        }
    }
};

BENCHMARK_F(RotateChain, Gtsam1)(benchmark::State &state) {
    for (auto _ : state) {
        Expression<Rot3> R1_{'R', 1};
        Expression<Point3> p1_{'p', 1};
        Expression<Point3> p2_ = rotate(R1_, p1_);
        std::vector<gtsam::Matrix> jacs(2);

        for (auto i = N; i-- > 0;) {
            Point3 p2 = p2_.value(vv[i], jacs);

            benchmark::DoNotOptimize(p2);
            benchmark::DoNotOptimize(jacs.data());
        }
    }
}

BENCHMARK_F(RotateChain, Gtsam2)(benchmark::State &state) {
    const Expression<Rot3> R1_{Rot3{}}, R2_{'R', 2};
    const Expression<Point3> p1_{'p', 1};
    const Expression<Point3> p2_ = rotate(R1_ * R2_, p1_);

    std::vector<gtsam::Matrix> jacs(3);

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Point3 p2 = p2_.value(vv[i], jacs);

            benchmark::DoNotOptimize(p2);
            benchmark::DoNotOptimize(jacs);
        }
    }
}

BENCHMARK_F(RotateChain, Gtsam3)(benchmark::State &state) {
    const Expression<Rot3> R1_{'R', 1}, R2_{'R', 2}, R3_{'R', 3};
    const Expression<Point3> p1_{'p', 1};
    const Expression<Point3> p2_ = rotate(R1_ * R2_ * R3_, p1_);

    std::vector<gtsam::Matrix> jacs(4);

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Point3 p2 = p2_.value(vv[i], jacs);

            benchmark::DoNotOptimize(p2);
            benchmark::DoNotOptimize(jacs);
        }
    }
}

BENCHMARK_F(RotateChain, Gtsam4)(benchmark::State &state) {
    const Expression<Rot3> R1_{'R', 1}, R2_{'R', 2}, R3_{'R', 3}, R4_{'R', 4};
    const Expression<Point3> p1_{'p', 1};
    const Expression<Point3> p2_ = rotate(R1_ * R2_ * R3_ * R4_, p1_);

    std::vector<gtsam::Matrix> jacs(5);

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Point3 p2 = p2_.value(vv[i], jacs);

            benchmark::DoNotOptimize(p2);
            benchmark::DoNotOptimize(jacs);
        }
    }
}

BENCHMARK_F(RotateChain, Gtsam5)(benchmark::State &state) {
    const Expression<Rot3> R1_{'R', 1}, R2_{'R', 2}, R3_{'R', 3}, R4_{'R', 4},
      R5_{'R', 5};
    const Expression<Point3> p1_{'p', 1};
    const Expression<Point3> p2_ = rotate(R1_ * R2_ * R3_ * R4_ * R5_, p1_);

    std::vector<gtsam::Matrix> jacs(6);

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Point3 p2 = p2_.value(vv[i], jacs);

            benchmark::DoNotOptimize(p2);
            benchmark::DoNotOptimize(jacs);
        }
    }
}

BENCHMARK_F(RotateChain, Gtsam6)(benchmark::State &state) {
    const Expression<Rot3> R1_{'R', 1}, R2_{'R', 2}, R3_{'R', 3}, R4_{'R', 4},
      R5_{'R', 5}, R6_{'R', 6};
    const Expression<Point3> p1_{'p', 1};
    const Expression<Point3> p2_ = rotate(R1_ * R2_ * R3_ * R4_ * R5_ * R6_, p1_);

    std::vector<gtsam::Matrix> jacs(7);

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Point3 p2 = p2_.value(vv[i], jacs);

            benchmark::DoNotOptimize(p2);
            benchmark::DoNotOptimize(jacs);
        }
    }
}

BENCHMARK_F(RotateChain, Gtsam7)(benchmark::State &state) {
    const Expression<Rot3> R1_{'R', 1}, R2_{'R', 2}, R3_{'R', 3}, R4_{'R', 4},
      R5_{'R', 5}, R6_{'R', 6}, R7_{'R', 7};
    const Expression<Point3> p1_{'p', 1};
    const Expression<Point3> p2_ = rotate(R1_ * R2_ * R3_ * R4_ * R5_ * R6_ * R7_, p1_);

    std::vector<gtsam::Matrix> jacs(8);

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Point3 p2 = p2_.value(vv[i], jacs);

            benchmark::DoNotOptimize(p2);
            benchmark::DoNotOptimize(jacs);
        }
    }
}

BENCHMARK_F(RotateChain, Gtsam8)(benchmark::State &state) {
    const Expression<Rot3> R1_{'R', 1}, R2_{'R', 2}, R3_{'R', 3}, R4_{'R', 4},
      R5_{'R', 5}, R6_{'R', 6}, R7_{'R', 7}, R8_{'R', 8};
    const Expression<Point3> p1_{'p', 1};
    const Expression<Point3> p2_ =
      rotate(R1_ * R2_ * R3_ * R4_ * R5_ * R6_ * R7_ * R8_, p1_);

    std::vector<gtsam::Matrix> jacs(9);

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Point3 p2 = p2_.value(vv[i], jacs);

            benchmark::DoNotOptimize(p2);
            benchmark::DoNotOptimize(jacs);
        }
    }
}


BENCHMARK_F(RotateChain, Gtsam9)(benchmark::State &state) {
    const Expression<Rot3> R1_{'R', 1}, R2_{'R', 2}, R3_{'R', 3}, R4_{'R', 4},
      R5_{'R', 5}, R6_{'R', 6}, R7_{'R', 7}, R8_{'R', 8}, R9_{'R', 9};
    const Expression<Point3> p1_{'p', 1};
    const Expression<Point3> p2_ =
      rotate(R1_ * R2_ * R3_ * R4_ * R5_ * R6_ * R7_ * R8_ * R9_, p1_);

    std::vector<gtsam::Matrix> jacs(10);

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Point3 p2 = p2_.value(vv[i], jacs);

            benchmark::DoNotOptimize(p2);
            benchmark::DoNotOptimize(jacs);
        }
    }
}


BENCHMARK_F(RotateChain, Gtsam10)(benchmark::State &state) {
    const Expression<Rot3> R1_{'R', 1}, R2_{'R', 2}, R3_{'R', 3}, R4_{'R', 4},
      R5_{'R', 5}, R6_{'R', 6}, R7_{'R', 7}, R8_{'R', 8}, R9_{'R', 9}, R10_{'R', 10};
    const Expression<Point3> p1_{'p', 1};
    const Expression<Point3> p2_ =
      rotate(R1_ * R2_ * R3_ * R4_ * R5_ * R6_ * R7_ * R8_ * R9_ * R10_, p1_);

    std::vector<gtsam::Matrix> jacs(11);

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Point3 p2 = p2_.value(vv[i], jacs);

            benchmark::DoNotOptimize(p2);
            benchmark::DoNotOptimize(jacs);
        }
    }
}

WAVE_BENCHMARK_MAIN()
