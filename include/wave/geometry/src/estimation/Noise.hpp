/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_NOISE_HPP
#define WAVE_GEOMETRY_NOISE_HPP

namespace wave {

/**
 * Gaussian noise with full covariance matrix
 * @tparam Leaf the leaf expression being described (e.g. RotationMd)
 */
template <typename Leaf>
class FullNoise {
    using MatrixType = BlockMatrix<Leaf, Leaf>;

 private:
    /** Constructs with the given covariance matrix */
    template <typename OtherDerived>
    explicit FullNoise(const Eigen::MatrixBase<OtherDerived> &cov)
        : cov_{cov.derived()},
          // Pre-calculate inverse sqrt covariance, used in normalization
          inv_sqrt_cov_{
            Eigen::SelfAdjointEigenSolver<MatrixType>{cov_}.operatorInverseSqrt()} {
        EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(MatrixType, OtherDerived)
    }

    template <typename OtherDerived>
    explicit FullNoise(Eigen::MatrixBase<OtherDerived> &&cov)
        : cov_{std::move(cov.derived())},
          // Pre-calculate inverse sqrt covariance, used in normalization
          inv_sqrt_cov_{
            Eigen::SelfAdjointEigenSolver<MatrixType>{cov_}.operatorInverseSqrt()} {
        EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(MatrixType, OtherDerived)
    }

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Constructs with the given covariance matrix */
    template <typename OtherDerived>
    static FullNoise FromCovariance(const Eigen::MatrixBase<OtherDerived> &cov) {
        return FullNoise{cov.derived()};
    }

    /** Constructs with the given covariance matrix */
    template <typename OtherDerived>
    static FullNoise FromCovariance(Eigen::MatrixBase<OtherDerived> &&cov) {
        return FullNoise{std::move(cov.derived())};
    }

    auto covariance() const & -> const MatrixType & {
        return this->cov_;
    }

    auto covariance() && -> MatrixType && {
        return std::move(*this).cov_;
    }

    auto inverseSqrtCov() const & -> const MatrixType & {
        return this->inv_sqrt_cov_;
    }

    auto inverseSqrtCov() && -> MatrixType && {
        return std::move(*this).inv_sqrt_cov_;
    }

 private:
    const MatrixType cov_;
    const MatrixType inv_sqrt_cov_;
};

/**
 * Gaussian noise with diagonal covariance matrix
 * @tparam Leaf the leaf expression being described (e.g. RotationMd)
 */
template <typename Leaf>
class DiagonalNoise {
    using MatrixType = DiagonalBlockMatrix<Leaf>;
    using Scalar = internal::scalar_t<Leaf>;
    enum : int { Size = internal::traits<Leaf>::TangentSize };

 private:
    /** Constructs with the given precalculated diagonal matrices */
    template <typename D1, typename D2>
    explicit DiagonalNoise(const Eigen::DiagonalBase<D1> &cov,
                           const Eigen::DiagonalBase<D2> &inv_sqrt_cov)
        : cov_{std::move(cov.derived())},
          inv_sqrt_cov_{std::move(inv_sqrt_cov.derived())} {
        EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(MatrixType, D1)
        EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(MatrixType, D2)
    }


 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Constructs from the given vector of standard deviations */
    template <typename OtherDerived>
    static DiagonalNoise FromStdDev(const Eigen::MatrixBase<OtherDerived> &stddev) {
        return DiagonalNoise{stddev.derived().cwiseProduct(stddev.derived()).asDiagonal(),
                             stddev.derived().cwiseInverse().asDiagonal()};
    }

    /** Construct from coefficients of standard deviation given as parameters */
    template <typename... S,
              std::enable_if_t<sizeof...(S) == Size and
                                 tmp::conjunction<std::is_convertible<S, Scalar>...>{},
                               bool> = true>
    static DiagonalNoise FromStdDev(S... stddev) {
        using Vec = Eigen::Matrix<Scalar, Size, 1>;
        return DiagonalNoise{Vec{(Scalar{stddev} * Scalar{stddev})...}.asDiagonal(),
                             Vec{(1 / Scalar{stddev})...}.asDiagonal()};
    }

    auto covariance() const & -> const MatrixType & {
        return this->cov_;
    }

    auto covariance() && -> MatrixType && {
        return std::move(*this).cov_;
    }

    auto inverseSqrtCov() const & -> const MatrixType & {
        return this->inv_sqrt_cov_;
    }

    auto inverseSqrtCov() && -> MatrixType && {
        return std::move(*this).inv_sqrt_cov_;
    }

 private:
    const MatrixType cov_;
    const MatrixType inv_sqrt_cov_;
};


}  // namespace wave

#endif  // WAVE_GEOMETRY_NOISE_HPP
